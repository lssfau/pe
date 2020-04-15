//=================================================================================================
/*!
 *  \file src/core/attachable/Spring.cpp
 *  \brief Source file for the Spring class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *
 *  This file is part of pe.
 *
 *  pe is free software: you can redistribute it and/or modify it under the terms of the GNU
 *  General Public License as published by the Free Software Foundation, either version 3 of the
 *  License, or (at your option) any later version.
 *
 *  pe is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along with pe. If not,
 *  see <http://www.gnu.org/licenses/>.
 */
//=================================================================================================


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/Configuration.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/TimeStep.h>
#include <pe/core/Visualization.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/Assert.h>
#include <pe/util/logging/DetailSection.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Spring class.
 *
 * \param sid The unique system-specific ID of the spring.
 * \param body1 The first rigid body anchored to the spring.
 * \param body2 The second rigid body anchored to the spring.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param length The rest length of the spring \f$ (0..\infty) \f$.
 * \param visible Specifies if the spring is visible in a visualization.
 */
Spring::Spring( id_t sid, BodyID body1, BodyID body2, const Vec3& anchor1, const Vec3& anchor2,
                real stiffness, real damping, real length, bool visible )
   : Parent( sid, body1, body2, anchor1, anchor2, stiffness, damping, length, visible )  // Initialization of the parent class
{
   // Registering the spring for visualization
   Visualization::add( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Spring class.
 */
Spring::~Spring()
{
   // Deregistering the spring from visualization
   Visualization::remove( this );

   // Logging the destruction of the spring
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed spring " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the spring visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the spring visible, \a false to make it invisible.
 * \return void
 */
void Spring::setVisible( bool visible )
{
   if( visible ^ visible_ ) {
      visible_ = visible;
      Visualization::changeVisibility( this );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  MPI FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the spring force generator is remote or not.
 *
 * \return \a true in case the spring is remote, \a false if not.
 *
 * This function returns whether the spring force generator is remote or not. In case the spring
 * is attached to two remote rigid bodies the function returns \a true. Otherwise, if the spring
 * is attached to at least one local rigid body, the function returns \a false.
 */
bool Spring::isRemote() const
{
   return bodies_[0]->isRemote() && bodies_[1]->isRemote();
}
//*************************************************************************************************




//=================================================================================================
//
//  FORCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of forces caused by the spring-damper system.
 *
 * \return void
 *
 * This function estimates and exerts the forces caused by the current deformation of the
 * spring-damper system. The forces are calculated according to the following two formulas:

         \f[ F = -k \cdot x - \delta \cdot v \f]
         \f[ m \cdot \ddot{x} + \delta \cdot \dot{x} + k \cdot x = 0, \f]

 * where \a F is the acting force, \a x is the displacement or the anchor points from the rest
 * length of the spring, \a v is the relative velocity between the two anchor points, \a k is
 * the stiffness of the spring and \f$ \delta \f$ is the damping factor.
 */
void Spring::applyForce()
{
   // Early exit in case both bodies are remote
   if( bodies_[0]->isRemote() && bodies_[1]->isRemote() ) return;

   // Calculating the current position of the anchor points
   const Vec3 pos1( bodies_[0]->pointFromBFtoWF( anchor1_ ) );
   const Vec3 pos2( bodies_[1]->pointFromBFtoWF( anchor2_ ) );

   // Calculating the current distance of the spring anchors in global coordinates
   const Vec3 dp( pos1 - pos2 );

   const real length( dp.length() );

   // Estimation of the spring direction's unit vector
   const Vec3 d( dp * (real(1)/length) );

   // Calculation of the total relative velocity in global coordinates
   Vec3 dv( bodies_[0]->velFromBF( anchor1_ ) - bodies_[1]->velFromBF( anchor2_ ) );

   // Calculation of the projection of the relative velocity onto the distance vector
   dv = ( trans(dv) * d ) * d;

   // Corrected spring damping
   const real factor( damping_ + real(0.5) * stiffness_ * TimeStep::size() );

   // Calculation of force (stable implementation)
   const Vec3 force( - stiffness_ * ( length - length_ ) * d     // Proportional part
                     - factor * dv                           );  // Differential part

   // Exerting force on the bodies
   if( !bodies_[0]->isRemote() && !bodies_[0]->isFixed() ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "      Spring " << sid_ << ": Applying force " << force << " on body " << bodies_[0]->getSystemID();
      }
      bodies_[0]->addForceAtPos(  force, pos1 );
   }
   if( !bodies_[1]->isRemote() && !bodies_[1]->isFixed() ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "      Spring " << sid_ << ": Applying force " << -force << " on body " << bodies_[1]->getSystemID();
      }
      bodies_[1]->addForceAtPos( -force, pos2 );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a spring.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void Spring::print( std::ostream& os ) const
{
   os << " Spring between rigid body " << bodies_[0]->getID() << " and " << bodies_[1]->getID() << "\n";

   if( verboseMode ) {
      os << "   1. anchor point: global   = " << bodies_[0]->pointFromBFtoWF( anchor1_ ) << "\n"
         << "                    relative = " << anchor1_ << "\n"
         << "   2. anchor point: global   = " << bodies_[1]->pointFromBFtoWF( anchor2_ ) << "\n"
         << "                    relative = " << anchor2_ << "\n";
   }

   os << "   Stiffness = " << stiffness_ << "\n"
      << "   Damping   = " << damping_   << "\n"
      << "   Length    = " << length_    << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  SPRING SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new spring between two rigid bodies.
 * \ingroup force_generator
 *
 * \param body1 The first rigid body anchored to the spring.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param body2 The second rigid body anchored to the spring.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param length The rest length of the spring \f$ (0..\infty) \f$.
 * \param visible Specifies if the spring is visible in a visualization.
 * \return Handle for the new spring.
 * \exception std::logic_error Invalid setup of spring inside a global section.
 * \exception std::invalid_argument Cannot attach spring to global rigid body.
 * \exception std::invalid_argument Invalid anchor point for body 1.
 * \exception std::invalid_argument Invalid anchor point for body 2.
 * \exception std::invalid_argument Invalid spring stiffness.
 * \exception std::invalid_argument Invalid spring damping.
 * \exception std::invalid_argument Invalid rest length.
 *
 * This function creates a spring between the two rigid bodies \a body1 and \a body2. \a anchor1
 * is the anchor point for \a body1, given in coordinates relative to the body frame of \a body1.
 * \a anchor2 is the anchor point for \a body2 and specified in coordinates relative to the body
 * frame of \a body2. \a stiffness specifies the stiffness of the spring, whereas \a damping sets
 * the damping factor of the spring-damper system. \a length specifies the rest length of the
 * spring. In case the two given anchor points of the two attached rigid bodies have a different
 * distance from each other, the spring is initially stressed.\n
 *
 * \b Note: Springs cannot be created within a pe_GLOBAL_SECTION. The attempt to create a global
 * spring will result in a \a std::logic_error exception! Additionally, springs cannot be attached
 * to global rigid bodies. The attempt to attach a spring to a global rigid body result in a
 * \a std::invalid_argument exception!
 */
PE_PUBLIC SpringID attachSpring( BodyID body1, const Vec3& anchor1,
                       BodyID body2, const Vec3& anchor2,
                       real stiffness, real damping, real length, bool visible )
{
   // Checking whether the spring is created within a global section
   if( GlobalSection::isActive() )
      throw std::logic_error( "Invalid setup of spring inside a global section" );

   // Checking the global flags of the two bodies
   if( body1->isGlobal() || body2->isGlobal() )
      throw std::invalid_argument( "Cannot attach spring to global rigid body" );

   // Checking whether the bodies contain the anchor points
   if( !( body1->containsRelPoint( anchor1 ) ) )
      throw std::invalid_argument( "Invalid anchor point for body 1" );
   if( !( body2->containsRelPoint( anchor2 ) ) )
      throw std::invalid_argument( "Invalid anchor point for body 2" );

   // Checking the spring stiffness, damping and rest length
   if( stiffness <= real(0) )
      throw std::invalid_argument( "Invalid spring stiffness" );
   if( damping < real(0) )
      throw std::invalid_argument( "Invalid spring damping" );
   if( length <= real(0) )
      throw std::invalid_argument( "Invalid rest length" );

   // Creating a new spring
   const id_t sid( UniqueID<Attachable>::create() );
   SpringID spring = new Spring( sid, body1, body2, anchor1, anchor2,
                                 stiffness, damping, length, visible );

   // WARNING: Using friend relationship to add spring to attachable storage.
   theCollisionSystem()->attachablestorage_.add( spring );

   // Logging the successful setup of the spring
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created spring " << sid << " between "
          << body1->getType() << " " << body1->getSystemID() << " and "
          << body2->getType() << " " << body2->getSystemID() << "\n"
          << "   Anchor point 1 = " << anchor1 << "\n"
          << "   Anchor point 2 = " << anchor2 << "\n"
          << "   stiffness      = " << stiffness << "\n"
          << "   damping        = " << damping << "\n"
          << "   length         = " << length;
   }

   return spring;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new spring between two rigid bodies.
 * \ingroup force_generator
 *
 * \param body1 The first rigid body anchored to the spring.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param body2 The second rigid body anchored to the spring.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param length The rest length of the spring \f$ (0..\infty) \f$.
 * \return Handle for the new spring.
 * \exception std::logic_error Invalid setup of spring inside a global section.
 * \exception std::invalid_argument Cannot attach spring to global rigid body.
 * \exception std::invalid_argument Invalid anchor point for body 1.
 * \exception std::invalid_argument Invalid anchor point for body 2.
 * \exception std::invalid_argument Invalid spring stiffness.
 * \exception std::invalid_argument Invalid spring damping.
 * \exception std::invalid_argument Invalid rest length.
 *
 * This function creates a (visible) spring between the two rigid bodies \a body1 and \a body2.
 * \a anchor1 is the anchor point for \a body1, given in coordinates relative to the body frame of
 * \a body1. \a anchor2 is the anchor point for \a body2 and specified in coordinates relative to
 * the body frame of \a body2. \a stiffness specifies the stiffness of the spring, whereas \a damping
 * sets the damping factor of the spring-damper system. \a length specifies the rest length of the
 * spring. In case the two given anchor points of the two attached rigid bodies have a different
 * distance from each other, the spring is initially stressed.\n
 *
 * \b Note: Springs cannot be created within a pe_GLOBAL_SECTION. The attempt to create a global
 * spring will result in a \a std::logic_error exception! Additionally, springs cannot be attached
 * to global rigid bodies. The attempt to attach a spring to a global rigid body result in a
 * \a std::invalid_argument exception!
 */
PE_PUBLIC SpringID attachSpring( BodyID body1, const Vec3& anchor1,
                                 BodyID body2, const Vec3& anchor2,
                                 real stiffness, real damping, real length )
{
   return attachSpring( body1, anchor1, body2, anchor2, stiffness, damping, length, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote spring.
 * \ingroup force_generator
 *
 * \param sid The unique system-specific ID of the spring.
 * \param body1 The first rigid body anchored to the spring.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param body2 The second rigid body anchored to the spring.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param length The rest length of the spring \f$ (0..\infty) \f$.
 * \param visible Specifies if the spring is visible in a visualization.
 * \return Handle for the new spring.
 *
 * This function instantiates a copy of a spring with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of a spring residing on a remote
 * MPI process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
SpringID instantiateSpring( id_t sid,
                            BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2,
                            real stiffness, real damping, real length, bool visible )
{
   typedef BodyStorage<Config>  BS;

   // Checking the anchor points
   pe_INTERNAL_ASSERT( body1->containsRelPoint( anchor1 ), "Invalid anchor point for body 1" );
   pe_INTERNAL_ASSERT( body2->containsRelPoint( anchor2 ), "Invalid anchor point for body 2" );

   // Checking the stiffness, damping and rest length
   pe_INTERNAL_ASSERT( stiffness >  real(0), "Invalid spring stiffness"   );
   pe_INTERNAL_ASSERT( damping   >= real(0), "Invalid spring damping"     );
   pe_INTERNAL_ASSERT( length    >  real(0), "Invalid spring rest length" );

   // Instantiating the spring
   SpringID spring = new Spring( sid, body1, body2, anchor1, anchor2,
                                 stiffness, damping, length, visible );

   // WARNING: Using friend relationship to add spring to attachable storage.
   theCollisionSystem()->attachablestorage_.add( spring );

   // Logging the successful instantiation of the spring
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated spring " << sid << " between "
          << body1->getType() << " " << body1->getSystemID() << " and "
          << body2->getType() << " " << body2->getSystemID() << "\n"
          << "   Anchor point 1 = " << anchor1 << "\n"
          << "   Anchor point 2 = " << anchor2 << "\n"
          << "   Stiffness      = " << stiffness << "\n"
          << "   Damping        = " << damping << "\n"
          << "   Length         = " << length;
   }

   return spring;
}
//*************************************************************************************************

} // namespace pe
