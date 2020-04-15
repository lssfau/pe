//=================================================================================================
/*!
 *  \file src/core/attachable/SpringBase.cpp
 *  \brief Source file for the SpringBase class
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
#include <pe/core/attachable/Spring.h>
#include <pe/core/MPI.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the SpringBase class.
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
SpringBase::SpringBase( id_t sid, BodyID body1, BodyID body2, const Vec3& anchor1, const Vec3& anchor2,
                        real stiffness, real damping, real length, bool visible )
   : Parent    ( springType, sid, visible )  // Initialization of the parent class
   , anchor1_  ( anchor1   )                 // The first body's anchor point in body relative coordinates
   , anchor2_  ( anchor2   )                 // The second body's anchor point in body relative coordinates
   , stiffness_( stiffness )                 // Stiffness of the spring
   , damping_  ( damping   )                 // Damping of the spring
   , length_   ( length    )                 // The length in non-deformed state
{
   // Checking the anchor points, the stiffness, the damping and the rest length of the spring.
   // Since the spring constructor is never directly called but only used in a small number
   // of functions that already check the spring arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( body1->containsRelPoint( anchor1 ), "Invalid anchor point for body 1" );
   pe_INTERNAL_ASSERT( body2->containsRelPoint( anchor2 ), "Invalid anchor point for body 2" );
   pe_INTERNAL_ASSERT( stiffness >  real(0), "Invalid spring stiffness" );
   pe_INTERNAL_ASSERT( damping   >= real(0), "Invalid spring damping"   );
   pe_INTERNAL_ASSERT( length    >  real(0), "Invalid rest length"      );

   // Storing the two attached rigid bodies
   pe_INTERNAL_ASSERT( bodies_.size() == 0, "Invalid number of attached bodies" );
   bodies_.reserve( 2 );
   bodies_.pushBack( body1 );
   bodies_.pushBack( body2 );
   pe_INTERNAL_ASSERT( bodies_.size() == 2, "Invalid number of attached bodies" );

   // Registering the spring with the two attached rigid bodies
   body1->registerAttachable( this );
   body2->registerAttachable( this );
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
SpringBase::~SpringBase()
{
   // Deregistering the spring from the attached rigid body
   bodies_[0]->deregisterAttachable( this );
   bodies_[1]->deregisterAttachable( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the stiffness of the spring.
 *
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \return void
 */
void SpringBase::setStiffness( real stiffness )
{
   if( stiffness <= real(0) )
      throw std::invalid_argument( "Invalid spring stiffness" );

   stiffness_ = stiffness;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the damping of the spring.
 *
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \return void
 */
void SpringBase::setDamping( real damping )
{
   if( damping < real(0) )
      throw std::invalid_argument( "Invalid spring damping" );

   damping_ = damping;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the rest length of the spring.
 *
 * \param length The rest length of the spring \f$ (0..\infty) \f$.
 * \return void
 */
void SpringBase::setLength( real length )
{
   if( length <= real(0) )
      throw std::invalid_argument( "Invalid rest length" );

   length_ = length;
}
//*************************************************************************************************

} // namespace pe
