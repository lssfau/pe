//=================================================================================================
/*!
 *  \file src/core/attachable/Gravity.cpp
 *  \brief Source file for the Gravity class
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

#include <ostream>
#include <stdexcept>
#include <pe/core/attachable/Gravity.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/logging/DetailSection.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Gravity class.
 *
 * \param sid The unique system-specific ID of the gravity force generator.
 * \param body The rigid body the force generator is attached to.
 * \param gravity The exerted gravity.
 */
Gravity::Gravity( id_t sid, BodyID body, const Vec3& gravity )
   : Parent( sid, body, gravity )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Gravity class.
 */
Gravity::~Gravity()
{
   // Logging the destruction of the gravity force generator
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed gravity force generator " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  MPI FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the gravity force generator is remote or not.
 *
 * \return \a true in case the force generator is remote, \a false if not.
 *
 * This function returns whether the gravity force generator is remote or not. In case the
 * force generator is attached to a remote rigid body the function returns \a true. Otherwise
 * it returns \a false.
 */
bool Gravity::isRemote() const
{
   return bodies_[0]->isRemote();
}
//*************************************************************************************************




//=================================================================================================
//
//  FORCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Applying the gravity to the attached rigid body.
 *
 * \return void
 *
 * This function triggers the acting gravity of the gravity force generator.
 */
void Gravity::applyForce()
{
   if( !bodies_[0]->isRemote() && !bodies_[0]->isFixed() )
   {
      const Vec3 force( gravity_ * bodies_[0]->getMass() );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "      Gravity " << sid_ << ": Applying force " << force << " on body " << bodies_[0]->getSystemID();
      }

      bodies_[0]->addForce( force );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of the gravity force generator.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void Gravity::print( std::ostream& os ) const
{
   os << " Gravity attached to rigid body " << bodies_[0]->getID() << "\n"
      << "   gravity = " << gravity_ << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GRAVITY SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new gravity force generator.
 * \ingroup force_generator
 *
 * \param body The rigid body the force generator is attached to.
 * \param gravity The exerted gravity (in global coordinates).
 * \return Handle for the new gravity force generator.
 * \exception std::logic_error Invalid setup of gravity force generator inside a global section.
 * \exception std::invalid_argument Cannot attach gravity force generator to global rigid body.
 *
 * \b Note: Gravity force generators cannot be created within a pe_GLOBAL_SECTION. The attempt
 * to create a global gravity force generator will result in a \a std::logic_error exception!
 * Additionally, gravity force generators cannot be attached to global rigid bodies. The attempt
 * to attach a spring to a global rigid body result in a \a std::invalid_argument exception!
 */
PE_PUBLIC GravityID attachGravity( BodyID body, const Vec3& gravity )
{
   // Checking whether the gravity force generator is created within a global section
   if( GlobalSection::isActive() )
      throw std::logic_error( "Invalid setup of gravity force generator inside a global section" );

   // Checking the global flag of the given body
   if( body->isGlobal() )
      throw std::invalid_argument( "Cannot attach gravity force generator to global rigid body" );

   // Creating a new gravity force generator
   const id_t sid( UniqueID<Attachable>::create() );
   const GravityID gfg = new Gravity( sid, body, gravity );

   // WARNING: Using friend relationship to add gravity to attachable storage.
   theCollisionSystem()->attachablestorage_.add( gfg );

   // Logging the successful setup of the gravity force generator
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created gravity force generator " << sid << " on " << body->getType()
          << " " << body->getSystemID() << "\n"
          << "   Gravity = " << gravity;
   }

   return gfg;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote spring.
 * \ingroup force_generator
 *
 * \param sid The unique system-specific ID of the gravity force generator.
 * \param body The rigid body the force generator is attached to.
 * \param gravity The exerted gravity.
 * \return Handle for the new gravity force generator.
 *
 * This function instantiates a copy of a gravity force generator with a certain system-specific
 * ID. For instance, it is used to locally instantiate a copy of a gravity force generator
 * residing on a remote MPI process. This function must NOT be called explicitly, but is reserved
 * for internal use only!
 */
GravityID instantiateGravity( id_t sid, BodyID body, const Vec3& gravity )
{
   // Instantiating the gravity force generator
   GravityID gfg = new Gravity( sid, body, gravity );

   // WARNING: Using friend relationship to add gravity to attachable storage.
   theCollisionSystem()->attachablestorage_.add( gfg );

   // Logging the successful instantiation of the gravity force generator
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated gravity force generator " << sid << " on " << body->getType()
          << " " << body->getSystemID() << "\n"
          << "   Gravity = " << gravity;
   }

   return gfg;
}
//*************************************************************************************************

} // namespace pe
