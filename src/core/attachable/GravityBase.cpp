//=================================================================================================
/*!
 *  \file src/core/attachable/GravityBase.cpp
 *  \brief Source file for the GravityBase class
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

#include <pe/core/attachable/GravityBase.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the GravityBase class.
 *
 * \param sid The unique system-specific ID of the gravity force generator.
 * \param body The rigid body the force generator is attached to.
 * \param gravity The exerted gravity.
 */
GravityBase::GravityBase( id_t sid, BodyID body, const Vec3& gravity )
   : Parent  ( gravityType, sid, false )  // Initialization of the parent class
   , gravity_( gravity )                  // The exerted gravity
{
   // Storing the attached rigid body
   bodies_.pushBack( body );
   pe_INTERNAL_ASSERT( bodies_.size() == 1, "Invalid number of attached bodies" );

   // Registering the gravity force generator with the attached rigid body
   body->registerAttachable( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the GravityBase class.
 */
GravityBase::~GravityBase()
{
   // Deregistering the gravity force generator from the attached rigid body
   bodies_[0]->deregisterAttachable( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the gravity force generator visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the gravity visible, \a false to make it invisible.
 * \return void
 */
void GravityBase::setVisible( bool /*visible*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the gravity of the gravity force generator.
 *
 * \param gravity The new exerted gravity.
 * \return void
 */
void GravityBase::setGravity( const Vec3& gravity )
{
   gravity_ = gravity;
}
//*************************************************************************************************

} // namespace pe
