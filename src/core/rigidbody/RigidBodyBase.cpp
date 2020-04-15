//=================================================================================================
/*!
 *  \file src/core/rigidbody/RigidBodyBase.cpp
 *  \brief Source file for the RigidBodyBase class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#include <pe/core/MPI.h>
#include <pe/core/rigidbody/RigidBodyBase.h>
#include <pe/system/SleepMode.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the RigidBodyBase class.
 *
 * \param body The body ID of this rigid body.
 */
RigidBodyBase::RigidBodyBase( BodyID body )
   : CCDBT( body )            // Initialization of the coarse collision detection body trait base class
   , FCDBT()                  // Initialization of the fine collision detection body trait base class
   , BGBT()                   // Initialization of the batch generation body trait base class
   , CRBT()                   // Initialization of the collision response body trait base class
   , BE()                     // Initialization of the user-defined body trait base class
   , fixed_( false )          // Fixation flag
   , awake_( true )           // Sleep mode flag
   , mass_( 0 )               // Total mass of the rigid body
   , invMass_( 0 )            // Inverse total mass of the rigid body
   , motion_(sleepThreshold)  // The current motion of the rigid body
   , gpos_()                  // Global position of the center of mass
   , rpos_()                  // Relative position within the body frame of the superordinate body
   , v_()                     // Linear velocity
   , w_()                     // Angular velocity
   , force_()                 // Total force
   , torque_()                // Total torque
   , I_()                     // Moment of inertia
   , Iinv_()                  // Inverse moment of inertia
   , q_()                     // Orientation of the body frame
   , R_()                     // Rigid body rotation
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the RigidBodyBase class.
 */
RigidBodyBase::~RigidBodyBase()
{}
//*************************************************************************************************

} // namespace pe
