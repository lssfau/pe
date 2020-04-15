//=================================================================================================
/*!
 *  \file src/core/joint/Joint.cpp
 *  \brief Source file for the Joint class
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

#include <pe/core/CollisionSystem.h>
#include <pe/core/Configuration.h>
#include <pe/core/joint/Joint.h>
#include <pe/core/joint/JointStorage.h>
#include <pe/core/MPI.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Joint class.
 *
 * \param type The type of the joint.
 * \param rows Number of rows the joint occupies in Jacobian matrices.
 * \param body1 The first body to which the joint is attached.
 * \param body2 The second body to which the joint is attached.
 * \param scale Scaling parameter for visualization purposes \f$[0..1] \f$.
 * \param sid Unique system ID.
 */
Joint::Joint( JointType type, size_t rows, BodyID body1, BodyID body2, real scale, id_t sid )
   : Parent( type, rows, body1, body2, scale, sid )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Detaching the joint.
 *
 * \return void
 *
 * This function detaches the joint from all rigid bodies it is currently attached to
 * and automatically destroys the joint.
 */
void detach( JointID joint )
{
   typedef JointStorage<Config> JS;

   // WARNING: Using friend relationship to get non-constant reference of joint storage.
   JS& jointstorage( theCollisionSystem()->jointstorage_ );

   // Removing the joint from the joint storage
   const JS::Iterator pos( jointstorage.find( joint ) );
   jointstorage.remove( pos );

   delete joint;
}
//*************************************************************************************************

} // namespace pe
