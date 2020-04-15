//=================================================================================================
/*!
 *  \file src/irrlicht/Camera.cpp
 *  \brief Source file for an Irrlicht camera
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


#if HAVE_IRRLICHT

//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/irrlicht/Camera.h>


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <irrlicht/irrlicht.h>


namespace pe {

namespace irrlicht {

//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the position of the camera.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 */
void Camera::setPosition( float px, float py, float pz )
{
   using ::irr::core::vector3df;
   camera_->setPosition( vector3df( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the target (focus point) of the camera.
 *
 * \param px The x-component of the global focus point.
 * \param py The y-component of the global focus point.
 * \param pz The z-component of the global focus point.
 * \return void
 */
void Camera::setTarget( float px, float py, float pz )
{
   using ::irr::core::vector3df;
   camera_->setTarget( vector3df( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the target (focus point) of the camera.
 *
 * \param vx The x-component of the up/sky vector.
 * \param vy The y-component of the up/sky vector.
 * \param vz The z-component of the up/sky vector.
 * \return void
 */
void Camera::setUpVector( float vx, float vy, float vz )
{
   using ::irr::core::vector3df;
   camera_->setUpVector( vector3df( vx, vy, vz ) );
}
//*************************************************************************************************

} // namespace irrlicht

} // namespace pe

#endif
