//=================================================================================================
/*!
 *  \file src/povray/CameraAnimation.cpp
 *  \brief Inferface class for all camera animations
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

#include <pe/povray/CameraAnimation.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor of the CameraAnimation class.
 */
CameraAnimation::~CameraAnimation()
{}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation for the updateLocation() function.
 *
 * \param location The global location to be updated.
 * \return void
 *
 * This function is the default implementation for the updateLocation() function. Per default
 * it does not change the given global location. In case the location is not supposed to be
 * animated, this behavior can be used in the derived classes without the requirement to
 * override this function.
 */
void CameraAnimation::updateLocation( Vec3& /*location*/ )
{
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation for the updateFocus() function.
 *
 * \param focus The focus point to be updated.
 * \return void
 *
 * This function is the default implementation for the updateFocus() function. Per default
 * it does not change the given focus point. In case the focus point is not supposed to be
 * animated, this behavior can be used in the derived classes without the requirement to
 * override this function.
 */
void CameraAnimation::updateFocus( Vec3& /*focus*/ )
{
   return;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation for the updateSky() function.
 *
 * \param sky The sky vector to be updated.
 * \return void
 *
 * This function is the default implementation for the updateSky() function. Per default
 * it does not change the given sky vector. In case the sky vector is not supposed to be
 * animated, this behavior can be used in the derived classes without the requirement to
 * override this function.
 */
void CameraAnimation::updateSky( Vec3& /*sky*/ )
{
   return;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
