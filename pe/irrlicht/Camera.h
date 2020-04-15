//=================================================================================================
/*!
 *  \file pe/irrlicht/Camera.h
 *  \brief Header file for an Irrlicht camera
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

#ifndef _PE_IRRLICHT_CAMERA_H_
#define _PE_IRRLICHT_CAMERA_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/irrlicht/Types.h>


namespace pe {

namespace irrlicht {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup irrlicht_camera Camera
 * \ingroup irrlicht
 */
/*!\brief An Irrlicht camera.
 * \ingroup irrlicht_camera
 *
 * The Camera class represents an Irrlicht camera. The class acts as a wrapper for an Irrlicht
 * camera scene node and offers most of the available features.
 */
class Camera
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Camera( ::irr::scene::ICameraSceneNode* camera );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setPosition( float px, float py, float pz );
   void setTarget( float px, float py, float pz );
   void setUpVector( float vx, float vy, float vz );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   ::irr::scene::ICameraSceneNode* camera_;  //!< Irrlicht camera handle.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The constructor of the Irrlicht Camera class.
 *
 * \param camera The Irrlicht camera handle
 */
inline Camera::Camera( ::irr::scene::ICameraSceneNode* camera )
   : camera_(camera)  // Irrlicht camera handle
{}
//*************************************************************************************************

} // namespace irrlicht

} // namespace pe

#endif
