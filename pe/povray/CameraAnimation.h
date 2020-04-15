//=================================================================================================
/*!
 *  \file pe/povray/CameraAnimation.h
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

#ifndef _PE_POVRAY_CAMERAANIMATION_H_
#define _PE_POVRAY_CAMERAANIMATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/Vector3.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Inferface class for all camera animations.
 * \ingroup povray_camera
 *
 * The CameraAnimation class is the interface for all camera animation classes. In order to
 * animate the POV-Ray camera, it is necessary to inherit from this class and to override the
 * updateLocation(), the updateFocus() and the updateSky() functions (as required). The
 * derived class can then be used to animate the POV-Ray camera:

   \code
   using namespace pe::povray;

   class MyCameraAnimation : public CameraAnimation
   {
      ... // implements the updateLocation(), updateFocus() and updateSky() functions
   };

   int main()
   {
      CameraID camera = theCamera();              // Returns a handle to the POV-Ray camera
      camera->animate( MyCameraAnimation(...) );  // Animates the POV-Ray camera
   }
   \endcode
 */
class PE_PUBLIC CameraAnimation
{
public:
   //**Constructors********************************************************************************
   // No explicitly declared constructor.
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~CameraAnimation() = 0;
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   virtual void updateLocation( Vec3& location );
   virtual void updateFocus   ( Vec3& focus    );
   virtual void updateSky     ( Vec3& sky      );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
