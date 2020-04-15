//=================================================================================================
/*!
 *  \file pe/povray/Camera.h
 *  \brief Implementation of a POV-Ray camera
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

#ifndef _PE_POVRAY_CAMERA_H_
#define _PE_POVRAY_CAMERA_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/Trigger.h>
#include <pe/math/Vector3.h>
#include <pe/povray/CameraAnimation.h>
#include <pe/povray/CameraID.h>
#include <pe/util/constraints/DerivedFrom.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/Null.h>
#include <pe/util/singleton/Singleton.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_camera Camera
 * \ingroup povray
 *
 * The POV-Ray camera is the center of a POV-Ray visualization and probably the single most
 * important element of a POV-Ray scene. Without a camera, POV-Ray is not able to create an
 * image. Additionally, the location and orientation of the camera defines what elements
 * are visible in a rendered image.\n
 * There is exactly one camera for the POV-Ray visualization. This camera is accessible via
 * the theCamera() function that returns a handle to the POV-Ray camera. The following example
 * shows how this camera can be configured:

   \code
   using namespace pe::povray;

   // Definition of a camera animation class
   class MyCameraAnimation : public CameraAnimation
   {
      ... // implements the updateLocation(), updateFocus() and updateSky() functions
   };

   // Implementation of the main() function
   int main()
   {
      ...

      // Getting a handle for the POV-Ray camera
      CameraID camera = theCamera();

      // Setting the global location of the camera. In case the camera is animated, this
      // location represents the starting point for the camera path.
      camera->setLocation( -50.0, -50.0, 50.0 );

      // Setting the focus point of the camera. In case a camera animation is specified,
      // this focus point is the starting point for the specified focus point motion.
      camera->setFocus( 0.0, 0.0, 2.0 );

      // Setting the sky vector of the camera. In case the camera is animated, this sky
      // vector is the starting point for the sky vector animation.
      camera->setSky( 0.0, 0.0, 1.0 );

      // Animating the camera. The animate() function is passed a camera animation object
      // that implements the update functions for the global location, the focus point and
      // the sky vector.
      camera->animate( MyCameraAnimation(...) );

      ... // Creating POV-Ray images with an animated camera

      // Removing the animation of the POV-Ray camera. The camera is now fixed at the last
      // location using the last focus point and sky vector 
      camera->removeAnimation();

      ... // Creating POV-Ray images with a fixed camera
   }
   \endcode
 */
/*!\brief A POV-Ray camera.
 * \ingroup povray_camera
 *
 * The Camera class represents the camera in the POV-Ray scene. The camera is the source of
 * the rays for the ray-tracing process and is therefore the single most important element
 * of a scene. Without a camera, POV-Ray is not able to create an image. Additionally, there
 * is only exactly one POV-Ray camera that is accessible via the theCamera() function.
 */
class PE_PUBLIC Camera : public Trigger
             , private Singleton<Camera,logging::Logger>
{
private:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Camera();
   //@}
   //**********************************************************************************************

public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Camera();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getLocation() const;
   inline const Vec3& getFocus() const;
   inline const Vec3& getSky() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setLocation( real x, real y, real z );
   void setLocation( const Vec3& location );
   void setFocus( real x, real y, real z );
   void setFocus( const Vec3& focus );
   void setSky( real x, real y, real z );
   void setSky( const Vec3& sky );
   //@}
   //**********************************************************************************************

   //**Animation functions*************************************************************************
   /*!\name Animation functions */
   //@{
   template< typename A >
   inline void animate( const A& animation );

   inline void removeAnimation();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void reset();
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os,                  bool newline ) const;
   void print( std::ostream& os, const char* tab, bool newline ) const;
   //@}
   //**********************************************************************************************

private:
   //**Trigger functions***************************************************************************
   /*!\name Trigger functions */
   //@{
   virtual void trigger();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 location_;               //!< The global position of the camera.
   Vec3 focus_;                  //!< The focus point of the camera.
   Vec3 sky_;                    //!< The up/sky-direction of the camera.
   CameraAnimation* animation_;  //!< The camera animation.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend CameraID theCamera();
   pe_BEFRIEND_SINGLETON;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CAMERA SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Camera setup functions */
//@{
inline CameraID theCamera();
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the POV-Ray camera.
 * \ingroup povray_camera
 *
 * \return Handle to the POV-Ray camera.
 */
inline CameraID theCamera()
{
   return Camera::instance();
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current global location of the camera.
 *
 * \return The global location.
 */
inline const Vec3& Camera::getLocation() const
{
   return location_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current focus point of the camera.
 *
 * \return The focus point of the camera.
 */
inline const Vec3& Camera::getFocus() const
{
   return focus_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current up/sky-direction of the camera.
 *
 * \return The up/sky-direction.
 */
inline const Vec3& Camera::getSky() const
{
   return sky_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resetting the POV-Ray camera.
 *
 * \return void
 *
 * This function resets the camera to the POV-Ray default values.
 */
inline void Camera::reset()
{
   location_.reset();      // Resetting the location to (0,0,0)
   focus_.set( 0, 1, 0 );  // Resetting the focus point to (0,1,0)
   sky_.set( 0, 0, 1 );    // Resetting the sky vector to (0,0,1) (the z-axis)

   // Deleting the camera animation
   if( animation_ ) {
      delete animation_;
      animation_ = NULL;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  ANIMATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Animating the POV-Ray camera.
 *
 * \return void
 *
 * This function registers the given camera animation. The following example demonstrates the
 * use of the animate() function:

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

 * The given type must be derived from the CameraAnimation interface class. In case \a A is not
 * a camera animation, a compile time error is created!
 */
template< typename A >  // Type of the animation
inline void Camera::animate( const A& animation )
{
   pe_CONSTRAINT_MUST_BE_DERIVED_FROM( A, CameraAnimation );

   // Deleting the old camera animation
   if( animation_ )
      delete animation_;

   // Registering the new animation
   animation_ = new A( animation );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removes the current camera animation.
 *
 * \return void
 *
 * This function removes the current camera animation.
 */
inline void Camera::removeAnimation()
{
   if( animation_ ) {
      delete animation_;
      animation_ = NULL;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Camera operators */
//@{
std::ostream& operator<<( std::ostream& os, const Camera& camera );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
