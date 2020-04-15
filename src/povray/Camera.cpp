//=================================================================================================
/*!
 *  \file src/povray/Camera.cpp
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iostream>
#include <stdexcept>
#include <pe/povray/Camera.h>
#include <pe/util/Logging.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for Camera.
 *
 * This constructor creates a default POV-Ray camera. The location of the camera is set to
 * (0,0,0), its focus point is (0,1,0) and the default sky vector is (0,0,1).
 */
Camera::Camera()
   : Trigger   ()                         // Initialization of the Trigger base class
   , Singleton<Camera,logging::Logger>()  // Initialization of the Singleton base class
   , location_ ( 0, 0, 0 )                // The global position of the camera
   , focus_    ( 0, 1, 0 )                // The focus point of the camera
   , sky_      ( 0, 0, 1 )                // The up/sky-direction of the camera
   , animation_( NULL    )                // The camera animation
{
   // Logging the successful setup of the POV-Ray camera
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the POV-Ray camera instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor of the Camera class.
 */
Camera::~Camera()
{
   delete animation_;

   // Logging the successful destruction of the POV-Ray camera
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the POV-Ray camera instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the global location of the POV-Ray camera.
 *
 * \param x The x-component of the new location of the camera.
 * \param y The y-component of the new location of the camera.
 * \param z The z-component of the new location of the camera.
 * \return void
 *
 * This function sets the global location of the POV-Ray camera.
 */
void Camera::setLocation( real x, real y, real z )
{
   location_ = Vec3( x, y, z );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global location of the POV-Ray camera.
 *
 * \param location The new location of the camera.
 * \return void
 *
 * This function sets the global location of the POV-Ray camera.
 */
void Camera::setLocation( const Vec3& location )
{
   location_ = location;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the focus point of the POV-Ray camera.
 *
 * \param x The x-component of the new focus point of the camera.
 * \param y The y-component of the new focus point of the camera.
 * \param z The z-component of the new focus point of the camera.
 * \return void
 *
 * This function sets the focus point of the POV-Ray camera.
 */
void Camera::setFocus( real x, real y, real z )
{
   focus_ = Vec3( x, y, z );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the focus point of the POV-Ray camera.
 *
 * \param focus The new focus point of the camera.
 * \return void
 *
 * This function sets the focus point of the POV-Ray camera.
 */
void Camera::setFocus( const Vec3& focus )
{
   focus_ = focus;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the sky vector of the POV-Ray camera.
 *
 * \param x The x-component of the new up/sky-direction of the camera.
 * \param y The y-component of the new up/sky-direction of the camera.
 * \param z The z-component of the new up/sky-direction of the camera.
 * \return void
 * \exception std::invalid_argument Invalid sky direction.
 */
void Camera::setSky( real x, real y, real z )
{
   const Vec3 sky( x, y, z );

   // Checking the sky vector
   if( sky.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid sky direction" );

   sky_ = sky;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the sky vector of the POV-Ray camera.
 *
 * \param sky The new up/sky-direction of the camera.
 * \return void
 * \exception std::invalid_argument Invalid sky direction.
 */
void Camera::setSky( const Vec3& sky )
{
   // Checking the sky vector
   if( sky.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid sky direction" );

   sky_ = sky;
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray camera.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Camera::print( std::ostream& os, bool newline ) const
{
   os << "camera {"
         " location <" << location_[0] << "," << location_[2] << "," << location_[1] << ">"
         " look_at <" << focus_[0] << "," << focus_[2] << "," << focus_[1] << ">"
         " sky <" << sky_[0] << "," << sky_[2] << "," << sky_[1] << ">"
         " }";

   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray camera.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the camera output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Camera::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "camera {\n"
      << tab << "   location <" << location_[0] << "," << location_[2] << "," << location_[1] << ">\n"
      << tab << "   look_at <" << focus_[0] << "," << focus_[2] << "," << focus_[1] << ">\n"
      << tab << "   sky <" << sky_[0] << "," << sky_[2] << "," << sky_[1] << ">\n"
      << tab << "}";

   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  TRIGGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Updating the camera location, focus point and sky vector.
 *
 * \return void
 *
 * This function triggers an update of the camera location, its focus points and the sky vector
 * depending on the current camera animation. In case no animation was specified, the state of
 * the camera remains unchanged.
 */
void Camera::trigger()
{
   if( animation_ ) {
      animation_->updateLocation( location_ );  // Updating the global location of the camera
      animation_->updateFocus   ( focus_    );  // Updating the focus point of the camera
      animation_->updateSky     ( sky_      );  // Updating the sky vector of the camra
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Camera class.
 * \ingroup povray_camera
 *
 * \param os Reference to the output stream.
 * \param camera Reference to a camera object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Camera& camera )
{
   camera.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
