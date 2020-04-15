//=================================================================================================
/*!
 *  \file src/povray/CustomLight.cpp
 *  \brief Implementation of a custom POV-Ray light
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

#include <sstream>
#include <pe/povray/CustomLight.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the individual initialization of the POV-Ray light source.
 *
 * \param lightsource The POV-Ray string representation of the light source.
 *
 * This constructor creates a POV-Ray light source using the given user-specific light source
 * string. The following example illustrate the use of this constructor:

   \code
   pe::povray::CustomLight( "<2,20,2> color rgb <0.9,0.9,0.9>" );  // User-specific light source
   \endcode

 * This setup of a light sources uses a custom POV-Ray light source string in order to create
 * a white point light source at the location (2,2,20). Note that since the given light source
 * is directly used in the POV-Ray files, points and vectors have to be directly specified in
 * the left-handed POV-Ray coordinate system! Also note that no checks are performed for the
 * user-specific light source setup. Possible errors will only be detected by POV-Ray, not the
 * \b pe physics engine!
 */
CustomLight::CustomLight( const char* const lightsource )
   : LightSource()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << lightsource << "\n";
   oss.str().swap( lightsource_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the individual initialization of the POV-Ray light source.
 *
 * \param lightsource The POV-Ray string representation of the light source.
 *
 * This constructor creates a POV-Ray light source using the given user-specific light source
 * string. The following example illustrate the use of this constructor:

   \code
   pe::povray::CustomLight( "<2,20,2> color rgb <0.9,0.9,0.9>" );  // User-specific light source
   \endcode

 * This setup of a light sources uses a custom POV-Ray light source string in order to create
 * a white point light source at the location (2,2,20). Note that since the given light source
 * is directly used in the POV-Ray files, points and vectors have to be directly specified in
 * the left-handed POV-Ray coordinate system! Also note that no checks are performed for the
 * user-specific light source setup. Possible errors will only be detected by POV-Ray, not the
 * \b pe physics engine!
 */
CustomLight::CustomLight( const std::string& lightsource )
   : LightSource()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << lightsource << "\n";
   oss.str().swap( lightsource_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
