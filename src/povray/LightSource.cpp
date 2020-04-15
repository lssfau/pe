//=================================================================================================
/*!
 *  \file src/povray/LightSource.cpp
 *  \brief Implementation of light sources for the POV-Ray visualization
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
#include <sstream>
#include <pe/povray/LightSource.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray light source.
 *
 * \param lightsource The POV-Ray string representation of the lightsource.
 *
 * This constructor creates a POV-Ray light source using the given declared/predefined light
 * source identifier or user-specific light source string. The following example illustrate
 * the use of this constructor:

   \code
   pe::povray::LightSource( "<200,200,-200> color rgb <1,1,1>" );
   \endcode

 * Note that this constructor requires knowledge about the POV-Ray specific syntax for a light
 * source. No checks are performed to verify the correctness of the light source string for
 * both the POV-Ray light source identifiers and user-specific light source definitions. Errors
 * will only be detected by POV-Ray, not by the \b pe physics engine.
 */
LightSource::LightSource( const char* const lightsource )
   : lightsource_()  // POV-Ray string representation of the light source
{
   std::ostringstream oss;
   oss << "   " << lightsource << "\n";
   lightsource_ = lightsource;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray light source.
 *
 * \param lightsource The POV-Ray string representation of the light source.
 *
 * This constructor creates a POV-Ray light source using the given declared/predefined light
 * source identifier or user-specific light source string. The following example illustrate
 * the use of this constructor:

   \code
   pe::povray::LightSource( "<200,200,-200> color rgb <1,1,1>" );
   \endcode

 * Note that this constructor requires knowledge about the POV-Ray specific syntax for a light
 * source. No checks are performed to verify the correctness of the light source string for
 * both the POV-Ray light source identifiers and user-specific light source definitions. Errors
 * will only be detected by POV-Ray, not by the \b pe physics engine.
 */
LightSource::LightSource( const std::string& lightsource )
   : lightsource_()  // POV-Ray string representation of the light source
{
   std::ostringstream oss;
   oss << "   " << lightsource << "\n";
   lightsource_ = lightsource;
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray light source.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void LightSource::print( std::ostream& os, bool newline ) const
{
   if( lightsource_.empty() )
      os << "light_source {\n   <0,0,0> color rgb <1,1,1>\n}";
   else
      os << "light_source {\n" << lightsource_ << "}";

   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray light source.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the light source output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void LightSource::print( std::ostream& os, const char* tab, bool newline ) const
{
   if( lightsource_.empty() ) {
      os << "light_source {\n   <0,0,0> color rgb <1,1,1>\n}";
   }
   else {
      std::string line;
      std::istringstream iss( lightsource_ );

      os << tab << "light_source {\n";

      while( std::getline( iss, line ) ) {
         os << tab << line << "\n";
      }

      os << tab << "}";
   }

   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the LightSource class.
 * \ingroup povray_lightsource
 *
 * \param os Reference to the output stream.
 * \param lightsource Reference to a light source object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const LightSource& lightsource )
{
   lightsource.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
