//=================================================================================================
/*!
 *  \file src/povray/Texture.cpp
 *  \brief Implementation of a POV-Ray texture
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
#include <pe/povray/Texture.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray texture.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Texture::print( std::ostream& os, bool newline ) const
{
   os << "texture {\n" << texture_ << "}";
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray texture.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the texture output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Texture::print( std::ostream& os, const char* tab, bool newline ) const
{
   std::string line;
   std::istringstream iss( texture_ );

   os << tab << "texture {\n";

   while( std::getline( iss, line ) ) {
      os << tab << line << "\n";
   }

   os << tab << "}";

   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray texture.
 *
 * \param os Reference to the output stream.
 * \param header \a true if the texture header is printed, \a false if not.
 * \param footer \a true if the texture footer is printed, \a false if not.
 * \return void
 */
void Texture::print( std::ostream& os, bool header, bool footer ) const
{
   if( header ) os << "texture {\n";
   os << texture_;
   if( footer ) os << "}\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Texture class.
 * \ingroup povray_texture
 *
 * \param os Reference to the output stream.
 * \param texture Reference to a texture object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Texture& texture )
{
   texture.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
