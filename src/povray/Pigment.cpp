//=================================================================================================
/*!
 *  \file src/povray/Pigment.cpp
 *  \brief Implementation of a POV-Ray pigment
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
#include <pe/povray/Pigment.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray pigment.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Pigment::print( std::ostream& os, bool newline ) const
{
   os << "pigment {\n" << pigment_ << "}";
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray pigment.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the pigment output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Pigment::print( std::ostream& os, const char* tab, bool newline ) const
{
   std::string line;
   std::istringstream iss( pigment_ );

   os << tab << "pigment {\n";

   while( std::getline( iss, line ) ) {
      os << tab << line << "\n";
   }

   os << tab << "}";

   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Pigment class.
 * \ingroup povray_pigment
 *
 * \param os Reference to the output stream.
 * \param pigment Reference to a pigment object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Pigment& pigment )
{
   pigment.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
