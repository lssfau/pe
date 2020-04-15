//=================================================================================================
/*!
 *  \file src/povray/Normal.cpp
 *  \brief Implementation of a POV-Ray normal
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
#include <pe/povray/Normal.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray normal.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Normal::print( std::ostream& os, bool newline ) const
{
   os << "normal {\n" << normal_ << "}";
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the current state of a POV-Ray normal.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the normal output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Normal::print( std::ostream& os, const char* tab, bool newline ) const
{
   std::string line;
   std::istringstream iss( normal_ );

   os << tab << "normal {\n";

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
/*!\brief Global output operator for the Normal class.
 * \ingroup povray_normal
 *
 * \param os Reference to the output stream.
 * \param normal Reference to a normal object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Normal& normal )
{
   normal.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
