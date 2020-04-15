//=================================================================================================
/*!
 *  \file src/core/ContactType.cpp
 *  \brief Source file for the contact types
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
#include <pe/core/ContactType.h>
#include <pe/util/Assert.h>


namespace pe {

//*************************************************************************************************
/*!\brief Global output operator for ContactType.
 *
 * \param os Reference to the output stream.
 * \param type The ContactType to be put into the stream.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ContactType type )
{
   switch( type )
   {
      case colliding:  os << "colliding";  break;
      case resting:    os << "resting";    break;
      case separating: os << "separating"; break;
      default: pe_INTERNAL_ASSERT( false, "Unknown contact type" ); break;
   }

   return os;
}
//*************************************************************************************************

} // namespace pe
