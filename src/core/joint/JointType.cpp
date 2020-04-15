//=================================================================================================
/*!
 *  \file src/core/joint/JointType.cpp
 *  \brief Source file for the rigid body joint types
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
#include <pe/core/joint/JointType.h>
#include <pe/util/Assert.h>


namespace pe {

//*************************************************************************************************
/*!\brief Global output operator for JointType.
 *
 * \param os Reference to the output stream.
 * \param type The JointType to be put into the stream.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, JointType type )
{
   switch( type )
   {
      case ballType:   os << "ball-in-socket joint"; break;
      case fixedType:  os << "fixed joint";          break;
      case hingeType:  os << "hinge joint";          break;
      case sliderType: os << "slider joint";         break;
      default: pe_INTERNAL_ASSERT( false, "Unknown joint type" ); break;
   }

   return os;
}
//*************************************************************************************************

} // namespace pe
