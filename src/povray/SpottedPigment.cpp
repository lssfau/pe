//=================================================================================================
/*!
 *  \file src/povray/SpottedPigment.cpp
 *  \brief Implementation of a POV-Ray spotted pigment
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
#include <pe/povray/SpottedPigment.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a spotted pigment according to the given color map.
 *
 * \param colormap The color map for the spotted pigment.
 *
 * This constructor creates a spotted pigment using the given color map.
 */
SpottedPigment::SpottedPigment( const ColorMap& colormap )
   : Pigment()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   spotted\n";
   colormap.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
