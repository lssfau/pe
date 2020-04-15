//=================================================================================================
/*!
 *  \file src/povray/RandomColorMap.cpp
 *  \brief Implementation of a random color map for the POV-Ray visualization
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
#include <pe/povray/RandomColor.h>
#include <pe/povray/RandomColorMap.h>
#include <pe/system/Precision.h>
#include <pe/util/Random.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the RandomColorMap class.
 */
RandomColorMap::RandomColorMap()
   : ColorMap()  // Initialization of the base class
{
   const unsigned int n( rand<unsigned int>( 2, 5 ) );
   real threshold( 0 );

   std::ostringstream oss;
   for( unsigned int i=0; i<n; ++i ) {
      threshold = rand<real>( threshold, 1 );
      oss << "   [" << threshold << " " << RandomColor() << "]\n";
   }

   oss.str().swap( colormap_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
