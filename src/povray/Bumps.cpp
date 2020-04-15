//=================================================================================================
/*!
 *  \file src/povray/Bumps.cpp
 *  \brief Implementation of a bumps normal for the POV-Ray visualization
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

#include <pe/povray/Bumps.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a bumps normal.
 *
 * \param depth The depth of the bumps effect \f$ [0..1] \f$.
 * \exception std::invalid_argument Invalid depth value.
 *
 * This constructor creates a POV-Ray bumps normal. The depth parameter for the bumps
 * has to be in the range from 0 to 1, inclusive. Otherwise, a \a std::invalid_argument
 * exception is thrown.
 */
Bumps::Bumps( real depth )
   : Normal()  // Initialization of the base class
{
   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   bumps " << depth << "\n";

   oss.str().swap( normal_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
