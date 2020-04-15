//=================================================================================================
/*!
 *  \file src/povray/CustomNormal.cpp
 *  \brief Implementation of a custom POV-Ray normal
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
#include <pe/povray/CustomNormal.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the individual initialization of the POV-Ray normal.
 *
 * \param normal The POV-Ray string representation of the normal.
 *
 * This constructor creates a POV-Ray normal using the given declared/predefined normal
 * identifier or user-specific normal string. The following example illustrate the use
 * of this constructor:

   \code
   pe::povray::CustomNormal( "ripples 2.2 translate <0,1,0>" );  // User-specific normal
   \endcode

 * This normal uses a user-specific normal specification. Note however that no checks are
 * performed for both the POV-Ray normal identifiers and the user-specific normals. Possible
 * errors will only be detected by POV-Ray, not the \b pe physics engine!
 */
CustomNormal::CustomNormal( const char* const normal )
   : Normal()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << normal << "\n";
   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the individual initialization of the POV-Ray normal.
 *
 * \param normal The POV-Ray string representation of the normal.
 *
 * This constructor creates a POV-Ray normal using the given declared/predefined normal
 * identifier or user-specific normal string. The following example illustrate the use
 * of this constructor:

   \code
   pe::povray::CustomNormal( "ripples 2.2 translate <0,1,0>" );  // User-specific normal
   \endcode

 * This normal uses a user-specific normal specification. Note however that no checks are
 * performed for both the POV-Ray normal identifiers and the user-specific normals. Possible
 * errors will only be detected by POV-Ray, not the \b pe physics engine!
 */
CustomNormal::CustomNormal( const std::string& normal )
   : Normal()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << normal << "\n";
   oss.str().swap( normal_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
