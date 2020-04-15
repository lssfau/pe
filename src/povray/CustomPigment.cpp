//=================================================================================================
/*!
 *  \file src/povray/CustomPigment.cpp
 *  \brief Implementation of a custom POV-Ray pigment
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
#include <pe/povray/CustomPigment.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray pigment.
 *
 * \param pigment The POV-Ray string representation of the pigment.
 *
 * This constructor creates a POV-Ray pigment using the given declared/predefined pigment
 * identifier or user-specific pigment string. The following examples illustrate the use
 * of this constructor:

   \code
   pe::povray::CustomPigment( "P_WoodGrain1A" );               (1)   // Declared in the "woods.inc" header
   pe::povray::CustomPigment( "checker color Black White" );   (2)   // User-specific pigment
   \endcode

 * The first pigment uses a predefined POV-Ray pigment from the \a woods.inc header file. The
 * second pigment is a user-specific pigment specification. Note however that no checks are
 * performed for both the POV-Ray pigment identifiers and the user-specific pigments. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
CustomPigment::CustomPigment( const char* const pigment )
   : Pigment()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << pigment << "\n";
   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray pigment.
 *
 * \param pigment The POV-Ray string representation of the pigment.
 *
 * This constructor creates a POV-Ray pigment using the given declared/predefined pigment
 * identifier or user-specific pigment string. The following examples illustrate the use
 * of this constructor:

   \code
   pe::povray::CustomPigment( "P_WoodGrain1A" );               (1)   // Declared in the "woods.inc" header
   pe::povray::CustomPigment( "checker color Black White" );   (2)   // User-specific pigment
   \endcode

 * The first pigment uses a predefined POV-Ray pigment from the \a woods.inc header file. The
 * second pigment is a user-specific pigment specification. Note however that no checks are
 * performed for both the POV-Ray pigment identifiers and the user-specific pigments. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
CustomPigment::CustomPigment( const std::string& pigment )
   : Pigment()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << pigment << "\n";
   oss.str().swap( pigment_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
