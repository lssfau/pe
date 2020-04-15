//=================================================================================================
/*!
 *  \file src/povray/CustomFinish.cpp
 *  \brief Implementation of a custom POV-Ray finish
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
#include <pe/povray/CustomFinish.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray finish.
 *
 * \param finish The POV-Ray string representation of the finish.
 *
 * This constructor creates a POV-Ray finish using the given declared/predefined finish
 * identifier or user-specific finish string. The following examples illustrate the use
 * of this constructor:

   \code
   pe::povray::CustomFinish( "F_Glass1" );                  (1)   // Declared in the "glass.inc" header
   pe::povray::CustomFinish( "ambient 0.1 diffuse 0.6" );   (2)   // User-specific finish
   \endcode

 * The first finish uses a predefined POV-Ray finish from the \a glass.inc header file. The
 * second finish is a user-specific finish specification. Note however that no checks are
 * performed for both the POV-Ray finish identifiers and the user-specific finishes. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
CustomFinish::CustomFinish( const char* const finish )
   : Finish()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << finish << "\n";
   oss.str().swap( finish_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray finish.
 *
 * \param finish The POV-Ray string representation of the finish.
 *
 * This constructor creates a POV-Ray finish using the given declared/predefined finish
 * identifier or user-specific finish string. The following examples illustrate the use
 * of this constructor:

   \code
   pe::povray::CustomFinish( "F_Glass1" );                  (1)   // Declared in the "glass.inc" header
   pe::povray::CustomFinish( "ambient 0.1 diffuse 0.6" );   (2)   // User-specific finish
   \endcode

 * The first finish uses a predefined POV-Ray finish from the \a glass.inc header file. The
 * second finish is a user-specific finish specification. Note however that no checks are
 * performed for both the POV-Ray finish identifiers and the user-specific finishes. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
CustomFinish::CustomFinish( const std::string& finish )
   : Finish()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << finish << "\n";
   oss.str().swap( finish_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
