//=================================================================================================
/*!
 *  \file src/povray/CustomTexture.cpp
 *  \brief Implementation of a custom POV-Ray texture
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
#include <pe/povray/CustomTexture.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray texture.
 *
 * \param texture The POV-Ray string representation of the texture.
 *
 * This constructor creates a custom POV-Ray texture using the given declared/predefined
 * texture identifier or user-specific texture string. The following examples illustrate
 * the use of this constructor:

   \code
   pe::povray::CustomTexture( "T_Stone9" );                        (1)   // Declared in the "stones.inc" header
   pe::povray::CustomTexture( "pigment { color rgb <1,1,0> }" );   (2)   // User-specific texture
   \endcode

 * The first texture uses a predefined POV-Ray texture from the \a woods.inc header file. The
 * second texture is a user-specific texture specification. Note however that no checks are
 * performed for both the POV-Ray texture identifiers and the user-specific textures. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
CustomTexture::CustomTexture( const char* const texture )
   : Texture()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << texture << "\n";
   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the individual initialization of a POV-Ray texture.
 *
 * \param texture The POV-Ray string representation of the texture.
 *
 * This constructor creates a custom POV-Ray texture using the given declared/predefined
 * texture identifier or user-specific texture string. The following examples illustrate
 * the use of this constructor:

   \code
   pe::povray::CustomTexture( "T_Stone9" );                        (1)   // Declared in the "stones.inc" header
   pe::povray::CustomTexture( "pigment { color rgb <1,1,0> }" );   (2)   // User-specific texture
   \endcode

 * The first texture uses a predefined POV-Ray texture from the \a woods.inc header file. The
 * second texture is a user-specific texture specification. Note however that no checks are
 * performed for both the POV-Ray texture identifiers and the user-specific textures. Possible
 * errors will only be detected by POV-Ray, not by the \b pe physics engine!
 */
CustomTexture::CustomTexture( const std::string& texture )
   : Texture()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   " << texture << "\n";
   oss.str().swap( texture_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
