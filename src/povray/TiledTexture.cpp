//=================================================================================================
/*!
 *  \file src/povray/TiledTexture.cpp
 *  \brief Implementation of a tiled POV-Ray texture
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

#include <pe/povray/TiledTexture.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a tiled texture.
 *
 * \param texture1 The first texture.
 * \param texture2 The second texture.
 *
 * This constructor creates a tiled POV-Ray texture from the two given textures. The following
 * images give an impression of tiled textures:
 *
 * \image html tiled.png
 * \image latex tiled.eps "Examples for POV-Ray textures" width=400pt
 *
 * The textures used to create these tiled textures were defined as

   \code
   using namespace pe::povray;

   // Creating a marble tiled texture
   Texture t1     = Texture( "T_Stone8" );   // Declared in the "stones.inc" header
   Texture t2     = Texture( "T_Stone9" );   // Declared in the "stones.inc" header
   Texture tiled1 = TiledTexture( t1, t2, Scale( 1.22 ) );

   // Creating a white/transparent-blue tiled texture
   Texture t3     = Texture( ColorPigment( "color rgb <1,1,1>" ) );
   Texture t4     = Texture( ColorPigment( "color rgbf <0,0,1,0.7>" ) );
   Texture tiled2 = TiledTexture( t3, t4, Scale( 1.22 ) );
   \endcode
 */
TiledTexture::TiledTexture( const Texture& texture1, const Texture& texture2 )
   : Texture()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   tiles {\n";
   texture1.print( oss, "      ", true );
   oss << "      tile2\n";
   texture2.print( oss, "      ", true );
   oss << "   }\n";

   oss.str().swap( texture_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
