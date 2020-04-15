//=================================================================================================
/*!
 *  \file src/povray/LayeredTexture.cpp
 *  \brief Implementation of a layered POV-Ray texture
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
#include <pe/povray/LayeredTexture.h>
#include <pe/util/Assert.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a layered texture from the two given POV-Ray textures.
 *
 * \param texture1 The bottom layer texture.
 * \param texture2 The top layer texture.
 *
 * This constructor creates a multi-layered texture from the two given POV-Ray textures. The
 * first given texture is used as the bottom layer, the second texture as the top layer of
 * the resulting layered texture.
 */
LayeredTexture::LayeredTexture( const Texture& texture1, const Texture& texture2 )
   : Texture()  // Initialization of the base class
{
   std::ostringstream oss;

   texture1.print( oss, false, true  );  // Printing the first texture without header
   texture2.print( oss, true , false );  // Printing the second texture without footer

   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a layered texture from the three given POV-Ray textures.
 *
 * \param texture1 The bottom layer texture.
 * \param texture2 The intermediate layer texture.
 * \param texture3 The top layer texture.
 *
 * This constructor creates a multi-layered texture from the three given POV-Ray textures. The
 * first given texture is used as the bottom layer, the second texture as intermediate layer,
 * and the third texture is used as top layer for the resulting layered texture.
 */
LayeredTexture::LayeredTexture( const Texture& texture1, const Texture& texture2,
                                const Texture& texture3 )
   : Texture()  // Initialization of the base class
{
   std::ostringstream oss;

   texture1.print( oss, false, true  );  // Printing the first texture without header
   texture2.print( oss, true , true  );  // Printing the intermediate texture
   texture3.print( oss, true , false );  // Printing the third texture without footer

   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a layered texture from the four given POV-Ray textures.
 *
 * \param texture1 The bottom layer texture.
 * \param texture2 The first intermediate layer texture.
 * \param texture3 The second intermediate layer texture.
 * \param texture4 The top layer texture.
 *
 * This constructor creates a multi-layered texture from the four given POV-Ray textures. The
 * first given texture is used as the bottom layer, the second texture as first intermediate
 * layer, the third texture as second intermediate layer, and the third texture is used as
 * top layer for the resulting layered texture.
 */
LayeredTexture::LayeredTexture( const Texture& texture1, const Texture& texture2,
                                const Texture& texture3, const Texture& texture4 )
   : Texture()  // Initialization of the base class
{
   std::ostringstream oss;

   texture1.print( oss, false, true  );  // Printing the first texture without header
   texture2.print( oss, true , true  );  // Printing the first intermediate texture
   texture3.print( oss, true , true  );  // Printing the second intermediate texture
   texture4.print( oss, true , false );  // Printing the fourth texture without footer

   oss.str().swap( texture_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a texture to the layered POV-Ray texture.
 *
 * \param texture The new topmost texture.
 * \return void
 *
 * This function adds a new texture to the layered POV-Ray texture. The new texture is used
 * as topmost/outermost layer for the layered texture.
 */
void LayeredTexture::add( const Texture& texture )
{
   pe_INTERNAL_ASSERT( !isEmpty(), "Empty layered texture detected" );

   std::ostringstream oss;

   print        ( oss, false, true  );  // Printing the old bottom layers
   texture.print( oss, true , false );  // Printing the new top layer

   oss.str().swap( texture_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
