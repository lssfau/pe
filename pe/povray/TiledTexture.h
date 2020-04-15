//=================================================================================================
/*!
 *  \file pe/povray/TiledTexture.h
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

#ifndef _PE_POVRAY_TILEDTEXTURE_H_
#define _PE_POVRAY_TILEDTEXTURE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <pe/povray/Texture.h>
#include <pe/povray/Transformation.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/TypeRestriction.h>
#include <pe/util/TypeList.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray tiled texture.
 * \ingroup povray_texture
 *
 * The TiledTexture class represents a tiled POV-Ray texture. A tiled texture consists of two
 * other POV-Ray textures. The following images give an impression of tiled textures:
 *
 * \image html tiled.png
 * \image latex tiled.eps "Examples for POV-Ray textures" width=400pt
 *
 * The textures used to create these tiled textures were defined as:

   \code
   using namespace pe::povray;

   // Creating a marble tiled texture
   Texture t1     = CustomTexture( "T_Stone8" );   // Declared in the "stones.inc" header
   Texture t2     = CustomTexture( "T_Stone9" );   // Declared in the "stones.inc" header
   Texture tiled1 = TiledTexture( t1, t2, Scale( 1.22 ) );

   // Creating a white/transparent-blue tiled texture
   Texture t3     = PlainTexture( ColorPigment( "color rgb <1,1,1>" ) );
   Texture t4     = PlainTexture( ColorPigment( "color rgbf <0,0,1,0.7>" ) );
   Texture tiled2 = TiledTexture( t3, t4, Scale( 1.22 ) );
   \endcode
 */
class PE_PUBLIC TiledTexture : public Texture
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( Transformation )  ValidTypes;  //!< Valid texture types.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit TiledTexture( const Texture& texture1, const Texture& texture2 );

   template< typename A >
   explicit TiledTexture( const Texture& texture1, const Texture& texture2, const A& a );

   template< typename A, typename B >
   explicit TiledTexture( const Texture& texture1, const Texture& texture2, const A& a, const B& b );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~TiledTexture();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename A >
   void add( const A& a );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




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
 * \param a The transformation for the tiled texture.
 *
 * This constructor creates a tiled POV-Ray texture from the two given textures. Additionally,
 * the transformation \a a is directly added to the texture. Valid transformations are
 *  - Scale
 *  - Translation
 *  - Rotation
 *
 * The attempt to use this function with any other type will result in a compile time error.\n
 * The following images give an impression of tiled textures:
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
template< typename A >  // Type of the transformation
TiledTexture::TiledTexture( const Texture& texture1, const Texture& texture2, const A& a )
   : Texture()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << "   tiles {\n";
   texture1.print( oss, "      ", true );
   oss << "      tile2\n";
   texture2.print( oss, "      ", true );
   oss << "   }\n";
   a.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a tiled texture.
 *
 * \param texture1 The first texture.
 * \param texture2 The second texture.
 * \param a The first transformation for the tiled texture.
 * \param b The second transformation for the tiled texture.
 *
 * This constructor creates a tiled POV-Ray texture from the two given textures. Additionally,
 * the two modifiers \a a and \a b are directly added to the texture. Valid transformations are
 *  - Scale
 *  - Translation
 *  - Rotation
 *
 * The attempt to use this function with any other type will result in a compile time error.\n
 * The following images give an impression of tiled textures:
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
template< typename A    // Type of the first transformation
        , typename B >  // Type of the second transformation
TiledTexture::TiledTexture( const Texture& texture1, const Texture& texture2, const A& a, const B& b )
   : Texture()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   std::ostringstream oss;
   oss << "   tiles {\n";
   texture1.print( oss, "      ", true );
   oss << "      tile2\n";
   texture2.print( oss, "      ", true );
   oss << "   }\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the TiledTexture class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline TiledTexture::~TiledTexture()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Texture, TiledTexture );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a transformation to the tiled POV-Ray texture.
 *
 * \param a The new transformation (scale, translation, rotation, ...).
 * \return void
 */
template< typename A >
void TiledTexture::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << texture_;
   a.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
