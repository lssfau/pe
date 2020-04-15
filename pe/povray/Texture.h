//=================================================================================================
/*!
 *  \file pe/povray/Texture.h
 *  \brief Implementation of a POV-Ray texture
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

#ifndef _PE_POVRAY_TEXTURE_H_
#define _PE_POVRAY_TEXTURE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <string>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup povray_texture Texture
 * \ingroup povray
 *
 * A texture gives a rigid body a specific appearance in the POV-Ray visualization. The basic
 * component of a texture is the pigment that defines the basic color composition (see the
 * \ref povray_pigment module). The pigment can be enhanced by certain finish effects (as for
 * instance glowing effects or reflections, see the \ref povray_finish section) or normal
 * transformations (like bumpy or rippled surfaces, see the \ref povray_normal section).\n\n
 *
 *
 * \section content Contents
 *
 * 1.) \ref plain_texture\n
 * 2.) \ref custom_texture\n
 * 3.) \ref tiled_texture\n
 * 4.) \ref layered_texture\n\n
 *
 *
 * \section plain_texture Plain textures
 *
 * Plain textures (implemented in the PlainTexture class) represents a plain POV-Ray texture
 * individually built from a Pigment, and (if desired) a Finish and a Normal. The following
 * images give an impression of several plain textures:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for plain POV-Ray textures" width=600pt
 *
 * The following code demonstrates the setup of these particular textures:

   \code
   using namespace pe::povray;

   // Creating a single-colored texture
   PlainTexture yellow(
      ColorPigment( 1.0, 1.0, 0.0 ),
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 0.9 ),
         PhongSize( 50 ),
         Reflection( 0.05 )
      )
   );

   // Creating a white & red striped texture
   PlainTexture striped(
      MarblePigment(
         ColorMap( "[0.5 color Red][0.5 color White]" )
      ),
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 0.9 ),
         PhongSize( 50 ),
         Reflection( 0.05 )
      )
   );

   // Creating a colorful texture
   PlainTexture striped(
      SpottedPigment(
         ColorMap( "[0.2 color Red][0.45 color Green][0.55 color Yellow][0.8 color Blue]" )
      ),
      Finish(
         Ambient( 0.1 ),
         Diffuse( 0.6 ),
         Phong( 0.9 ),
         PhongSize( 50 ),
         Reflection( 0.05 )
      )
   );
   \endcode

 * \n \section custom_texture Custom definitions of POV-Ray textures
 *
 * Custom textures (as implemented in the CustomTexture class) offer the possibility to manually
 * create POV-Ray textures or to use predefined POV-Ray texture identifiers. The following images
 * illustrate some declared textures from basic POV-Ray include files:
 *
 * \image html declared.png
 * \image latex declared.eps "Examples for custom POV-Ray textures" width=820pt
 *
 * Examples:

   \code
   CustomTexture texture1( "T_Stone8 scale 10.0" );   (1)   // Declared in the "stones.inc" header
   CustomTexture texture2( "T_Stone9 scale 10.0" );   (2)   // Declared in the "stones.inc" header
   CustomTexture texture3( "T_Grnt10 scale  5.0" );   (3)   // Declared in the "stones.inc" header
   CustomTexture texture4( "T_Wood1  scale  3.0" );   (4)   // Declared in the "woods.inc" header
   CustomTexture texture5( "T_Silver_1A"         );   (5)   // Declared in the "metals.inc" header
   \endcode

 * \n \section tiled_texture Tiled textures
 *
 * Another possibility to create new textures are tiled textures:
 *
 * \image html tiled.png
 * \image latex tiled.eps "Examples for tiled POV-Ray textures" width=400pt
 *
 * The code to create these textures is shown below:

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

 * \n \section layered_texture Layered textures
 *
 * It is also possible to combine several POV-Ray textures to create a variety of special effects.
 * Layered textures (implemented in the LayeredTexture class) consists of several textures that
 * are partially transparent and are laid one on top of each other to create more complex textures.
 * The different texture layers show through the transparent portions to create the appearance of
 * one texture that is a combination of several textures. The following illustration gives an
 * impression of a layered texture:
 *
 * \image html layered.png
 * \image latex layered.eps "Examples for layered POV-Ray textures" width=600pt
 *
 * Example:

   \code
   using namespace pe::povray;

   // Setup of a texture finish
   Finish finish(
      Ambient( 0.1 ),
      Diffuse( 0.6 ),
      Phong( 1.0 ),
      PhongSize( 50 ),
      Reflection( 0.05 )
   );

   // Creating a granite texture
   PlainTexture t1(
      GranitePigment(
         ColorMap( "[0.0  color Red   ]"
                   "[0.33 color Yellow]"
                   "[0.66 color Blue  ]"
                   "[1.0  color Red   ]" ),
         Turbulence( 0.8 ),
         Omega( 0.6 )
      ),
      finish
   );

   // Creating a transparent agate texture
   PlainTexture t2(
      AgatePigment(
         ColorMap( "[0.0  color rgbf <1,0,0,0.8>]"
                   "[0.33 color rgbf <1,1,0,0.8>]"
                   "[0.66 color rgbf <0,0,1,0.8>]"
                   "[1.0  color rgbf <1,0,0,0.8>]" ),
         Turbulence( 0.6 ),
         Octaves( 2 ),
         Omega( 0.4 )
      ),
      finish
   );

   // Creating a layered texture
   LayeredTexture layered( t1, t2 );
   \endcode
 */
/*!\brief A POV-Ray texture.
 * \ingroup povray_texture
 *
 * The Texture class represents a POV-Ray texture that is built from pigments, finishes, normals
 * and transformations.
 */
class PE_PUBLIC Texture
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Texture();
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool isEmpty() const;
   inline void reset  ();
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os,                  bool newline ) const;
   void print( std::ostream& os, const char* tab, bool newline ) const;
   void print( std::ostream& os, bool header    , bool footer  ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::string texture_;  //!< POV-Ray string representation of the texture.
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
/*!\brief Default constructor for the Texture class.
 *
 * The default texture is a solid black colored texture with the default POV-Ray finish effects.
 */
inline Texture::Texture()
   : texture_()  // POV-Ray string representation of the texture
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns \a true if the texture is empty
 *
 * \return \a true if the texture is empty, \a false if it is not.
 *
 * This function returns whether the texture is empty and therefore represents the default
 * POV-Ray texture. In case the texture is empty, the function returns \a true, else it returns
 * \a false.
 */
inline bool Texture::isEmpty() const
{
   return texture_.empty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the POV-Ray texture.
 *
 * \return void
 *
 * This function removes all components from the POV-Ray texture and resets all values to the
 * POV-Ray defaults.
 */
inline void Texture::reset()
{
   texture_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Texture operators */
//@{
std::ostream& operator<<( std::ostream& os, const Texture& texture );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
