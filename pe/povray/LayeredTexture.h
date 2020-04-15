//=================================================================================================
/*!
 *  \file pe/povray/LayeredTexture.h
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

#ifndef _PE_POVRAY_LAYEREDTEXTURE_H_
#define _PE_POVRAY_LAYEREDTEXTURE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/Texture.h>
#include <pe/util/constraints/SameSize.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A POV-Ray layered texture.
 * \ingroup povray_texture
 *
 * One option to create a variety of special texture effects is the combination of several POV-Ray
 * textures. The LayeredTexture class represents a multi-layered POV-Ray texture, that consists of
 * several textures that are partially transparent and are laid one on top of the other to create
 * a more complex texture. The different texture layers show through the transparent portions to
 * create the appearance of one texture that is a combination of several textures. The following
 * illustration gives an impression of a layered texture:
 *
 * \image html layered.png
 * \image latex layered.eps "Examples for layered POV-Ray textures" width=600pt
 *
 * Layered textures are created via the LayeredTexture constructors, which allow up to 4 textures
 * to be combined into a multi-layered texture. The subsequent code shows the configuration of the
 * illustrated layered POV-Ray texture consisting of two textures. The first texture is used as the
 * bottom layer, the second texture as the top layer of the resulting layered texture.

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
         ColorMap( "[0.0 color  rgbf <1,0,0,0.8>]"
                   "[0.33 color rgbf <1,1,0,0.8>]"
                   "[0.66 color rgbf <0,0,1,0.8>]"
                   "[1.0 color  rgbf <1,0,0,0.8>]" ),
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
class PE_PUBLIC LayeredTexture : public Texture
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit LayeredTexture( const Texture& texture1, const Texture& texture2 );
   explicit LayeredTexture( const Texture& texture1, const Texture& texture2,
                            const Texture& texture3 );
   explicit LayeredTexture( const Texture& texture1, const Texture& texture2,
                            const Texture& texture3, const Texture& texture4 );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~LayeredTexture();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void add( const Texture& texture );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the LayeredTexture class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline LayeredTexture::~LayeredTexture()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Texture, LayeredTexture );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
