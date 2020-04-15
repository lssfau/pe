//=================================================================================================
/*!
 *  \file pe/povray/PlainTexture.h
 *  \brief Implementation of a plain POV-Ray texture
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

#ifndef _PE_POVRAY_PLAINTEXTURE_H_
#define _PE_POVRAY_PLAINTEXTURE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <pe/povray/Texture.h>
#include <pe/povray/TextureItem.h>
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
/*!\brief A plain POV-Ray texture.
 * \ingroup povray_texture
 *
 * The PlainTexture class represents a plain POV-Ray texture built from a Pigment, a Finish
 * and a Normal. The following images give an impression of plain textures:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for POV-Ray textures" width=600pt
 *
 * In order to create these textures, the following code can be used:

   \code
   using namespace pe::pov;

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
 */
class PE_PUBLIC PlainTexture : public Texture
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_2( TextureItem, Transformation )  ValidTypes;  //!< Valid texture types.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline PlainTexture();

   template< typename A >
   explicit PlainTexture( const A& a );

   template< typename A, typename B >
   explicit PlainTexture( const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit PlainTexture( const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit PlainTexture( const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit PlainTexture( const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~PlainTexture();
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
/*!\brief Default constructor for the PlainTexture class.
 *
 * The default texture is a solid black colored texture with the default POV-Ray finish effects.
 */
inline PlainTexture::PlainTexture()
   : Texture()  // Initialization of the base class
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a plain POV-Ray texture consisting of a single texture component.
 *
 * \param a The texture component/transformation.
 *
 * A POV-Ray texture can be directly created from several texture components. This constructor
 * creates a POV-Ray texture from a single given texture component. Additionally, the following
 * constructors are available:
 *
 * -# PlainTexture::PlainTexture( const A& a, const B& b );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d, const E& e );
 *
 * The given components must be either a texture item (as for instance Pigment, Finish or Normal)
 * or a POV-Ray transformation (as for instance Scale, Translation or Rotation). The attempt to
 * create a texture using an invalid type results in a compile time error.\n
 * The images demonstrate three possible POV-Ray textures created from different POV-Ray pigments:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for POV-Ray textures" width=600pt
 *
 * The following example code demonstrates how these texture can be directly created using the
 * single component constructor:

   \code
   using namespace pe::pov;

   PlainTexture t1( ColorPigment( "color rgb <1,1,0>" ) );                                    (1)
   PlainTexture t2( MarblePigment ( ColorMap( "[0.0  color White ][0.5  color White]"
                                              "[0.5  color Red   ][1.0  color Red  ]" ) ) );  (2)
   PlainTexture t3( SpottedPigment( ColorMap( "[0.2  color Red   ][0.45 color Green]"
                                              "[0.55 color Yellow][0.8  color Blue ]" ) ) );  (3)
   \endcode
 */
template< typename A >  // Type of the texture item or transformation
PlainTexture::PlainTexture( const A& a )
   : Texture()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   a.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a plain POV-Ray texture consisting of two texture components.
 *
 * \param a The first texture component/transformation.
 * \param b The second texture component/transformation.
 *
 * A POV-Ray texture can be directly created from several texture components. This constructor
 * creates a POV-Ray texture from the two given texture components. Additionally, the following
 * constructors are available:
 *
 * -# PlainTexture::PlainTexture( const A& a );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d, const E& e );
 *
 * The given components must be either a texture item (as for instance Pigment, Finish or Normal)
 * or a POV-Ray modifier (as for instance Scale, Translation or Rotation). The attempt to create a
 * texture using an invalid type results in a compile time error.\n
 * The images demonstrate three possible POV-Ray textures created from different POV-Ray pigments:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for POV-Ray textures" width=600pt
 *
 * The following example code demonstrates how these texture can be directly created using the
 * single component constructor:

   \code
   using namespace pe::pov;

   PlainTexture t1( ColorPigment( "color rgb <1,1,0>" ) );                                    (1)
   PlainTexture t2( MarblePigment ( ColorMap( "[0.0  color White ][0.5  color White]"
                                              "[0.5  color Red   ][1.0  color Red  ]" ) ) );  (2)
   PlainTexture t3( SpottedPigment( ColorMap( "[0.2  color Red   ][0.45 color Green]"
                                              "[0.55 color Yellow][0.8  color Blue ]" ) ) );  (3)
   \endcode
 */
template< typename A    // Type of the first texture item or transformation
        , typename B >  // Type of the second texture item or transformation
PlainTexture::PlainTexture( const A& a, const B& b )
   : Texture()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a plain POV-Ray texture consisting of three texture components.
 *
 * \param a The first texture component/transformation.
 * \param b The second texture component/transformation.
 * \param c The third texture component/transformation.
 *
 * A POV-Ray texture can be directly created from several texture components. This constructor
 * creates a POV-Ray texture from the three given texture components. Additionally, the following
 * constructors are available:
 *
 * -# PlainTexture::PlainTexture( const A& a );
 * -# PlainTexture::PlainTexture( const A& a, const B& b );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d, const E& e );
 *
 * The given components must be either a texture item (as for instance Pigment, Finish or Normal)
 * or a POV-Ray modifier (as for instance Scale, Translation or Rotation). The attempt to create a
 * texture using an invalid type results in a compile time error.\n
 * The images demonstrate three possible POV-Ray textures created from different POV-Ray pigments:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for POV-Ray textures" width=600pt
 *
 * The following example code demonstrates how these texture can be directly created using the
 * single component constructor:

   \code
   using namespace pe::pov;

   PlainTexture t1( ColorPigment( "color rgb <1,1,0>" ) );                                    (1)
   PlainTexture t2( MarblePigment ( ColorMap( "[0.0  color White ][0.5  color White]"
                                              "[0.5  color Red   ][1.0  color Red  ]" ) ) );  (2)
   PlainTexture t3( SpottedPigment( ColorMap( "[0.2  color Red   ][0.45 color Green]"
                                              "[0.55 color Yellow][0.8  color Blue ]" ) ) );  (3)
   \endcode
 */
template< typename A    // Type of the first texture item or transformation
        , typename B    // Type of the second texture item or transformation
        , typename C >  // Type of the third texture item or transformation
PlainTexture::PlainTexture( const A& a, const B& b, const C& c )
   : Texture()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a plain POV-Ray texture consisting of four texture components.
 *
 * \param a The first texture component/transformation.
 * \param b The second texture component/transformation.
 * \param c The third texture component/transformation.
 * \param d The fourth texture component/transformation.
 *
 * A POV-Ray texture can be directly created from several texture components. This constructor
 * creates a POV-Ray texture from the four given texture components. Additionally, the following
 * constructors are available:
 *
 * -# PlainTexture::PlainTexture( const A& a );
 * -# PlainTexture::PlainTexture( const A& a, const B& b );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d, const E& e );
 *
 * The given components must be either a texture item (as for instance Pigment, Finish or Normal)
 * or a POV-Ray modifier (as for instance Scale, Translation or Rotation). The attempt to create a
 * texture using an invalid type results in a compile time error.\n
 * The images demonstrate three possible POV-Ray textures created from different POV-Ray pigments:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for POV-Ray textures" width=600pt
 *
 * The following example code demonstrates how these texture can be directly created using the
 * single component constructor:

   \code
   using namespace pe::pov;

   PlainTexture t1( ColorPigment( "color rgb <1,1,0>" ) );                                    (1)
   PlainTexture t2( MarblePigment ( ColorMap( "[0.0  color White ][0.5  color White]"
                                              "[0.5  color Red   ][1.0  color Red  ]" ) ) );  (2)
   PlainTexture t3( SpottedPigment( ColorMap( "[0.2  color Red   ][0.45 color Green]"
                                              "[0.55 color Yellow][0.8  color Blue ]" ) ) );  (3)
   \endcode
 */
template< typename A    // Type of the first texture item or transformation
        , typename B    // Type of the second texture item or transformation
        , typename C    // Type of the third texture item or transformation
        , typename D >  // Type of the fourth texture item or transformation
PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d )
   : Texture()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a plain POV-Ray texture consisting of four texture components.
 *
 * \param a The first texture component/transformation.
 * \param b The second texture component/transformation.
 * \param c The third texture component/transformation.
 * \param d The fourth texture component/transformation.
 * \param e The fifth texture component/transformation.
 *
 * A POV-Ray texture can be directly created from several texture components. This constructor
 * creates a POV-Ray texture from the five given texture components. Additionally, the following
 * constructors are available:
 *
 * -# PlainTexture::PlainTexture( const A& a );
 * -# PlainTexture::PlainTexture( const A& a, const B& b );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d );
 * -# PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d, const E& e );
 *
 * The given components must be either a texture item (as for instance Pigment, Finish or Normal)
 * or a POV-Ray modifier (as for instance Scale, Translation or Rotation). The attempt to create a
 * texture using an invalid type results in a compile time error.\n
 * The images demonstrate three possible POV-Ray textures created from different POV-Ray pigments:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for POV-Ray textures" width=600pt
 *
 * The following example code demonstrates how these texture can be directly created using the
 * single component constructor:

   \code
   using namespace pe::pov;

   PlainTexture t1( ColorPigment( "color rgb <1,1,0>" ) );                                    (1)
   PlainTexture t2( MarblePigment ( ColorMap( "[0.0  color White ][0.5  color White]"
                                              "[0.5  color Red   ][1.0  color Red  ]" ) ) );  (2)
   PlainTexture t3( SpottedPigment( ColorMap( "[0.2  color Red   ][0.45 color Green]"
                                              "[0.55 color Yellow][0.8  color Blue ]" ) ) );  (3)
   \endcode
 */
template< typename A    // Type of the first texture item or transformation
        , typename B    // Type of the second texture item or transformation
        , typename C    // Type of the third texture item or transformation
        , typename D    // Type of the fourth texture item or transformation
        , typename E >  // Type of the fifth texture item or transformation
PlainTexture::PlainTexture( const A& a, const B& b, const C& c, const D& d, const E& e )
   : Texture()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( E, ValidTypes );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );
   e.print( oss, "   ", true );

   oss.str().swap( texture_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the PlainTexture class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline PlainTexture::~PlainTexture()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Texture, PlainTexture );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a texture component/modifier to the plain POV-Ray texture.
 *
 * \param a The new texture component (pigment, finish, normal, scale, translation, rotation, ...).
 * \return void
 */
template< typename A >
void PlainTexture::add( const A& a )
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
