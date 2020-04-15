//=================================================================================================
/*!
 *  \file pe/povray/Finish.h
 *  \brief Implementation of a POV-Ray finish
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

#ifndef _PE_POVRAY_FINISH_H_
#define _PE_POVRAY_FINISH_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <sstream>
#include <string>
#include <pe/povray/FinishItem.h>
#include <pe/povray/TextureItem.h>
#include <pe/util/constraints/SameType.h>
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
/*!\defgroup povray_finish Finish
 * \ingroup povray
 *
 * After the pigment, the finish is the second most important part of a texture: it finishes the
 * appearance of a pigment. A finish is applied to define how rigid bodies interact with light.
 * For instance, a finish can create a mirrored body, make it appear polished or glowing, as
 * illustrated in the following image:
 *
 * \image html reflection.png
 * \image latex reflection.eps "Examples for reflection" width=400pt
 *
 * The following code examples give an impression of how a finish can be created. A finish can
 * be directly created with up to five finish item (for instance Ambient, Diffuse, Phong, ...)
 * or extended by the add() function:

   \code
   // Creating a finish consisting of three finish items
   pe::povray::Finish finish( pe::povray::Ambient( 0.1 ),
                              pe::povray::Diffuse( 0.6 ),
                              pe::povray::Phong( 0.9 )   );

   // Extending the POV-Ray finish with the add() function
   finish.add( pe::povray::Reflection( 0.05 ) );

   // Using the finish to finalize a texture
   pe::povray::Texture texture( pe::povray::ColorPigment( 1, 1, 0 ),
                                finish );
   \endcode

 * Possible finish items include ambient and diffuse lighting, highlights, reflection effects and
 * refraction:\n\n
 *
 *
 * \section finish_ambient Ambient lighting
 *
 * Ambient lighting specifies the luminance of the body itself. The lighting value is in the range
 * \f$ [0..1] \f$. If the ambient lighting is set to 0, the rigid body will appear black if it is
 * not directly lighted by a light source. The following demonstrates the ambient lighting effect:
 * the first image uses the POV-Ray default of 0.1, the second was rendered with an ambient value
 * of 0.6.
 *
 * \image html ambient.png
 * \image latex ambient.eps "Examples for ambient lighting" width=400pt
 *
 * For more details about ambient lighting see the Ambient class description.\n\n
 *
 *
 * \section finish_diffuse Diffuse lighting
 *
 * Diffuse lighting specifies how much light is diffused at the surface of the rigid body. The
 * lighting value has to be in the range \f$ [0..1] \f$. The images give an impression of three
 * diffuse lighting values: the first image uses a diffuse lighting of 0.3, the second was
 * rendered with the POV-Ray default of 0.6 and the third has a diffuse value of 0.9.
 *
 * \image html diffuse.png
 * \image latex diffuse.eps "Examples for diffuse lighting" width=600pt
 *
 * For more information about the diffuse lighting effect, see the Diffuse class description.\n\n
 *
 *
 * \section finish_phong Phong highlighting
 *
 * Adding a phong highlight to a finish creates a highlight on the rigid body that is the color
 * of the light source. The phong value specifies the saturation of the highlight and has to be
 * in the range \f$ [0..1] \f$. Additionally, the size of the phong highlight can be specified.
 * The phong size has to be in the range \f$ [0..\infty) \f$.\n
 * Phong highlights are similar to specular highlights. However, specular highlights are more
 * accurate as far as physical laws are concerned. Note that phong highlights and specular
 * highlights are usually not both used on the same body!\n
 * The following images illustrate the effect of the phong value. The first image shows a sphere
 * without a phong highlight, the second image demonstrates a phong highlight of strength 0.9.
 *
 * \image html phong.png
 * \image latex phong.eps "Examples for the phong highlight" width=400pt
 *
 * The next images illustrate the effect of the phong size value: the first image has a phong
 * size of 180, the second uses the POV-Ray default of 40 and the third has a phong size of 4.
 *
 * \image html phongsize.png
 * \image latex phongsize.eps "Examples for the phong size" width=600pt
 *
 * For more information about phong highlighting, see both the Phong and the PhongSize class
 * descriptions.\n\n
 *
 *
 * \section finish_specular Specular highlighting
 *
 * A specular highlight creates a highlight on the rigid body that is the color of the light
 * source. The specular value specifies the saturation of the highlight and has to be in the
 * range \f$ [0..1] \f$. Additionally, the size of the specular highlight can be specified via
 * the roughness parameter that has to be in the range \f$ (0..1] \f$.\n
 * The following images illustrate the effect of the specular value. The first image shows a
 * sphere without a specular highlight, the second image demonstrates a specular highlight of
 * strength 0.6.
 *
 * \image html specular.png
 * \image latex specular.eps "Examples for the specular highlight" width=400pt
 *
 * The next images illustrate the effect of the roughness parameter: the first image has a
 * roughness of 0.01, the second uses the POV-Ray default of 0.05 and the third has a roughness
 * of 0.1.
 *
 * \image html roughness.png
 * \image latex roughness.eps "Examples for the roughness" width=600pt
 *
 * For more information about specular highlighting, see the both the Specular and Roughness
 * class description.\n\n
 *
 *
 * \section finish_reflection Reflections
 *
 * The reflection finish gives a rigid body a mirrored or partially mirrored surface. This body
 * will then reflect other bodies in the scene. The relection value has to be in the range
 * \f$ [0..1] \f$. A value of 0 turns the reflection off, a value of 1 gives the body an almost
 * perfectly mirrored surface. The images illustrate the reflection effect: the first image has
 * no reflection, whereas the second one uses a reflection of 0.3.
 *
 * \image html reflection.png
 * \image latex reflection.eps "Examples for reflection" width=400pt
 *
 * For more information about the reflection effect, see the Reflection class description.\n\n
 *
 *
 * \section finish_refraction Refraction
 *
 * Refraction only has meaning on rigid bodies that have at least a litte bit of transparency.
 * Refraction is the bending of light rays as they pass into a more dense or less dense medium.
 * As light does not go through opaque things, they don't refract. Without refraction, transparent
 * bodies look like colored air. The refraction value has to be in the range \f$ [1..\infty) \f$.
 * A value of 1.0 is the POV-Ray default and will not change the refraction of a body. Examples for
 * some physical refraction values are 1.000292 for air, 1.33 for water or 1.5 for glass. The
 * images illustrate the refraction effect: the first image has no refraction, whereas the second
 * image uses a refraction of 2.0.
 *
 * \image html refraction.png
 * \image latex refraction.eps "Examples for refraction" width=400pt
 *
 * For more information about refraction, see the Refraction class description.
 */
/*!\brief A POV-Ray finish.
 * \ingroup povray_finish
 *
 * The Finish class represents a POV-Ray finish for rigid bodies. It implements the necessary
 * functionality to act as a TextureItem.
 */
class PE_PUBLIC Finish : public TextureItem
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( FinishItem )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef Finish  Type;  //!< Type of the TextureItem.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline Finish();

   template< typename A >
   explicit Finish( const A& a );

   template< typename A, typename B >
   explicit Finish( const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit Finish( const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit Finish( const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit Finish( const A& a, const B& b, const C& c, const D& d, const E& e );

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
   template< typename A >
   void add( const A& a );

   inline void reset();
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os,                  bool newline ) const;
   void print( std::ostream& os, const char* tab, bool newline ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::string finish_;  //!< POV-Ray string representation of the finish.
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
/*!\brief The default constructor for Finish.
 */
inline Finish::Finish()
   : TextureItem()  // Initialization of the base class
   , finish_()      // POV-Ray string representation of the finish
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray finish consisting of a single finish component.
 *
 * \param a The finish component.
 *
 * This constructor creates a POV-Ray finish using the given finish item \a a. Valid finish
 * items are
 *  - Ambient
 *  - Diffuse
 *  - Phong
 *  - PhongSize
 *  - Specular
 *  - Roughness
 *  - Reflection
 *  - Refraction
 *
 * In case \a A is not a finish item, a compile time error is created!
 */
template< typename A >  // Type of the finish component
Finish::Finish( const A& a )
   : TextureItem()  // Initialization of the base class
   , finish_()      // POV-Ray string representation of the finish
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   a.print( oss, "   ", true );

   oss.str().swap( finish_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray finish consisting of two finish components.
 *
 * \param a The first finish component.
 * \param b The second finish component.
 *
 * This constructor creates a POV-Ray finish using the two given finish items \a a and \a b.
 * Valid finish items are
 *  - Ambient
 *  - Diffuse
 *  - Phong
 *  - PhongSize
 *  - Specular
 *  - Roughness
 *  - Reflection
 *  - Refraction
 *
 * In case either \a A or \a B is not a finish item or in case \a A and \a B happen to be the
 * same item, a compile time error is created!
 */
template< typename A    // Type of the first finish component
        , typename B >  // Type of the second finish component
Finish::Finish( const A& a, const B& b )
   : TextureItem()  // Initialization of the base class
   , finish_()      // POV-Ray string representation of the finish
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( finish_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray finish consisting of three finish components.
 *
 * \param a The first finish component.
 * \param b The second finish component.
 * \param c The third finish component.
 *
 * This constructor creates a POV-Ray finish using the three given finish items \a a, \a b and
 * \a c. Valid finish items are
 *  - Ambient
 *  - Diffuse
 *  - Phong
 *  - PhongSize
 *  - Specular
 *  - Roughness
 *  - Reflection
 *  - Refraction
 *
 * In case at least one of the given parameters is not a finish item or in case two of these
 * items have the same type, a compile time error is created!
 */
template< typename A    // Type of the first finish component
        , typename B    // Type of the second finish component
        , typename C >  // Type of the third finish component
Finish::Finish( const A& a, const B& b, const C& c )
   : TextureItem()  // Initialization of the base class
   , finish_()      // POV-Ray string representation of the finish
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, C );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( finish_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray finish consisting of four finish components.
 *
 * \param a The first finish component.
 * \param b The second finish component.
 * \param c The third finish component.
 * \param d The fourth finish component.
 *
 * This constructor creates a POV-Ray finish using the four given finish items \a a, \a b, \a c
 * and \a d. Valid finish items are
 *  - Ambient
 *  - Diffuse
 *  - Phong
 *  - PhongSize
 *  - Specular
 *  - Roughness
 *  - Reflection
 *  - Refraction
 *
 * In case at least one of the given parameters is not a finish item or in case two of these
 * item have the same type, a compile time error is created!
 */
template< typename A    // Type of the first finish component
        , typename B    // Type of the second finish component
        , typename C    // Type of the third finish component
        , typename D >  // Type of the fourth finish component
Finish::Finish( const A& a, const B& b, const C& c, const D& d )
   : TextureItem()  // Initialization of the base class
   , finish_()      // POV-Ray string representation of the finish
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( C, D );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( finish_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray finish consisting of four finish components.
 *
 * \param a The first finish component.
 * \param b The second finish component.
 * \param c The third finish component.
 * \param d The fourth finish component.
 * \param e The fifth finish component.
 *
 * This constructor creates a POV-Ray finish using the five given finish items \a a, \a b, \a c,
 * \a d and \a e. Valid finish items are
 *  - Ambient
 *  - Diffuse
 *  - Phong
 *  - PhongSize
 *  - Specular
 *  - Roughness
 *  - Reflection
 *  - Refraction
 *
 * In case at least one of the given parameters is not a finish item or in case
 * two of these item have the same type, a compile time error is created!
 */
template< typename A    // Type of the first finish component
        , typename B    // Type of the second finish component
        , typename C    // Type of the third finish component
        , typename D    // Type of the fourth finish component
        , typename E >  // Type of the fifth finish component
Finish::Finish( const A& a, const B& b, const C& c, const D& d, const E& e )
   : TextureItem()  // Initialization of the base class
   , finish_()      // POV-Ray string representation of the finish
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( E, ValidTypes );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, B );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( A, E );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, C );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( B, E );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( C, D );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( C, E );
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( D, E );

   std::ostringstream oss;
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );
   e.print( oss, "   ", true );

   oss.str().swap( finish_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a finish component to the POV-Ray finish.
 *
 * \param a The new finish component.
 * \return void
 *
 * This function adds a new finish item to the finish. Valid finish items are
 *  - Ambient
 *  - Diffuse
 *  - Phong
 *  - PhongSize
 *  - Specular
 *  - Roughness
 *  - Reflection
 *  - Refraction
 *
 * In case the given parameter is not a finish item, a compile time error is created!
 */
template< typename A >  // Type of the finish component
void Finish::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << finish_;
   a.print( oss, "   ", true );

   oss.str().swap( finish_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the POV-Ray finish.
 *
 * \return void
 *
 * This function removes all components from the POV-Ray finish and resets all values to the
 * POV-Ray defaults.
 */
inline void Finish::reset()
{
   finish_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Finish operators */
//@{
std::ostream& operator<<( std::ostream& os, const Finish& finish );
//@}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
