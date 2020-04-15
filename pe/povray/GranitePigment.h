//=================================================================================================
/*!
 *  \file pe/povray/GranitePigment.h
 *  \brief Implementation of a POV-Ray granite pigment
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

#ifndef _PE_POVRAY_GRANITEPIGMENT_H_
#define _PE_POVRAY_GRANITEPIGMENT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/ColorMap.h>
#include <pe/povray/Frequency.h>
#include <pe/povray/Lambda.h>
#include <pe/povray/Octaves.h>
#include <pe/povray/Omega.h>
#include <pe/povray/Phase.h>
#include <pe/povray/Pigment.h>
#include <pe/povray/Transformation.h>
#include <pe/povray/Turbulence.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/TypeRestriction.h>
#include <pe/util/TypeList.h>


namespace pe {

namespace povray {

//*************************************************************************************************
/*!\brief A POV-Ray granite pigment.
 * \ingroup povray_pigment
 *
 * The GranitePigment class represent a granite POV-Ray pigment. The granite pigment creates a
 * sort of bozo-like color pattern. Typically, the result are pockets of color from one end of
 * the color map surrounded by rivers of colors from the other end. With the proper color map
 * it can look very convincingly like real granite. Granite uses the color map from 0 to 1
 * without reversing.\n
 * The following pictures show some examples for granite pigments. Very nice examples for
 * realistic granite pigments can be found in the "stones.inc" POV-Ray header file.
 *
 * \image html granitePigment.png
 * \image latex granitePigment.eps "Examples for granite pigments" width=800pt
 *
 * Granite pigments are created via the GranitePigment constructor:
 * -# GranitePigment::GranitePigment( const pe::povray::ColorMap& colormap );
 *
 * However, the appearance of the GranitePigment can be altered by directly specifying up to
 * five additional modifiers or transformations or by using the add() function.
 *
 * Examples:

   \code
   using namespace pe::pov;

   // Specification of a color map
   ColorMap colormap( "[0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red]" );

   // Creating a plain pigment without further modifications
   GranitePigment plain( colormap );

   // Creating a distorted pigment by specifying additional turbulence modifiers
   GranitePigment distorted( colormap, Turbulence( 0.8 ), Omega( 0.6 ) );

   // Adding an additional turbulence modifier
   distorted.add( Lambda( 2.0 ) );
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!
 */
class PE_PUBLIC GranitePigment : public Pigment
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_7( Turbulence, Lambda, Octaves, Omega,
                          Frequency, Phase, Transformation )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit GranitePigment( const ColorMap& colormap );

   template< typename A >
   explicit GranitePigment( const ColorMap& colormap, const A& a );

   template< typename A, typename B >
   explicit GranitePigment( const ColorMap& colormap, const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit GranitePigment( const ColorMap& colormap, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit GranitePigment( const ColorMap& colormap, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit GranitePigment( const ColorMap& colormap, const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~GranitePigment();
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
/*!\brief Creating a POV-Ray granite pigment with one additional modifier/transformation.
 *
 * \param colormap The color map for the granite pigment.
 * \param a The additional modifier/transformation.
 *
 * This constructor creates a POV-Ray granite pigment using one additional modifier or
 * transformation. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *  - Frequency
 *  - Phase
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >  // Type of the modifier/transformation
GranitePigment::GranitePigment( const ColorMap& colormap, const A& a )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << "   granite\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray granite pigment with two additional modifier/transformation.
 *
 * \param colormap The color map for the granite pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 *
 * This constructor creates a POV-Ray granite pigment using two additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *  - Frequency
 *  - Phase
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A    // Type of the first modifier/transformation
        , typename B >  // Type of the second modifier/transformation
GranitePigment::GranitePigment( const ColorMap& colormap, const A& a, const B& b )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   std::ostringstream oss;
   oss << "   granite\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray granite pigment with three additional modifier/transformation.
 *
 * \param colormap The color map for the granite pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 *
 * This constructor creates a POV-Ray granite pigment using three additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *  - Frequency
 *  - Phase
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C >  // Type of the third modifier/transformation
GranitePigment::GranitePigment( const ColorMap& colormap, const A& a, const B& b, const C& c )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   std::ostringstream oss;
   oss << "   granite\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray granite pigment with four additional modifier/transformation.
 *
 * \param colormap The color map for the granite pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray granite pigment using four additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *  - Frequency
 *  - Phase
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D >  // Type of the fourth modifier/transformation
GranitePigment::GranitePigment( const ColorMap& colormap,
                              const A& a, const B& b, const C& c, const D& d )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   std::ostringstream oss;
   oss << "   granite\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray granite pigment with five additional modifier/transformation.
 *
 * \param colormap The color map for the granite pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 * \param e The fifth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray granite pigment using five additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *  - Frequency
 *  - Phase
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D    // Type of the fourth modifier/transformation
        , typename E >  // Type of the fifth modifier/transformation
GranitePigment::GranitePigment( const ColorMap& colormap,
                              const A& a, const B& b, const C& c, const D& d, const E& e )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( E, ValidTypes );

   std::ostringstream oss;
   oss << "   granite\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );
   e.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the GranitePigment class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline GranitePigment::~GranitePigment()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Pigment, GranitePigment );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a modifier or transformation to the POV-Ray granite pigment.
 *
 * \param a The new modifier/transformation.
 * \return void
 *
 * This function adds a new modifier or transformation to the granite pigment. Valid
 * modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *  - Frequency
 *  - Phase
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >  // Type of the modifier/transformation
void GranitePigment::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << pigment_;
   a.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
