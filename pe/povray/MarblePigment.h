//=================================================================================================
/*!
 *  \file pe/povray/MarblePigment.h
 *  \brief Implementation of a POV-Ray marble pigment
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

#ifndef _PE_POVRAY_MARBLEPIGMENT_H_
#define _PE_POVRAY_MARBLEPIGMENT_H_


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
/*!\brief A POV-Ray marble pigment.
 * \ingroup povray_pigment
 *
 * The MarblePigment class represent a marble POV-Ray pigment. Marble pigments map a POV-Ray
 * ColorMap on the rigid body that can be distorted by a specified level of turbulence. The
 * images demonstrate some possible marble pigments: the first pigment uses no turbulence,
 * the second uses a turbulence value of 0.8 and the third example uses the POV-Ray pigment
 * "T_Stone9" from the "stones.inc" header file.
 *
 * \image html marblePigment.png
 * \image latex marblePigment.eps "Examples for marble pigments" width=600pt
 *
 * Marble pigments are created via the MarblePigment constructor:
 * -# MarblePigment::MarblePigment( const pe::povray::ColorMap& colormap );
 *
 * However, the appearance of the MarblePigment can be altered by directly specifying up to
 * five additional modifiers or transformations or by using the add() function.
 *
 * Examples:

   \code
   using namespace pe::pov;

   // Specification of a color map
   ColorMap colormap( "[0.5 color Red][0.5 color White]" );

   // Creating a plain pigment without further modifications
   MarblePigment striped( colormap );

   // Creating a distorted pigment by specifying additional turbulence modifiers
   MarblePigment distorted( colormap, Turbulence( 0.8 ), Omega( 0.4 ) );

   // Adding an additional turbulence modifier
   distorted.add( Lambda( 1.5 ) );
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!
 */
class PE_PUBLIC MarblePigment : public Pigment
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
   explicit MarblePigment( const ColorMap& colormap );

   template< typename A >
   explicit MarblePigment( const ColorMap& colormap, const A& a );

   template< typename A, typename B >
   explicit MarblePigment( const ColorMap& colormap, const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit MarblePigment( const ColorMap& colormap, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit MarblePigment( const ColorMap& colormap, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit MarblePigment( const ColorMap& colormap, const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~MarblePigment();
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
/*!\brief Creating a POV-Ray marble pigment with one additional modifier/transformation.
 *
 * \param colormap The color map for the marble pigment.
 * \param a The additional modifier/transformation.
 *
 * This constructor creates a POV-Ray marble pigment using one additional modifier or
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
MarblePigment::MarblePigment( const ColorMap& colormap, const A& a )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << "   marble\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray marble pigment with two additional modifier/transformation.
 *
 * \param colormap The color map for the marble pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 *
 * This constructor creates a POV-Ray marble pigment using two additional modifiers or
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
MarblePigment::MarblePigment( const ColorMap& colormap, const A& a, const B& b )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   std::ostringstream oss;
   oss << "   marble\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray marble pigment with three additional modifier/transformation.
 *
 * \param colormap The color map for the marble pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 *
 * This constructor creates a POV-Ray marble pigment using three additional modifiers or
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
MarblePigment::MarblePigment( const ColorMap& colormap, const A& a, const B& b, const C& c )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   std::ostringstream oss;
   oss << "   marble\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray marble pigment with four additional modifier/transformation.
 *
 * \param colormap The color map for the marble pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray marble pigment using four additional modifiers or
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
MarblePigment::MarblePigment( const ColorMap& colormap,
                              const A& a, const B& b, const C& c, const D& d )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   std::ostringstream oss;
   oss << "   marble\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray marble pigment with five additional modifier/transformation.
 *
 * \param colormap The color map for the marble pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 * \param e The fifth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray marble pigment using five additional modifiers or
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
MarblePigment::MarblePigment( const ColorMap& colormap,
                              const A& a, const B& b, const C& c, const D& d, const E& e )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( E, ValidTypes );

   std::ostringstream oss;
   oss << "   marble\n";
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
/*!\brief Destructor for the MarblePigment class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline MarblePigment::~MarblePigment()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Pigment, MarblePigment );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a modifier or transformation to the POV-Ray marble pigment.
 *
 * \param a The new modifier/transformation.
 * \return void
 *
 * This function adds a new modifier or transformation to the marble pigment. Valid
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
void MarblePigment::add( const A& a )
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
