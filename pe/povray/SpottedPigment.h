//=================================================================================================
/*!
 *  \file pe/povray/SpottedPigment.h
 *  \brief Implementation of a POV-Ray spotted pigment
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

#ifndef _PE_POVRAY_SPOTTEDPIGMENT_H_
#define _PE_POVRAY_SPOTTEDPIGMENT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/ColorMap.h>
#include <pe/povray/Frequency.h>
#include <pe/povray/Phase.h>
#include <pe/povray/Pigment.h>
#include <pe/povray/Transformation.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/TypeRestriction.h>
#include <pe/util/TypeList.h>


namespace pe {

namespace povray {

//*************************************************************************************************
/*!\brief A POV-Ray spotted pigment.
 * \ingroup povray_pigment
 *
 * The SpottedPigment class represents a spotted POV-Ray pigment. Spotted pigments use a
 * POV-Ray ColorMap to create colored spots on the rigid bodies. The images demonstrate
 * several spotted pigments (see the code example below for the setup details):
 *
 * \image html spottedPigment.png
 * \image latex spottedPigment.eps "Examples for spotted pigments" width=800pt
 *
 * Spotted pigments are created via the \a SpottedPigment constructor:
 * -# SpottedPigment::SpottedPigment( const pe::povray::ColorMap& colormap );
 *
 * However, the appearance of the SpottedPigment can be altered by directly specifying up
 * to five additional modifiers or transformations or by using the add() function.
 *
 * Examples:

   \code
   using namespace pe::pov;

   // Creating a red dotted pigment
   ColorMap colormap1( "[0.4 color Red][0.4 color White]" )
   SpottedPigment reddots( colormap1 );

   // Creating a red/white spotted pigment
   ColorMap colormap2( "[0.5 color Red][0.5 color White]" )
   SpottedPigment redwhite( colormap2 );

   // Creating a white dotted pigment
   ColorMap colormap3( "[0.6 color Red][0.6 color White]" )
   SpottedPigment whitedots( colormap3 );

   // Creating a colorful spotted pigment
   ColorMap colormap4( "[0.2 color Red][0.45 color Green][0.55 color Yellow][0.8 color Blue]" );
   SpottedPigment colorful( colormap4, Frequency( 1.1 ) );

   Pigment reddots   = SpottedPigment( colormap );
   Pigment whitedots = SpottedPigment( "[0.6 color Red][0.6 color White]" );
   \endcode

 * \b Note: For an explanation of the color map parameter string format, take a look at the
 * details of the ColorMap class!
 */
class PE_PUBLIC SpottedPigment : public Pigment
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_3( Frequency, Phase, Transformation )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit SpottedPigment( const ColorMap& colormap );

   template< typename A >
   explicit SpottedPigment( const ColorMap& colormap, const A& a );

   template< typename A, typename B >
   explicit SpottedPigment( const ColorMap& colormap, const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit SpottedPigment( const ColorMap& colormap, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit SpottedPigment( const ColorMap& colormap, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit SpottedPigment( const ColorMap& colormap, const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~SpottedPigment();
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
/*!\brief Creating a POV-Ray spotted pigment with one additional modifier/transformation.
 *
 * \param colormap The color map for the spotted pigment.
 * \param a The additional modifier/transformation.
 *
 * This constructor creates a POV-Ray spotted pigment using one additional modifier or
 * transformation. Valid modifiers are
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
SpottedPigment::SpottedPigment( const ColorMap& colormap, const A& a )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << "   spotted\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray spotted pigment with two additional modifier/transformation.
 *
 * \param colormap The color map for the spotted pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 *
 * This constructor creates a POV-Ray spotted pigment using two additional modifiers or
 * transformations. Valid modifiers are
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
SpottedPigment::SpottedPigment( const ColorMap& colormap, const A& a, const B& b )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   std::ostringstream oss;
   oss << "   spotted\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray spotted pigment with three additional modifier/transformation.
 *
 * \param colormap The color map for the spotted pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 *
 * This constructor creates a POV-Ray spotted pigment using three additional modifiers or
 * transformations. Valid modifiers are
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
SpottedPigment::SpottedPigment( const ColorMap& colormap, const A& a, const B& b, const C& c )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   std::ostringstream oss;
   oss << "   spotted\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray spotted pigment with four additional modifier/transformation.
 *
 * \param colormap The color map for the spotted pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray spotted pigment using four additional modifiers or
 * transformations. Valid modifiers are
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
SpottedPigment::SpottedPigment( const ColorMap& colormap,
                              const A& a, const B& b, const C& c, const D& d )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   std::ostringstream oss;
   oss << "   spotted\n";
   colormap.print( oss, "   ", true );
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray spotted pigment with five additional modifier/transformation.
 *
 * \param colormap The color map for the spotted pigment.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 * \param e The fifth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray spotted pigment using five additional modifiers or
 * transformations. Valid modifiers are
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
SpottedPigment::SpottedPigment( const ColorMap& colormap,
                              const A& a, const B& b, const C& c, const D& d, const E& e )
   : Pigment()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( E, ValidTypes );

   std::ostringstream oss;
   oss << "   spotted\n";
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
/*!\brief Destructor for the SpottedPigment class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline SpottedPigment::~SpottedPigment()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Pigment, SpottedPigment );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a modifier or transformation to the POV-Ray spotted pigment.
 *
 * \param a The new modifier/transformation.
 * \return void
 *
 * This function adds a new modifier or transformation to the spotted pigment. Valid
 * modifiers are
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
void SpottedPigment::add( const A& a )
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
