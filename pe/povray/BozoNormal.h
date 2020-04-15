//=================================================================================================
/*!
 *  \file pe/povray/BozoNormal.h
 *  \brief Implementation of an bozo normal for the POV-Ray visualization
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

#ifndef _PE_POVRAY_BOZONORMAL_H_
#define _PE_POVRAY_BOZONORMAL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <stdexcept>
#include <pe/povray/Frequency.h>
#include <pe/povray/Lambda.h>
#include <pe/povray/Normal.h>
#include <pe/povray/Octaves.h>
#include <pe/povray/Omega.h>
#include <pe/povray/Phase.h>
#include <pe/povray/Transformation.h>
#include <pe/povray/Turbulence.h>
#include <pe/system/Precision.h>
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
/*!\brief A POV-Ray bozo normal.
 * \ingroup povray_normal
 *
 * The BozoNormal class acts as a veneer class for the Normal class to create an bozo normal
 * for the POV-Ray visualization. Bozo normals create a random, bumpy pattern similar to the
 * Bumps normal. However, the effect of the bozo normal is weaker and softer. The following
 * image gives an impression of a bozo normal:
 *
 * \image html bozoNormal.png
 * \image latex bozoNormal.eps "POV-Ray bozo normal" width=400pt
 *
 * For the individual configuration of a bozo normal, the following modifiers can be used
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
 * Any other modifiers/transformations result in a compile time error!\n
 * The following code example demonstrates the construction of a bozo normal. The first
 * and obligatory parameter specifies the size of the normal effect. Additionally, up to
 * five modifiers/transformations can be used to tune the bozo effect:

   \code
   // Creating a default bozo normal with a depth of 0.5
   pe::povray::BozoNormal bozo1( 0.5 );

   // Creating an bozo normal with a depth of 0.3 and additional turbulence modifier
   pe::povray::BozoNormal bozo2( 0.3, pe::povray::Turbulence( 0.5 ) );

   // Creating a scaled and rotated bozo normal with an individual turbulence setting
   pe::povray::BozoNormal bozo3( 0.4, pe::povray::Turbulence( 0.8 ),
                                      pe::povray::Octaves( 4 ),
                                      pe::povray::Omega( 0.3 ),
                                      pe::povray::Scale( 5.0 ),
                                      pe::povray::Rotation( 0.0, PI, 0.0 ) );
   \endcode
 */
class PE_PUBLIC BozoNormal : public Normal
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
   explicit BozoNormal( real depth );

   template< typename A >
   explicit BozoNormal( real depth, const A& a );

   template< typename A, typename B >
   explicit BozoNormal( real depth, const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit BozoNormal( real depth, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit BozoNormal( real depth, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit BozoNormal( real depth, const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~BozoNormal();
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
/*!\brief Creating a POV-Ray bozo normal with one additional modifier/transformation.
 *
 * \param depth The depth of the bozo effect \f$ [0..1] \f$.
 * \param a The additional modifier/transformation.
 *
 * This constructor creates a POV-Ray bozo normal using one additional modifier or
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
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the bozo normal has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A >  // Type of the modifier/transformation
BozoNormal::BozoNormal( real depth, const A& a )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   bozo " << depth << "\n";
   a.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray bozo normal with two additional modifiers/transformations.
 *
 * \param depth The depth of the bozo effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 *
 * This constructor creates a POV-Ray bozo normal using two additional modifiers or
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
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the bozo normal has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B >  // Type of the second modifier/transformation
BozoNormal::BozoNormal( real depth, const A& a, const B& b )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   bozo " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray bozo normal with three additional modifiers/transformations.
 *
 * \param depth The depth of the bozo effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 *
 * This constructor creates a POV-Ray bozo normal using three additional modifiers or
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
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the bozo normal has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C >  // Type of the third modifier/transformation
BozoNormal::BozoNormal( real depth, const A& a, const B& b, const C& c )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   bozo " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray bozo normal with four additional modifiers/transformations.
 *
 * \param depth The depth of the bozo effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray bozo normal using four additional modifiers or
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
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the bozo normal has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D >  // Type of the fourth modifier/transformation
BozoNormal::BozoNormal( real depth, const A& a, const B& b, const C& c, const D& d )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   bozo " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray bozo normal with five additional modifiers/transformations.
 *
 * \param depth The depth of the bozo effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 * \param e The fifth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray bozo normal using five additional modifiers or
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
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the bozo normal has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D    // Type of the fourth modifier/transformation
        , typename E >  // Type of the fifth modifier/transformation
BozoNormal::BozoNormal( real depth, const A& a, const B& b, const C& c, const D& d, const E& e )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( E, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   bozo " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );
   e.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the BozoNormal class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline BozoNormal::~BozoNormal()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Normal, BozoNormal );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a modifier or transformation to the POV-Ray bozo normal.
 *
 * \param a The new modifier/transformation.
 * \return void
 *
 * This function adds a new modifier or transformation to the bozo normal. Valid modifiers are
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
void BozoNormal::add( const A& a )
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   std::ostringstream oss;
   oss << normal_;
   a.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe

#endif
