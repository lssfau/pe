//=================================================================================================
/*!
 *  \file pe/povray/Waves.h
 *  \brief Implementation of a waves normal for the POV-Ray visualization
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

#ifndef _PE_POVRAY_WAVES_H_
#define _PE_POVRAY_WAVES_H_


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
/*!\brief A POV-Ray waves normal.
 * \ingroup povray_normal
 *
 * The Waves class represents a POV-Ray normal using the waves keyword. The waves normal is
 * similar to the Ripples normal, except instead of creating smooth, even ripples, it creates
 * more rough and tumble waves that look more like ocean waves. The depth parameter adjusts
 * the size of the waves. A value of 0 creates a calm surface, the maximum value of 1 creates
 * tsunami-like waves.
 *
 * \image html waves.png
 * \image latex waves.eps "POV-Ray waves normal" width=400pt
 *
 * For the individual configuration of a waves normal, the following modifiers can be used
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
 * The following code example demonstrates the construction of a waves normal. The first
 * and obligatory parameter specifies the depth of the waves. Additionally, up to five
 * modifiers/transformations can be used to tune the waves effect:

   \code
   // Creating a default waves normal with a wave depth of 0.5
   pe::povray::Waves waves1( 0.5 );

   // Creating a wave normal with a wave depth of 0.3 and additional turbulence and frequency
   pe::povray::Waves waves2( 0.3, pe::povray::Turbulence( 0.5 ),
                               pe::povray::Frequency( 3 ) );

   // Creating a scaled and rotated wave normal with an individual turbulence setting
   pe::povray::Waves waves3( 0.4, pe::povray::Turbulence( 0.8 ),
                                  pe::povray::Octaves( 4 ),
                                  pe::povray::Omega( 0.3 ),
                                  pe::povray::Scale( 5.0 ),
                                  pe::povray::Rotation( 0.0, PI, 0.0 ) );
   \endcode
 */
class PE_PUBLIC Waves : public Normal
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
   explicit Waves( real depth );

   template< typename A >
   explicit Waves( real depth, const A& a );

   template< typename A, typename B >
   explicit Waves( real depth, const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit Waves( real depth, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit Waves( real depth, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit Waves( real depth, const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~Waves();
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
/*!\brief Creating a POV-Ray waves normal with one additional modifier/transformation.
 *
 * \param depth The depth of the waves effect \f$ [0..1] \f$.
 * \param a The additional modifier/transformation.
 *
 * This constructor creates a POV-Ray waves normal using one additional modifier or
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
 * The depth parameter for the waves has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A >  // Type of the modifier/transformation
Waves::Waves( real depth, const A& a )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   waves " << depth << "\n";
   a.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray waves normal with two additional modifiers/transformations.
 *
 * \param depth The depth of the waves effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 *
 * This constructor creates a POV-Ray waves normal using two additional modifiers or
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
 * The depth parameter for the waves has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B >  // Type of the second modifier/transformation
Waves::Waves( real depth, const A& a, const B& b )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   waves " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray waves normal with three additional modifiers/transformations.
 *
 * \param depth The depth of the waves effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 *
 * This constructor creates a POV-Ray waves normal using three additional modifiers or
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
 * The depth parameter for the waves has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C >  // Type of the third modifier/transformation
Waves::Waves( real depth, const A& a, const B& b, const C& c )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   waves " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray waves normal with four additional modifiers/transformations.
 *
 * \param depth The depth of the waves effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray waves normal using four additional modifiers or
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
 * The depth parameter for the waves has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D >  // Type of the fourth modifier/transformation
Waves::Waves( real depth, const A& a, const B& b, const C& c, const D& d )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   waves " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray waves normal with five additional modifiers/transformations.
 *
 * \param depth The depth of the waves effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 * \param e The fifth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray waves normal using five additional modifiers or
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
 * The depth parameter for the waves has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D    // Type of the fourth modifier/transformation
        , typename E >  // Type of the fifth modifier/transformation
Waves::Waves( real depth, const A& a, const B& b, const C& c, const D& d, const E& e )
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
   oss << "   waves " << depth << "\n";
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
/*!\brief Destructor for the Waves class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline Waves::~Waves()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Normal, Waves );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a modifier or transformation to the POV-Ray waves normal.
 *
 * \param a The new modifier/transformation.
 * \return void
 *
 * This function adds a new modifier or transformation to the waves normal. Valid modifiers are
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
void Waves::add( const A& a )
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
