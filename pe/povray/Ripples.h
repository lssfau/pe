//=================================================================================================
/*!
 *  \file pe/povray/Ripples.h
 *  \brief Implementation of a ripples normal for the POV-Ray visualization
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

#ifndef _PE_POVRAY_RIPPLES_H_
#define _PE_POVRAY_RIPPLES_H_


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
/*!\brief A POV-Ray ripples normal.
 * \ingroup povray_normal
 *
 * The Ripples class repesents a POV-Ray normal using the ripples keyword. The ripples normal
 * consists of evenly spaced, smooth ripples which originate from 10 random locations inside
 * the box with corners (0,0,0) and (1,1,1). In order to shift the origin of the ripples to
 * a different location, Translation/Rotation transformations have to be used. All resulting
 * waves have the same frequency, so the ripple effect is smooth at a significant distance
 * from the center.
 *
 * \image html ripples.png
 * \image latex ripples.eps "POV-Ray ripples normal" width=400pt
 *
 * For the individual configuration of a ripples normal, the following modifiers can be used
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
 * The following code example demonstrates the construction of a ripples normal. The first
 * and obligatory parameter specifies the depth of the ripples. Additionally, up to five
 * modifiers/transformations can be used to tune the ripples effect:

   \code
   // Creating a default ripples normal with a ripple depth of 0.5
   pe::povray::Ripples ripples1( 0.5 );

   // Creating a ripple normal with a ripple depth of 0.3 and additional turbulence and frequency
   pe::povray::Ripples ripples2( 0.3, pe::povray::Turbulence( 0.5 ),
                                      pe::povray::Frequency( 3 ) );

   // Creating a scaled and rotated ripple normal with an individual turbulence setting
   pe::povray::Ripples ripples3( 0.4, pe::povray::Turbulence( 0.8 ),
                                      pe::povray::Octaves( 4 ),
                                      pe::povray::Omega( 0.3 ),
                                      pe::povray::Scale( 5.0 ),
                                      pe::povray::Rotation( 0.0, PI, 0.0 ) );
   \endcode
 *
 */
class PE_PUBLIC Ripples : public Normal
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
   explicit Ripples( real depth );

   template< typename A >
   explicit Ripples( real depth, const A& a );

   template< typename A, typename B >
   explicit Ripples( real depth, const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit Ripples( real depth, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit Ripples( real depth, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit Ripples( real depth, const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~Ripples();
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
/*!\brief Creating a POV-Ray ripples normal with one additional modifier/transformation.
 *
 * \param depth The depth of the ripples effect \f$ [0..1] \f$.
 * \param a The additional modifier/transformation.
 *
 * This constructor creates a POV-Ray ripples normal using one additional modifier or
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
 * The depth parameter for the ripples has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A >  // Type of the modifier/transformation
Ripples::Ripples( real depth, const A& a )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   ripples " << depth << "\n";
   a.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray ripples normal with two additional modifiers/transformations.
 *
 * \param depth The depth of the ripples effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 *
 * This constructor creates a POV-Ray ripples normal using two additional modifiers or
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
 * The depth parameter for the ripples has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B >  // Type of the second modifier/transformation
Ripples::Ripples( real depth, const A& a, const B& b )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   ripples " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray ripples normal with three additional modifiers/transformations.
 *
 * \param depth The depth of the ripples effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 *
 * This constructor creates a POV-Ray ripples normal using three additional modifiers or
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
 * The depth parameter for the ripples has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C >  // Type of the third modifier/transformation
Ripples::Ripples( real depth, const A& a, const B& b, const C& c )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   ripples " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray ripples normal with four additional modifiers/transformations.
 *
 * \param depth The depth of the ripples effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray ripples normal using four additional modifiers or
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
 * The depth parameter for the ripples has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D >  // Type of the fourth modifier/transformation
Ripples::Ripples( real depth, const A& a, const B& b, const C& c, const D& d )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   ripples " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray ripples normal with five additional modifiers/transformations.
 *
 * \param depth The depth of the ripples effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 * \param e The fifth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray ripples normal using five additional modifiers or
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
 * The depth parameter for the ripples has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D    // Type of the fourth modifier/transformation
        , typename E >  // Type of the fifth modifier/transformation
Ripples::Ripples( real depth, const A& a, const B& b, const C& c, const D& d, const E& e )
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
   oss << "   ripples " << depth << "\n";
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
/*!\brief Destructor for the Ripples class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline Ripples::~Ripples()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Normal, Ripples );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a modifier or transformation to the POV-Ray ripples normal.
 *
 * \param a The new modifier/transformation.
 * \return void
 *
 * This function adds a new modifier or transformation to the ripples normal. Valid modifiers are
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
void Ripples::add( const A& a )
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
