//=================================================================================================
/*!
 *  \file pe/povray/Dents.h
 *  \brief Implementation of a dents normal for the POV-Ray visualization
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

#ifndef _PE_POVRAY_DENTS_H_
#define _PE_POVRAY_DENTS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <stdexcept>
#include <pe/povray/Lambda.h>
#include <pe/povray/Normal.h>
#include <pe/povray/Octaves.h>
#include <pe/povray/Omega.h>
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
/*!\brief A POV-Ray dents normal.
 * \ingroup povray_normal
 *
 * The Dents class acts as a veneer class for the Normal class to create a dents normal for
 * the POV-Ray visualization. The dents normal basically makes an object look like someone
 * attacked it with a sledgehammer. The surface appears to have dents beaten into it. The
 * depth parameter adjustes the size of the dents. A value of 0 creates the impression of
 * a brand new object (and turn off the normal effect), a value of 1 gives the impression
 * that the object survived a hail storm.
 *
 * \image html dents.png
 * \image latex dents.eps "POV-Ray dents normal" width=400pt
 *
 * For the individual configuration of a dents normal, the following modifiers can be used
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * Any other modifiers/transformations result in a compile time error!\n
 * The following code example demonstrates the construction of a dents normal. The first
 * and obligatory parameter specifies the depth of the dents. Additionally, up to five
 * modifiers/transformations can be used to tune the dents effect:

   \code
   // Creating a default dents normal with a bump depth of 0.5
   pe::povray::Dents dents1( 0.5 );

   // Creating a wave normal with a wave depth of 0.3 and additional turbulence modifier
   pe::povray::Dents dents2( 0.3, pe::povray::Turbulence( 0.5 ) );

   // Creating a scaled and rotated wave normal with an individual turbulence setting
   pe::povray::Dents dents3( 0.4, pe::povray::Turbulence( 0.8 ),
                                  pe::povray::Octaves( 4 ),
                                  pe::povray::Omega( 0.3 ),
                                  pe::povray::Scale( 5.0 ),
                                  pe::povray::Rotation( 0.0, PI, 0.0 ) );
   \endcode
 */
class PE_PUBLIC Dents : public Normal
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_5( Turbulence, Lambda, Octaves,
                          Omega, Transformation )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Dents( real depth );

   template< typename A >
   explicit Dents( real depth, const A& a );

   template< typename A, typename B >
   explicit Dents( real depth, const A& a, const B& b );

   template< typename A, typename B, typename C >
   explicit Dents( real depth, const A& a, const B& b, const C& c );

   template< typename A, typename B, typename C, typename D >
   explicit Dents( real depth, const A& a, const B& b, const C& c, const D& d );

   template< typename A, typename B, typename C, typename D, typename E >
   explicit Dents( real depth, const A& a, const B& b, const C& c, const D& d, const E& e );

   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~Dents();
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
/*!\brief Creating a POV-Ray dents normal with one additional modifier/transformation.
 *
 * \param depth The depth of the dents effect \f$ [0..1] \f$.
 * \param a The additional modifier/transformation.
 *
 * This constructor creates a POV-Ray dents normal using one additional modifier or
 * transformation. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the dents has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A >  // Type of the modifier/transformation
Dents::Dents( real depth, const A& a )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   dents " << depth << "\n";
   a.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray dents normal with two additional modifiers/transformations.
 *
 * \param depth The depth of the dents effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 *
 * This constructor creates a POV-Ray dents normal using two additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the dents has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B >  // Type of the second modifier/transformation
Dents::Dents( real depth, const A& a, const B& b )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   dents " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray dents normal with three additional modifiers/transformations.
 *
 * \param depth The depth of the dents effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 *
 * This constructor creates a POV-Ray dents normal using three additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the dents has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C >  // Type of the third modifier/transformation
Dents::Dents( real depth, const A& a, const B& b, const C& c )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   dents " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray dents normal with four additional modifiers/transformations.
 *
 * \param depth The depth of the dents effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray dents normal using four additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the dents has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D >  // Type of the fourth modifier/transformation
Dents::Dents( real depth, const A& a, const B& b, const C& c, const D& d )
   : Normal()  // Initialization of the base class
{
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( A, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( B, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( C, ValidTypes );
   pe_CONSTRAINT_SOFT_TYPE_RESTRICTION( D, ValidTypes );

   if( depth < real(0) || depth > real(1) )
      throw std::invalid_argument( "Invalid depth value" );

   std::ostringstream oss;
   oss << "   dents " << depth << "\n";
   a.print( oss, "   ", true );
   b.print( oss, "   ", true );
   c.print( oss, "   ", true );
   d.print( oss, "   ", true );

   oss.str().swap( normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a POV-Ray dents normal with five additional modifiers/transformations.
 *
 * \param depth The depth of the dents effect \f$ [0..1] \f$.
 * \param a The first additional modifier/transformation.
 * \param b The second additional modifier/transformation.
 * \param c The third additional modifier/transformation.
 * \param d The fourth additional modifier/transformation.
 * \param e The fifth additional modifier/transformation.
 *
 * This constructor creates a POV-Ray dents normal using five additional modifiers or
 * transformations. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!\n
 * The depth parameter for the dents has to be in the range from 0 to 1, inclusive.
 * Otherwise, a \a std::invalid_argument exception is thrown.
 */
template< typename A    // Type of the first modifier/transformation
        , typename B    // Type of the second modifier/transformation
        , typename C    // Type of the third modifier/transformation
        , typename D    // Type of the fourth modifier/transformation
        , typename E >  // Type of the fifth modifier/transformation
Dents::Dents( real depth, const A& a, const B& b, const C& c, const D& d, const E& e )
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
   oss << "   dents " << depth << "\n";
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
/*!\brief Destructor for the Dents class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline Dents::~Dents()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Normal, Dents );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a modifier or transformation to the POV-Ray dents normal.
 *
 * \param a The new modifier/transformation.
 * \return void
 *
 * This function adds a new modifier or transformation to the dents normal. Valid modifiers are
 *  - Turbulence
 *  - Octaves
 *  - Omega
 *  - Lambda
 *
 * Additionally, the following transformations are valid parameters:
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >  // Type of the modifier/transformation
void Dents::add( const A& a )
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
