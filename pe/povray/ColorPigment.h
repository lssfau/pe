//=================================================================================================
/*!
 *  \file pe/povray/ColorPigment.h
 *  \brief Implementation of a POV-Ray color pigment
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

#ifndef _PE_POVRAY_COLORPIGMENT_H_
#define _PE_POVRAY_COLORPIGMENT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <string>
#include <pe/povray/Color.h>
#include <pe/povray/Pigment.h>
#include <pe/povray/Transformation.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/TypeRestriction.h>
#include <pe/util/TypeList.h>


namespace pe {

namespace povray {

//*************************************************************************************************
/*!\brief A POV-Ray color pigment.
 * \ingroup povray_pigment
 *
 * The ColorPigment class represents a single-colored POV-Ray pigment. Single-colored pigments
 * are the simplest possible pigments: they consist of a single, fixed color for the entire
 * body. The following images give an idea of what single-colored pigments can look like:
 *
 * \image html singlecolor.png
 * \image latex singlecolor.eps "Examples for single-colored pigments" width=600pt
 *
 * Single-colored pigments are created via one of the ColorPigment constructors:
 * -# ColorPigment::ColorPigment( real red, real green, real blue, real alpha );
 * -# ColorPigment::ColorPigment( const pe::povray::Color& color );
 * -# ColorPigment::ColorPigment( const std::string& color );
 *
 * Examples:

   \code
   using namespace pe::pov;
   Pigment yellow = ColorPigment( 1.0, 1.0, 0.0, 0.0 );            (1)
   Pigment red    = ColorPigment( Color( "color rgb <1,0,0>" ) );  (2)
   Pigment blue   = ColorPigment( "color rgb <0,0,1>" );           (3)
   \endcode

 * \b Note: In case of spheres, a single-colored pigment is not able to show the rotation of
 * the spheres.
 */
class PE_PUBLIC ColorPigment : public Pigment
{
private:
   //**Type definitions****************************************************************************
   typedef pe_TYPELIST_1( Transformation )  ValidTypes;  //!< Valid modifiers/transformations.
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit ColorPigment( real red, real green, real blue, real alpha=0 );
   explicit ColorPigment( const Color& color );
   explicit ColorPigment( const std::string& color );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~ColorPigment();
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
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the ColorPigment class.
 *
 * This destructor is explicitly defined to provide a compile time EDO check.
 */
inline ColorPigment::~ColorPigment()
{
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE( Pigment, ColorPigment );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a transformation to the POV-Ray color pigment.
 *
 * \param a The new transformation.
 * \return void
 *
 * This function adds a new transformation to the color pigment. Valid transformations are
 *  - Scale
 *  - Rotation
 *  - Translation
 *
 * The attempt to use any other type results in a compile time error!
 */
template< typename A >
void ColorPigment::add( const A& a )
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
