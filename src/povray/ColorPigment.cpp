//=================================================================================================
/*!
 *  \file src/povray/ColorPigment.cpp
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/povray/ColorPigment.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a single-colored pigment.
 *
 * \param red The red channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param green The green channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param blue The blue channel of the color. Has to be in the range \f$ [0..1] \f$.
 * \param alpha The transparency of the color. Has to be in the range \f$ [0..1] \f$.
 * \exception std::invalid_argument Invalid color value.
 *
 * This constructor creates a single-colored/monochrome pigment. The following images give
 * an impression of single-colored pigments:
 *
 * \image html singlecolor.png
 * \image latex singlecolor.eps "Examples for single-colored pigments" width=600pt
 *
 * The values for each of the three color channels and the transparency have to be in the range
 * \f$ [0..1] \f$. Otherwise, a \a std::invalid_argument exception is thrown.
 */
ColorPigment::ColorPigment( real red, real green, real blue, real alpha )
   : Pigment()  // Initialization of the base class
{
   Color color( red, green, blue, alpha );

   std::ostringstream oss;
   oss << pigment_ << "   " << color << "\n";

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a single-colored pigment.
 *
 * \param color The color of the pigment.
 *
 * This constructor creates a single-colored/monochrome pigment. The following images give
 * an impression of single-colored pigments:
 *
 * \image html singlecolor.png
 * \image latex singlecolor.eps "Examples for single-colored pigments" width=600pt
 */
ColorPigment::ColorPigment( const Color& color )
   : Pigment()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << pigment_ << "   " << color << "\n";

   oss.str().swap( pigment_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Creating a single-colored pigment.
 *
 * \param color The input string containing the color parameters.
 * \exception std::invalid_argument Invalid color input string.
 *
 * This constructor creates a single-colored/monochrome pigment. The following images give
 * an impression of single-colored pigments:
 *
 * \image html singlecolor.png
 * \image latex singlecolor.eps "Examples for single-colored pigments" width=600pt
 *
 * For details about the format of the color parameter string, see the details of the Color
 * class description.
 */
ColorPigment::ColorPigment( const std::string& color )
   : Pigment()  // Initialization of the base class
{
   Color c( color );

   std::ostringstream oss;
   oss << pigment_ << "   " << c << "\n";

   oss.str().swap( pigment_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
