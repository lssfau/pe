//=================================================================================================
/*!
 *  \file src/povray/Tightness.cpp
 *  \brief Tightness light source modifier
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

#include <iostream>
#include <stdexcept>
#include <pe/povray/Tightness.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a tightness light source modifier.
 *
 * \param tightness The tightness value \f$ [0..100] \f$.
 * \exception std::invalid_argument Invalid tightness value.
 *
 * This constructor creates a tightness light modifier. It can be used to specify an additional
 * exponential softening on the edges of the light cone. The tightness value must be in the
 * range \f$ [0..100] \f$. Otherwise a \a std::invalid_argument exception is thrown.
 */
Tightness::Tightness( real tightness )
   : LightItem()              // Initialization of the base class
   , tightness_( tightness )  // The tightness value
{
   if( tightness < real(0) || tightness > real(100)  )
      throw std::invalid_argument( "Invalid tightness value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray tightness light source modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Tightness::print( std::ostream& os, bool newline ) const
{
   os << "tightness " << tightness_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray tightness light source modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the tightness output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Tightness::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "tightness " << tightness_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Tightness class.
 * \ingroup povray_lightsource
 *
 * \param os Reference to the output stream.
 * \param tightness Reference to a Tightness object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Tightness& tightness )
{
   tightness.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
