//=================================================================================================
/*!
 *  \file src/povray/FadePower.cpp
 *  \brief Fade power light source modifier
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
#include <pe/povray/FadePower.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a fade power light source modifier.
 *
 * \param power The fade power value \f$ (0..\infty) \f$.
 * \exception std::invalid_argument Invalid fade power value.
 *
 * The fade power value determines the falloff rate of a POV-Ray light source, which describes
 * the actual attenuation of the light source. For example linear or quadratic falloff can be
 * used by setting fade_power to 1 or 2 respectively. The \a power value has to be in the range
 * \f$ (0..\infty) \f$. Otherwise a \a std::invalid_argument exception is thrown.
 */
FadePower::FadePower( real power )
   : LightItem()      // Initialization of the base class
   , power_( power )  // The fade power value
{
   if( power <= real(0) )
      throw std::invalid_argument( "Invalid fade power value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray fade power light source modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void FadePower::print( std::ostream& os, bool newline ) const
{
   os << "fade_power " << power_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray fade power light source modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the fade power output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void FadePower::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "fade_power " << power_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the FadePower class.
 * \ingroup povray_lightsource
 *
 * \param os Reference to the output stream.
 * \param fadepower Reference to a FadePower object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const FadePower& fadepower )
{
   fadepower.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
