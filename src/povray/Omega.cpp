//=================================================================================================
/*!
 *  \file src/povray/Omega.cpp
 *  \brief POV-Ray omega modifier
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
#include <sstream>
#include <stdexcept>
#include <pe/povray/Omega.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Omega modifier.
 *
 * \param omega The omega value \f$ (0..1) \f$.
 * \exception std::invalid_argument Invalid omega value.
 *
 * This constructor creates a new omega modifier that can be used to adjust the steps sizes
 * during the calculation of a Turbulence effect. The omega value may be any value in the
 * range from 0 to 1, exclusive. Any other value will trigger a \a std::invalid_argument
 * exception.
 */
Omega::Omega( real omega )
   : Modifier()       // Initialization of the base class
   , omega_( omega )  // The omega value
{
   if( omega <= real(0) || omega >= real(1) )
      throw std::invalid_argument( "Invalid omega value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray omega modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Omega::print( std::ostream& os, bool newline ) const
{
   os << "omega " << omega_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray omega modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the omega output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Omega::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "omega " << omega_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Omega class.
 * \ingroup povray_modifier
 *
 * \param os Reference to the output stream.
 * \param omega Reference to a omega modifier object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Omega& omega )
{
   omega.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
