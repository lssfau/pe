//=================================================================================================
/*!
 *  \file src/povray/Turbulence.cpp
 *  \brief POV-Ray turbulence modifier
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
#include <pe/povray/Turbulence.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Turbulence modifier.
 *
 * \param turbulence The turbulence value \f$ [0..1] \f$.
 * \exception std::invalid_argument Invalid turbulence value.
 *
 * This constructor creates a POV-Ray turbulence modifier that can be used to distorted specific
 * patterns (as for instance color maps or ripples). The turbulence value has to be in the range
 * 0 to 1 inclusive. Any other value is invalid and will result in a \a std::invalid_argument
 * exception.
 */
Turbulence::Turbulence( real turbulence )
   : Modifier()                 // Initialization of the base class
   , turbulence_( turbulence )  // The turbulence value
{
   if( turbulence < real(0) || turbulence > real(1) )
      throw std::invalid_argument( "Invalid turbulence value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray turbulence modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Turbulence::print( std::ostream& os, bool newline ) const
{
   os << "turbulence " << turbulence_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray turbulence modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the turbulence output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Turbulence::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "turbulence " << turbulence_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Turbulence class.
 * \ingroup povray_modifier
 *
 * \param os Reference to the output stream.
 * \param turbulence Reference to a turbulence modifier object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Turbulence& turbulence )
{
   turbulence.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
