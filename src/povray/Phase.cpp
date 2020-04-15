//=================================================================================================
/*!
 *  \file src/povray/Phase.cpp
 *  \brief POV-Ray phase modifier
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
#include <pe/povray/Phase.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Phase modifier.
 *
 * \param phase The phase value \f$ [0..1] \f$.
 * \exception std::invalid_argument Invalid phase value.
 *
 * This constructor creates a new phase modifier that can be used to offset a color map or to
 * displace a normal. The phase value has to be in the range 0 to 1 inclusive, otherwise a
 * \a std::invalid_argument exception is thrown.
 */
Phase::Phase( real phase )
   : Modifier()       // Initialization of the base class
   , phase_( phase )  // The phase value
{
   if( phase < real(0) || phase > real(1) )
      throw std::invalid_argument( "Invalid phase value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray phase modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Phase::print( std::ostream& os, bool newline ) const
{
   os << "phase " << phase_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray phase modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the phase output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Phase::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "phase " << phase_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Phase class.
 * \ingroup povray_modifier
 *
 * \param os Reference to the output stream.
 * \param phase Reference to a phase modifier object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Phase& phase )
{
   phase.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
