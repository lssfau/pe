//=================================================================================================
/*!
 *  \file src/povray/Frequency.cpp
 *  \brief POV-Ray frequency modifier
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
#include <pe/povray/Frequency.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Frequency modifier.
 *
 * \param frequency The frequency value.
 * \exception std::invalid_argument Invalid frequency value.
 *
 * This constructor creates a frequency modifier, that controls how often a specific pattern
 * is repeated over the range between 0.0 and 1.0. A value greater than 1 will compress the
 * pattern, a value less than 1 will stretch it. Negative values will reverse the pattern.
 * However, a frequency value of 0 is an invalid value and triggers a \a std::invalid_argument
 * exception.
 */
Frequency::Frequency( real frequency )
   : Modifier()               // Initialization of the base class
   , frequency_( frequency )  // The frequency value
{
   if( frequency == real(0) )
      throw std::invalid_argument( "Invalid frequency value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray frequency modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Frequency::print( std::ostream& os, bool newline ) const
{
   os << "frequency " << frequency_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray frequency modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the frequency output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Frequency::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "frequency " << frequency_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Frequency class.
 * \ingroup povray_modifier
 *
 * \param os Reference to the output stream.
 * \param frequency Reference to a frequency modifier object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Frequency& frequency )
{
   frequency.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
