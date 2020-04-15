//=================================================================================================
/*!
 *  \file src/povray/Octaves.cpp
 *  \brief POV-Ray octaves modifier
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
#include <pe/povray/Octaves.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Octaves modifier.
 *
 * \param octaves The octaves value \f$ [1..10] \f$.
 * \exception std::invalid_argument Invalid octaves value.
 *
 * This constructor creates a new octaves modifier that can be used to specify the number of
 * steps during the construction of a turbulence effect. The octaves value may be any integral
 * value between 1 and 10. Any other value will trigger a \a std::invalid_argument exception.
 */
Octaves::Octaves( unsigned int octaves )
   : Modifier()           // Initialization of the base class
   , octaves_( octaves )  // The octaves value
{
   if( octaves == 0 || octaves > 10 )
      throw std::invalid_argument( "Invalid octaves value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray octaves modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Octaves::print( std::ostream& os, bool newline ) const
{
   os << "octaves " << octaves_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray octaves modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the octaves output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Octaves::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "octaves " << octaves_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Octaves class.
 * \ingroup povray_modifier
 *
 * \param os Reference to the output stream.
 * \param octaves Reference to a octaves modifier object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Octaves& octaves )
{
   octaves.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
