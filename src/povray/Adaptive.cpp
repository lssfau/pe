//=================================================================================================
/*!
 *  \file src/povray/Adaptive.cpp
 *  \brief Adaptive light source modifier
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
#include <pe/povray/Adaptive.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for an adaptive light source modifier.
 *
 * \param adaptive The adaptive value \f$ [0..\infty) \f$.
 *
 * This constructor creates an adaptive light modifier that specifies the number of test rays
 * for the adaptive sampling process.
 */
Adaptive::Adaptive( unsigned int adaptive )
   : LightItem()            // Initialization of the base class
   , adaptive_( adaptive )  // The adaptive value
{}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray adaptive light source modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Adaptive::print( std::ostream& os, bool newline ) const
{
   os << "adaptive " << adaptive_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray adaptive light source modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the adaptive output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Adaptive::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "adaptive " << adaptive_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Adaptive class.
 * \ingroup povray_lightsource
 *
 * \param os Reference to the output stream.
 * \param adaptive Reference to a Adaptive object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Adaptive& adaptive )
{
   adaptive.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
