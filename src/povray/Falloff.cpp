//=================================================================================================
/*!
 *  \file src/povray/Falloff.cpp
 *  \brief Falloff light source modifier
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
#include <pe/math/Constants.h>
#include <pe/povray/Falloff.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a falloff light source modifier.
 *
 * \param falloff The falloff value \f$ [0..\frac{\pi}{2}] \f$.
 * \exception std::invalid_argument Invalid falloff value.
 *
 * This constructor creates a falloff light modifier that specifies the overall size of the
 * light cone of a focused light source, as for instance a spotlight. The falloff has to be
 * specified in radian measure and must be in the range \f$ [0..\frac{\pi}{2}] \f$. In case
 * the value is not in this valid range, a \a std::invalid_argument exception is thrown.
 */
Falloff::Falloff( real falloff )
   : LightItem()                         // Initialization of the base class
   , falloff_( falloff*real(180)/M_PI )  // The falloff value
{
   if( falloff < real(0) || falloff > M_PI/real(2)  )
      throw std::invalid_argument( "Invalid falloff value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray falloff light source modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Falloff::print( std::ostream& os, bool newline ) const
{
   os << "falloff " << falloff_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray falloff light source modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the falloff output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Falloff::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "falloff " << falloff_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Falloff class.
 * \ingroup povray_lightsource
 *
 * \param os Reference to the output stream.
 * \param falloff Reference to a Falloff object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Falloff& falloff )
{
   falloff.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
