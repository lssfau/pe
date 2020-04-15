//=================================================================================================
/*!
 *  \file src/povray/FadeDistance.cpp
 *  \brief Fade distance light source modifier
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
#include <pe/povray/FadeDistance.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a fade distance light source modifier.
 *
 * \param distance The fade distance value \f$ (0..\infty) \f$.
 * \exception std::invalid_argument Invalid fade distance value.
 *
 * The fade distance value specifies the distance at which the full light intensity of a light
 * source arrives, i.e. the intensity which was given by the color specification of the light
 * source. The \a distance value has to be in the range \f$ (0..\infty) \f$. Otherwise a
 * \a std::invalid_argument exception is thrown.
 */
FadeDistance::FadeDistance( real distance )
   : LightItem()            // Initialization of the base class
   , distance_( distance )  // The fade distance value
{
   if( distance <= real(0) )
      throw std::invalid_argument( "Invalid fade distance value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray fade distance light source modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void FadeDistance::print( std::ostream& os, bool newline ) const
{
   os << "fade_distance " << distance_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray fade distance light source modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the fade distance output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void FadeDistance::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "fade_distance " << distance_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the FadeDistance class.
 * \ingroup povray_lightsource
 *
 * \param os Reference to the output stream.
 * \param fadedistance Reference to a FadeDistance object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const FadeDistance& fadedistance )
{
   fadedistance.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
