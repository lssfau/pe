//=================================================================================================
/*!
 *  \file src/povray/Reflection.cpp
 *  \brief Finish component for reflections
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
#include <pe/povray/Reflection.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a reflection finish component.
 *
 * \param reflection The reflection value.
 * \exception std::invalid_argument Invalid reflection value.
 *
 * The reflection finish gives a rigid body a mirrored or partially mirrored surface. This body
 * will then reflect other bodies in the scene. The value has to be in the range \f$ [0..1] \f$.
 * Otherwise a \a std::invalid_argument exception is thrown. A value of 0 turns the reflection
 * off, a value of 1 gives the body an almost perfectly mirrored surface. The images illustrate
 * the reflection effect: the first image has no reflection, whereas the second one uses a
 * reflection of 0.3.
 *
 * \image html reflection.png
 * \image latex reflection.eps "Examples for reflection" width=400pt
 *
 * If the reflection value is not specified, the default POV-Ray value of 0 is used.
 */
Reflection::Reflection( real reflection )
   : FinishItem()               // Initialization of the base class
   , reflection_( reflection )  // Value of the reflection
{
   if( reflection < real(0) || reflection > real(1) )
      throw std::invalid_argument( "Invalid reflection value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray reflection finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Reflection::print( std::ostream& os, bool newline ) const
{
   os << "reflection " << reflection_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray reflection finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the reflection output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Reflection::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "reflection " << reflection_;
if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Reflection class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param reflection Reference to a reflection object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Reflection& reflection )
{
   reflection.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
