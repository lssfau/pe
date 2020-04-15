//=================================================================================================
/*!
 *  \file src/povray/Shadowless.cpp
 *  \brief Shadowless light source modifier
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
#include <pe/povray/Shadowless.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the shadowless light source modifier..
 *
 * This constructor creates an shadowless light modifier that switches off all shadows and
 * highlights created from a particular light source.
 */
Shadowless::Shadowless()
   : LightItem()  // Initialization of the base class
{}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray shadowless light source modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Shadowless::print( std::ostream& os, bool newline ) const
{
   os << "shadowless";
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray shadowless light source modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the shadowless output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Shadowless::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "shadowless";
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
 * \param shadowless Reference to a Shadowless object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Shadowless& shadowless )
{
   shadowless.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
