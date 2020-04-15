//=================================================================================================
/*!
 *  \file src/povray/Translation.cpp
 *  \brief POV-Ray transformation for translations
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
#include <pe/povray/Translation.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Translation transformation.
 *
 * \param x The x-component of the translation.
 * \param y The y-component of the translation.
 * \param z The z-component of the translation.
 *
 * This constructor creates a POV-Ray translation transformation that can be used to shift
 * textures, pigment or normals to a different location. For translations, the order of
 * the arguments doesn't matter (in comparison to a Rotation).
 */
Translation::Translation( real x, real y, real z )
   : Transformation()         // Initialization of the base class
   , translation_( x, z, y )  // The translation vector
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a Translation transformation.
 *
 * \param dx The translation vector.
 *
 * This constructor creates a POV-Ray translation transformation that can be used to shift
 * textures, pigment or normals to a different location. For translations, the order of
 * the arguments doesn't matter (in comparison to a Rotation).
 */
Translation::Translation( const Vec3& dx )
   : Transformation()                     // Initialization of the base class
   , translation_( dx[0], dx[2], dx[1] )  // The translation vector
{}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray translation transformation.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Translation::print( std::ostream& os, bool newline ) const
{
   os << "translate " << translation_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray translation transformation.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the translation output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Translation::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "translate " << translation_;
if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Translation class.
 * \ingroup povray_transformation
 *
 * \param os Reference to the output stream.
 * \param translation Reference to a translation transformation object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Translation& translation )
{
   translation.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
