//=================================================================================================
/*!
 *  \file src/povray/Scale.cpp
 *  \brief POV-Ray transformation for scaling operations
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
#include <pe/povray/Scale.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Scale transformation.
 *
 * \param scale The scaling value \f$ (0..\infty) \f$.
 * \exception std::invalid_argument Invalid scale value.
 *
 * This constructor creates a new scale transformation that can be used to either stretch or
 * squeeze a texture, pigment or normal. A scale value larger than 1 will stretch the texture,
 * pigment or normal, a value smaller than 1 will squeeze it. A negative value will reverse the
 * texture, pigment or normal. However, a scale value of 0 triggers a \a std::invalid_argument
 * exception.
 */
Scale::Scale( real scale )
   : Transformation()  // Initialization of the base class
   , scale_( scale )   // The scale value
{
   if( scale == real(0) )
      throw std::invalid_argument( "Invalid scale value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray scale transformation.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Scale::print( std::ostream& os, bool newline ) const
{
   os << "scale " << scale_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray scale transformation.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the scale output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Scale::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "scale " << scale_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Scale class.
 * \ingroup povray_transformation
 *
 * \param os Reference to the output stream.
 * \param scale Reference to a scale transformation object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Scale& scale )
{
   scale.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
