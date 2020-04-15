//=================================================================================================
/*!
 *  \file src/povray/Roughness.cpp
 *  \brief Modifier for the specular highlighting finish component
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
#include <pe/povray/Roughness.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the roughness finish component.
 *
 * \param roughness The roughness of the specular highlight
 * \exception std::invalid_argument Invalid roughness parameter.
 *
 * This constructor creates a Roughness modifier that can be used to modifiy the appearance of a
 * specular highlight. The roughness parameter has to be in the range \f$ (0..1] \f$. Otherwise,
 * a \a std::invalid_argument exception is thrown. The next images demonstrate the effect of the
 * roughness parameter: the first image has a roughness of 0.01, the second uses the default
 * POV-Ray value of 0.05 and the third has a roughness of 0.1.
 *
 * \image html roughness.png
 * \image latex roughness.eps "Examples for the roughness" width=600pt
 *
 * In case a specular highlight is used and the roughness is not specified, the default POV-Ray
 * value of 0.05 is used.
 */
Roughness::Roughness( real roughness )
   : FinishItem()             // Initialization of the base class
   , roughness_( roughness )  // The roughness of the specular highlight
{
   if( roughness <= real(0) || roughness > real(1) )
      throw std::invalid_argument( "Invalid roughness size" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray roughness finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Roughness::print( std::ostream& os, bool newline ) const
{
   os << "roughness " << roughness_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray roughness finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the roughness output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Roughness::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "roughness " << roughness_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Roughness class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param roughness Reference to a roughness object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Roughness& roughness )
{
   roughness.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
