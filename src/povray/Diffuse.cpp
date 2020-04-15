//=================================================================================================
/*!
 *  \file src/povray/Diffuse.cpp
 *  \brief Finish component for diffuse lighting
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
#include <pe/povray/Diffuse.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a diffuse lighting finish component.
 *
 * \param diffuse The diffuse lighting value.
 * \exception std::invalid_argument Invalid diffuse value.
 *
 * The diffuse lighting specifies how much light is diffused at the surface of the rigid body.
 * The value has to be in the range \f$ [0..1] \f$. Otherwise a \a std::invalid_argument exception
 * is thrown. The images give an impression of three diffuse lighting values: the first image
 * uses a diffuse lighting of 0.3, the second was rendered with the POV-Ray default of 0.6 and
 * the third has a diffuse value of 0.9.
 *
 * \image html diffuse.png
 * \image latex diffuse.eps "Examples for diffuse lighting" width=600pt
 *
 * If the diffuse lighting value is not specified, the default POV-Ray value of 0.6 is used.
 */
Diffuse::Diffuse( real diffuse )
   : FinishItem()         // Initialization of the base class
   , diffuse_( diffuse )  // The diffuse value
{
   if( diffuse < real(0) || diffuse > real(1) )
      throw std::invalid_argument( "Invalid diffuse value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray diffuse finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Diffuse::print( std::ostream& os, bool newline ) const
{
   os << "diffuse " << diffuse_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray diffuse finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the diffuse output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Diffuse::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "diffuse " << diffuse_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Diffuse class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param diffuse Reference to a diffuse object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Diffuse& diffuse )
{
   diffuse.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
