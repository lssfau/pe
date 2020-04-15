//=================================================================================================
/*!
 *  \file src/povray/Ambient.cpp
 *  \brief Finish component for ambient lighting
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
#include <pe/povray/Ambient.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for an ambient lighting finish component.
 *
 * \param ambient The ambient lighting value \f$ [0..1] \f$.
 * \exception std::invalid_argument Invalid ambient value.
 *
 * The ambient lighting value specifies the luminance of the body itself. The value has to be
 * in the range \f$ [0..1] \f$. Otherwise a \a std::invalid_argument exception is thrown. If the
 * ambient lighting is set to 0, the rigid body will appear black if it is not directly lighted
 * by a light source. The following shows how ambient lighting works: the first image uses the
 * POV-Ray default of 0.1, the second was rendered with an ambient value of 0.6.
 *
 * \image html ambient.png
 * \image latex ambient.eps "Examples for ambient lighting" width=400pt
 *
 * If the ambient lighting value is not specified, the default POV-Ray value of 0.1 is used.
 */
Ambient::Ambient( real ambient )
   : FinishItem()         // Initialization of the base class
   , ambient_( ambient )  // The ambient value
{
   if( ambient < real(0) || ambient > real(1) )
      throw std::invalid_argument( "Invalid ambient value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray ambient finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Ambient::print( std::ostream& os, bool newline ) const
{
   os << "ambient " << ambient_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray ambient finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the ambient output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Ambient::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "ambient " << ambient_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Ambient class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param ambient Reference to a ambient object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Ambient& ambient )
{
   ambient.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
