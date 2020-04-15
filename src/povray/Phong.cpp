//=================================================================================================
/*!
 *  \file src/povray/Phong.cpp
 *  \brief Finish component for phong highlighting
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
#include <pe/povray/Phong.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a phong highlighting finish component.
 *
 * \param phong The strength of the phong highlight.
 * \exception std::invalid_argument Invalid phong value.
 *
 * Adding a phong highlight to the finish of a rigid body creates a highlight on the body that
 * is the color of the light source. The phong value specifies the saturation of the highlight
 * and has to be in the range \f$ [0..1] \f$. Otherwise a \a std::invalid_argument exception
 * is thrown. The first image shows a sphere without a phong highlight, the second image
 * demonstrates a phong highlight of strength 0.9.
 *
 * \image html phong.png
 * \image latex phong.eps "Examples for the phong highlight" width=400pt
 *
 * If no phong highlight is specified for a rigid body, the default POV-Ray value of 0 is used
 * (which completely turns off phong highlighting). The size of the phong highlight will be
 * set to the POV-Ray default of 40. Alternatively, the size of the phong highlight can be
 * individually specified by the PhongSize finish component.
 */
Phong::Phong( real phong )
   : FinishItem()     // Initialization of the base class
   , phong_( phong )  // The phong highlight value
{
   if( phong < real(0) || phong > real(1) )
      throw std::invalid_argument( "Invalid phong value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray phong finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Phong::print( std::ostream& os, bool newline ) const
{
   os << "phong " << phong_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray phong finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the phong output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Phong::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "phong " << phong_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Phong class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param phong Reference to a phong object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Phong& phong )
{
   phong.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
