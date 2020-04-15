//=================================================================================================
/*!
 *  \file src/povray/PhongSize.cpp
 *  \brief Modifier for the phong highlighting finish component
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
#include <pe/povray/PhongSize.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the phong size finish component.
 *
 * \param phongsize The size of the phong highlight.
 * \exception std::invalid_argument Invalid phong size.
 *
 * This constructor creates a PhongSize modifier that can be used to set the size of a phong
 * highlight. The phong size has to be a value in the range \f$ [0..\infty) \f$. Otherwise, a
 * \a std::invalid_argument exception is thrown. The following images illustrate the effect of
 * the phong size value: the first image has a phong size of 180, the second uses the POV-Ray
 * default of 40 and the third has a phong size of 4.
 *
 * \image html phongsize.png
 * \image latex phongsize.eps "Examples for the phong size" width=600pt
 *
 * In case a phong highlight is used and the phong size is not specified, the default POV-Ray
 * value of 40 is used.
 */
PhongSize::PhongSize( real phongsize )
   : FinishItem()             // Initialization of the base class
   , phongsize_( phongsize )  // The size of the phong highlight
{
   if( phongsize < real(0) )
      throw std::invalid_argument( "Invalid phong size" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray phong size finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.

 * \return void
 */
void PhongSize::print( std::ostream& os, bool newline ) const
{
   os << "phong_size " << phongsize_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray phong size finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the phong size output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void PhongSize::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "phong_size " << phongsize_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the PhongSize class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param phongsize Reference to a phong size object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const PhongSize& phongsize )
{
   phongsize.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
