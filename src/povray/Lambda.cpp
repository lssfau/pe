//=================================================================================================
/*!
 *  \file src/povray/Lambda.cpp
 *  \brief POV-Ray lambda modifier
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
#include <pe/povray/Lambda.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a Lambda modifier.
 *
 * \param lambda The lambda value \f$ (1..\infty) \f$.
 * \exception std::invalid_argument Invalid lambda value.
 *
 * This constructor creates a new lambda modifier that can be used to adjust the behavior
 * of the Turbulence modifier. The lambda value has to be larger than 1, otherwise a
 * \a std::invalid_argument exception is thrown.
 */
Lambda::Lambda( real lambda )
   : Modifier()         // Initialization of the base class
   , lambda_( lambda )  // The lambda value
{
   if( lambda <= real(1) )
      throw std::invalid_argument( "Invalid lambda value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray lambda modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Lambda::print( std::ostream& os, bool newline ) const
{
   os << "lambda " << lambda_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray lambda modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the lambda output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Lambda::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "lambda " << lambda_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Lambda class.
 * \ingroup povray_modifier
 *
 * \param os Reference to the output stream.
 * \param lambda Reference to a lambda modifier object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Lambda& lambda )
{
   lambda.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
