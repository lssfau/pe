//=================================================================================================
/*!
 *  \file src/povray/Specular.cpp
 *  \brief Finish component for specular highlighting
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

#include <sstream>
#include <stdexcept>
#include <pe/povray/Specular.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a specular highlighting finish component.
 *
 * \param specular The strength of the specular highlight \f$ [0..1] \f$.
 * \exception std::invalid_argument Invalid specular value.
 *
 * Adding a specular highlight to the finish of a rigid body creates a highlight on the body
 * that is the color of the light source. The specular value specifies the saturation of the
 * highlight and has to be in the range \f$ [0..1] \f$. Otherwise a \a std::invalid_argument
 * exception is thrown. The first image shows a sphere without a specular highlight, the second
 * image demonstrates a phong highlight of strength 0.6.
 *
 * \image html specular.png
 * \image latex specular.eps "Examples for the specular highlight" width=400pt
 *
 * If no specular highlight is specified for a rigid body, the default POV-Ray value of 0 is
 * used (which completely turns off specular highlighting). The size of the specular highlight
 * will be set to the POV-Ray default of 0.05. Alternatively, the size of the specular highlight
 * can be individually specified by the Roughness finish component.
 */
Specular::Specular( real specular )
   : FinishItem()           // Initialization of the base class
   , specular_( specular )  // The value of the specular highlight
{
   if( specular < real(0) || specular > real(1) )
      throw std::invalid_argument( "Invalid specular value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray specular finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Specular::print( std::ostream& os, bool newline ) const
{
   os << "specular " << specular_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray specular finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the specular output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Specular::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "specular " << specular_;
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Specular class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param specular Reference to a specular object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Specular& specular )
{
   specular.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
