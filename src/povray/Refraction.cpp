//=================================================================================================
/*!
 *  \file src/povray/Refraction.cpp
 *  \brief Finish component for refractions
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
#include <pe/povray/Refraction.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a refraction finish component.
 *
 * \param refraction The refraction value.
 * \exception std::invalid_argument Invalid refraction value.
 *
 * Refraction only has meaning on rigid bodies that have at least a litte bit of transparency.
 * Refraction is the bending of light rays as they pass into a more dense or less dense medium.
 * As light does not go through opaque things, they don't refract. Without refraction, transparent
 * bodies look like colored air. The refraction value has to be in the range \f$ [1..\infty) \f$.
 * A value of 1.0 is the POV-Ray default and will not change the refraction of a body. Examples
 * for some physical refraction values are 1.000292 for air, 1.33 for water or 1.5 for glass.
 *
 * \image html refraction.png
 * \image latex refraction.eps "Examples for refraction" width=400pt
 *
 * If no refraction is specified for a rigid body, refraction is turned off by default.
 */
Refraction::Refraction( real refraction )
   : FinishItem()               // Initialization of the base class
   , refraction_( refraction )  // The refraction value
{
   if( refraction < real(1) )
      throw std::invalid_argument( "Invalid refraction value" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray refraction finish item.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Refraction::print( std::ostream& os, bool newline ) const
{
   os << "refraction 1\n"
         "ior " << refraction_;
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray refraction finish item.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the refraction output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void Refraction::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "refraction 1\n"
      << tab << "ior " << refraction_;
if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the Refraction class.
 * \ingroup povray_finish
 *
 * \param os Reference to the output stream.
 * \param refraction Reference to a refraction object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const Refraction& refraction )
{
   refraction.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
