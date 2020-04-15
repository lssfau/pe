//=================================================================================================
/*!
 *  \file src/povray/PointAt.cpp
 *  \brief Focus point light source modifier
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
#include <pe/povray/PointAt.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for a focus point light source modifier.
 *
 * \param x The x-component of the focus point of the light source.
 * \param y The y-component of the focus point of the light source.
 * \param z The z-component of the focus point of the light source.
 *
 * This constructor creates a PointAt light modifier that can be used for parallel light sources
 * and spotlights to direct the emitted light.
 */
PointAt::PointAt( real x, real y, real z )
   : LightItem()        // Initialization of the base class
   , point_( x, y, z )  // The light source focus point
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a focus point light source modifier.
 *
 * \param point The focus point of the light source.
 *
 * This constructor creates a PointAt light modifier that can be used for parallel light sources
 * and spotlights to direct the emitted light.
 */
PointAt::PointAt( const Vec3& point )
   : LightItem()      // Initialization of the base class
   , point_( point )  // The light source focus point
{}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the POV-Ray focus point light source modifier.
 *
 * \param os Reference to the output stream.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void PointAt::print( std::ostream& os, bool newline ) const
{
   os << "point_at <" << point_[0] << "," << point_[2] << "," << point_[1] << ">";
   if( newline ) os << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the POV-Ray focus point light source modifier.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation of the focus point output.
 * \param newline \a true if a new-line is appended to the end of the output, \a false if not.
 * \return void
 */
void PointAt::print( std::ostream& os, const char* tab, bool newline ) const
{
   os << tab << "point_at <" << point_[0] << "," << point_[2] << "," << point_[1] << ">";
   if( newline ) os << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the PointAt class.
 * \ingroup povray_lightsource
 *
 * \param os Reference to the output stream.
 * \param pointAt Reference to a PointAt object.
 * \return The output stream.
 *
 * The output operator uses neither an indentation at the beginning nor a new-line character
 * at the end of the output.
 */
std::ostream& operator<<( std::ostream& os, const PointAt& pointAt )
{
   pointAt.print( os, false );
   return os;
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
