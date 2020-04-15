//=================================================================================================
/*!
 *  \file src/povray/PointLight.cpp
 *  \brief Implementation of point light sources for the POV-Ray visualization
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

#include <pe/povray/PointLight.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a point light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 *
 * This constructor creates a point light at the global position \a gpos. This light source emits
 * light of the specified color uniformly in all directions.
 */
PointLight::PointLight( const Vec3& gpos, const Color& color )
   : LightSource()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n";

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
