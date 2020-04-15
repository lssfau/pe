//=================================================================================================
/*!
 *  \file src/povray/ParallelLight.cpp
 *  \brief Implementation of parallel light sources for the POV-Ray visualization
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
#include <pe/povray/ParallelLight.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating a parallel light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 *
 * This constructor creates a parallel light source at the global position \a gpos. The light
 * source will emit light of the specified color from a plane determined by a perpendicular
 * defined by the global light position and the focus point of the light source. Per default,
 * this focus point is set to (0,0,0). However, via the PointAt light modifier, the focus point
 * can be set individually. If the \a shadowless parameter is set to \a true, the light source
 * will not cast shadows.
 *
 * \b Note: \n
 * -# Any parts of a rigid body "above" the light plane still get illuminated according to the
 *    light direction, but they will not cast or receive shadows.
 * -# The fade distance and fade power parameters use the global light position to determine the
 *    distance for light attenuation, so the attenuation still looks like that of a point source.
 */
ParallelLight::ParallelLight( const Vec3& gpos, const Color& color )
   : LightSource()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   parallel\n";

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
