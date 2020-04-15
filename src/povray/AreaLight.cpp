//=================================================================================================
/*!
 *  \file src/povray/AreaLight.cpp
 *  \brief Implementation of area light sources for the POV-Ray visualization
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

#include <pe/povray/AreaLight.h>


namespace pe {

namespace povray {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Creating an area light source.
 *
 * \param gpos The global position of the light source.
 * \param color The color of the light source.
 * \param u First axis of the area light source.
 * \param v Second axis of the area light source.
 * \param m Number of point light sources along the first axis.
 * \param n Number of point light sources along the second axis.
 *
 * This constructor creates a POV-Ray area light source. The center of the area light source is
 * placed at \a gpos. The \a m times \a n point light sources are placed on the plane spanned
 * by the two specified axes \a u and \a v.
 */
AreaLight::AreaLight( const Vec3& gpos, const Color& color, const Vec3& u, const Vec3& v,
                      size_t m, size_t n )
   : LightSource()  // Initialization of the base class
{
   std::ostringstream oss;
   oss << "   <" << gpos[0] << "," << gpos[2] << "," << gpos[1] << ">\n"
       << "   " << color << "\n"
       << "   area_light\n"
       << "   " << u << ", " << v << ", " << m << ", " << n << "\n";

   oss.str().swap( lightsource_ );
}
//*************************************************************************************************

} // namespace povray

} // namespace pe
