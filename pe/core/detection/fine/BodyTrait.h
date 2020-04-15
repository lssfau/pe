//=================================================================================================
/*!
 *  \file pe/core/detection/fine/BodyTrait.h
 *  \brief Rigid body customization class for the fine collision detection
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

#ifndef _PE_CORE_DETECTION_FINE_BODYTRAIT_H_
#define _PE_CORE_DETECTION_FINE_BODYTRAIT_H_


namespace pe {

namespace detection {

namespace fine {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body customization class for the fine collision detection.
 * \ingroup fine_collision_detection
 *
 * The fine detection module BodyTrait class template is used to adapt the RigidBody class
 * to the used fine collision detection algorithm. For instance, this class configures the
 * axis-aligned bounding box for rigid bodies depending on the requirements of the selected
 * fine collision detection algorithm.\n
 * In order to adapt the RigidBody class to a particular algorithm, the base template needs
 * to be specialized.
 */
template< typename C >  // Type of the configuration
class BodyTrait
{};
//*************************************************************************************************

} // namespace fine

} // namespace detection

} // namespace pe

#endif
