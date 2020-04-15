//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/JointTrait.h
 *  \brief Joint customization class for the coarse collision detection
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

#ifndef _PE_CORE_RESPONSE_JOINTTRAIT_H_
#define _PE_CORE_RESPONSE_JOINTTRAIT_H_


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Joint customization class for the collision response.
 * \ingroup collision_response
 *
 * The JointTrait class template is used to adapt the Joint class to the used collision
 * response algorithm. Depending on the used algorithm, a joint may require additional
 * data or functionality to efficiently support the collision response calculations.\n
 * In order to adapt the Joint class to a particular algorithm, the base template needs
 * to be specialized.
 */
template< typename C >  // Type of the configuration
class JointTrait
{};
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
