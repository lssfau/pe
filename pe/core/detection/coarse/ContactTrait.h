//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/ContactTrait.h
 *  \brief Contact customization class for the coarse collision detection
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

#ifndef _PE_CORE_DETECTION_COARSE_CONTACTTRAIT_H_
#define _PE_CORE_DETECTION_COARSE_CONTACTTRAIT_H_


namespace pe {

namespace detection {

namespace coarse {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact customization class for the coarse collision detection.
 * \ingroup coarse_collision_detection
 *
 * The coarse collision detection ContactTrait class template is used to adapt the Contact class
 * to the used coarse collision detection algorithm. Depending on the used algorithm, a contact
 * requires additional data or functionality to efficiently support the coarse collision detection
 * process.\n
 * In order to adapt the Contact class to a particular algorithm, the base template needs to be
 * specialized.
 */
template< typename C >  // Type of the configuration
class ContactTrait
{};
//*************************************************************************************************

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
