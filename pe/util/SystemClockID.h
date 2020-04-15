//=================================================================================================
/*!
 *  \file pe/util/SystemClockID.h
 *  \brief Implementation of a smart SystemClock handle
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

#ifndef _PE_UTIL_SYSTEMCLOCKID_H_
#define _PE_UTIL_SYSTEMCLOCKID_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/shared_ptr.hpp>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

class SystemClock;




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Handle for the system clock of the \b pe physics engine.
 * \ingroup util
 */
typedef boost::shared_ptr<SystemClock>  SystemClockID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for the system clock of the \b pe physics engine.
 * \ingroup util
 */
typedef boost::shared_ptr<const SystemClock>  ConstSystemClockID;
//*************************************************************************************************

} // namespace pe

#endif
