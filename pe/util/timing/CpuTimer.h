//=================================================================================================
/*!
 *  \file pe/util/timing/CpuTimer.h
 *  \brief Progress timer for CPU time measurements
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

#ifndef _PE_UTIL_TIMING_CPUTIMER_H_
#define _PE_UTIL_TIMING_CPUTIMER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/timing/CpuPolicy.h>
#include <pe/util/timing/Timer.h>


namespace pe {

namespace timing {

//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Progress timer for CPU time measurements.
 * \ingroup timing
 *
 * The CpuTimer combines the Timer class template with the CpuPolicy timing policy. It measures
 * the amount of time the measured program or code fragment uses in processing central processing
 * unit (CPU) instructions.
 */
typedef Timer<CpuPolicy>  CpuTimer;
//*************************************************************************************************

} // timing

} // pe

#endif
