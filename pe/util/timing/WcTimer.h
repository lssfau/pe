//=================================================================================================
/*!
 *  \file pe/util/timing/WcTimer.h
 *  \brief Progress timer for wall clock time measurements
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

#ifndef _PE_UTIL_TIMING_WCTIMER_H_
#define _PE_UTIL_TIMING_WCTIMER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/timing/Timer.h>
#include <pe/util/timing/WcPolicy.h>


namespace pe {

namespace timing {

//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Progress timer for wall clock time measurements.
 * \ingroup timing
 *
 * The WcTimer combines the Timer class template with the WcPolicy timing policy. It measures
 * the amount of "wall clock" time elapsing for the processing of a programm or code fragment.
 * In contrast to the measurement of CPU time, the wall clock time also contains waiting times
 * such as input/output operations.
 */
typedef Timer<WcPolicy>  WcTimer;
//*************************************************************************************************

} // timing

} // pe

#endif
