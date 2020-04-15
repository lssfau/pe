//=================================================================================================
/*!
 *  \file pe/config/ProfilingMode.h
 *  \brief Configuration file for the profiling mode
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
/*!\brief Profiling mode switch.
 * \ingroup config
 *
 * This value toggles the profiling mode of the physics engine. If set to \a true, profiling
 * data will be gathered and written to the log file(s). Note that in order to be able to
 * write the data to the log file(s), the pe::loglevel must be at least set to \a info. In
 * case the profiling mode is activated and the current logging level is set too low, a compile
 * time error is created. Also note that collecting profiling data might induce a noticeable
 * runtime overhead. Therefore the profiling mode is switched off per default, which completely
 * removes all runtime overhead.
 *
 * Possible settings for the profiling mode:
 *  - Deactivated: \a false (default)
 *  - Activated  : \a true
 */
const bool profilingMode = false;
//*************************************************************************************************
