//=================================================================================================
/*!
 *  \file pe/system/ProfilingMode.h
 *  \brief System settings for the profiling mode
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

#ifndef _PE_SYSTEM_PROFILINGMODE_H_
#define _PE_SYSTEM_PROFILINGMODE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/Logging.h>
#include <pe/util/logging/LogLevel.h>
#include <pe/util/StaticAssert.h>


namespace pe {

//=================================================================================================
//
//  PROFILING CONFIGURATION
//
//=================================================================================================

#include <pe/config/ProfilingMode.h>




//=================================================================================================
//
//  COMPILE TIME CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
namespace {

pe_STATIC_ASSERT( logging::loglevel >= logging::info );

}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
