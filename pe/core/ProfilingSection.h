//=================================================================================================
/*!
 *  \file pe/core/ProfilingSection.h
 *  \brief Header file for the profiling section
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

#ifndef _PE_CORE_PROFILINGSECTION_H_
#define _PE_CORE_PROFILINGSECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/ProfilingMode.h>


//=================================================================================================
//
//  GAMES SECTION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Profiling section for time measurements and code analysis.
 * \ingroup core
 * The \a pe_PROFILING_SECTION macro provides a specific runtime environment that is activated
 * in case the profiling functionality of the \b pe is switched on (i.e., pe::profiling is set
 * to \a true). The following code demonstrates the use of this macro for the time measurement
 * of a certain code section:

   \code
   pe::timing::WcTimer timer;

   // Starting the timer
   pe_PROFILING_SECTION {
      timer.start();
   }

   // Code section to be analysed
   ...

   // Ending the timer
   pe_PROFILING_SECTION {
      timer.end();
   }
   \endcode
 */
#define pe_PROFILING_SECTION if( profilingMode )
//*************************************************************************************************

#endif
