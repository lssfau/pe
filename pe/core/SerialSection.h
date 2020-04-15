//=================================================================================================
/*!
 *  \file pe/core/SerialSection.h
 *  \brief Serial section for code exclusively executed in non-parallel simulations
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

#ifndef _PE_CORE_SERIALSECTION_H_
#define _PE_CORE_SERIALSECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/MPISettings.h>


//=================================================================================================
//
//  SERIAL SECTION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Serial section for code exclusively executed in non-parallel simulations.
 * \ingroup mpi
 *
 * This macro starts a serial section that contains code that is only executed in case the
 * pe is compiled without MPI support or the simulation consists of only a single MPI
 * process.
 */
#if HAVE_MPI
#  define pe_SERIAL_SECTION if( MPISettings::size() == 1 )
#else
#  define pe_SERIAL_SECTION if( true )
#endif
//*************************************************************************************************

#endif
