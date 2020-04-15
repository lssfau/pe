//=================================================================================================
/*!
 *  \file pe/core/MPISection.h
 *  \brief MPI section for code exclusively executed for MPI parallel simulations
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

#ifndef _PE_CORE_MPISECTION_H_
#define _PE_CORE_MPISECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/MPISettings.h>


//=================================================================================================
//
//  MPI SECTION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief MPI section for code exclusively executed for MPI parallel simulations.
 * \ingroup mpi
 *
 * This macro starts an MPI section that contains code that is only executed in case the
 * pe is compiled with MPI support and the simulation consists of more than a single
 * MPI process.
 */
#if HAVE_MPI
#  define pe_MPI_SECTION if( MPISettings::size() > 1 )
#else
#  define pe_MPI_SECTION if( false )
#endif
//*************************************************************************************************

#endif
