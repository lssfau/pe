//=================================================================================================
/*!
*  \file pe/core/MPITag.h
 *  \brief Header file for enumerating MPI tag numbers
 *
 *  Copyright (C) 2012 Tobias Preclik
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

#ifndef _PE_CORE_MPITAG_H_
#define _PE_CORE_MPITAG_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/Byte.h>


namespace pe {

//=================================================================================================
//
//  MPI TAGS
//
//=================================================================================================

//*************************************************************************************************
//! Associate a unique tag number to MPI messages.
enum MPITag {
   mpitagMPISystemCheckProcesses = 0,
   mpitagBodySimpleAsciiWriter,
   mpitagHCTSSynchronizePositionsAndVelocities,
   mpitagHCTSSynchronizePositionsAndVelocities2,
   mpitagHCTSSynchronizeVelocityCorrections,
   mpitagHCTSSynchronizeVelocities,
   mpitagDEMSynchronizeForces,
   mpitagDEMSynchronizePositionsAndVelocities,
   mpitagDEMSynchronizePositionsAndVelocities2,
   mpitagDEMObsoleteSynchronizeComplete,
   mpitagDEMObsoleteSynchronizeForces,
   mpitagDEMObsoleteSynchronizePositionsAndVelocities,
   mpitagFFDSynchronizeComplete,
   mpitagFFDSynchronize1,
   mpitagFFDSynchronize2,
   mpitagFFDSynchronize3,
   mpitagFFDSynchronize4,
   mpitagUser,
};
//*************************************************************************************************

} // namespace pe

#endif
