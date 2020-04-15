//=================================================================================================
/*!
 *  \file pe/core/RootSection.h
 *  \brief Exclusive section for the root process in a parallel environment
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

#ifndef _PE_CORE_ROOTSECTION_H_
#define _PE_CORE_ROOTSECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/ExclusiveSection.h>
#include <pe/core/MPISettings.h>


namespace pe {

//=================================================================================================
//
//  ROOT SECTION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Exclusive section for the root process in a parallel environment.
 * \ingroup mpi
 *
 * This macro is closely related to the pe::pe_EXCLUSIVE_SECTION macro.  It provides the option
 * to start a special exclusive section for the root process in a MPI parallel environment. The
 * following example demonstrates how a root section is used:

   \code
   int main( int argc, char** argv )
   {
      // Initialization of the MPI system
      // The MPI system must be initialized before any pe functionality is used. It is
      // recommended to make MPI_Init() the very first call of the main function.
      MPI_Init( &argc, &argv );

      // Parallel region
      // This code is executed by all processes of the parallel simulation.
      ...

      // Exclusive section
      // This section is only executed by the root process and skipped by all other processes.
      // This can for example be used to perform special setups of checks of the parallel
      // simulation environment.
      pe_ROOT_SECTION {
         ...
      }

      // Second parallel region
      // This code is again executed by all processes of the parallel simulation.
      ...

      // Finalizing the MPI system
      // The MPI system must be finalized after the last pe functionality has been used. It
      // is recommended to make MPI_Finalize() the very last call of the main function.
      MPI_Finalize();
   }
   \endcode

 * Using the pe::pe_ROOT_SECTION (or pe::pe_EXCLUSIVE_SECTION) macro has a similar effect as using
 * an if-condition to test the rank of the MPI process:

   \code
   if( rank == ... ) {
      // Exclusive code for a specific MPI process
   }
   \endcode

 * However, it is strongly recommended to either use the pe::pe_EXCLUSIVE_SECTION (for any
 * process) or the pe::pe_ROOT_SECTION (specifically for the root process) instead since only
 * in this case the \b pe is able to detect certain errors.\n
 * Note that starting an exclusive or root section for an invalid MPI rank (i.e. a negative rank
 * or a rank greater-or-equal than the actual total number of MPI processes) will result in a
 * \a std::invalid_argument exception.
 *
 * The following actions must not be performed inside an exclusive or root section:
 * - starting another exclusive section (i.e. a nested exclusive section); this results in a
 *   \a std::runtime_error exception)
 * - starting a process serialization (see pe::pe_SERIALIZATION)
 * - the setup of infinite rigid bodies (as for instance the Plane geometric primitive)
 * - translating or rotating global rigid bodies (i.e. rigid bodies created inside a
 *   pe::pe_GLOBAL_SECTION) via setPosition(), setOrientation(), translate(), or any rotate()
 *   function
 *
 * The following functions must not be called inside an exclusive section:
 * - the pe::World::simulationStep() function
 * - the pe::World::run() function
 * - the pe::World::synchronize() function
 * - the pe::MPISystem::checkProcesses() function
 * - any visualization functionality (e.g. the pe::povray::activateWriter() function or the
 *   pe::povray::Writer::writeFile() function)
 */
#define pe_ROOT_SECTION \
   if( pe::ExclusiveSection exclusiveSection = MPISettings::root() )
//*************************************************************************************************

} // namespace pe

#endif
