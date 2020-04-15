//=================================================================================================
/*!
 *  \file src/core/ProcessManager.cpp
 *  \brief Source file for the ProcessManager class
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
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/domaindecomp/Process.h>
#include <pe/core/MPI.h>
#include <pe/core/ProcessManager.h>


namespace pe {

//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the ProcessManager class.
 */
ProcessManager::~ProcessManager()
{}
//*************************************************************************************************




//=================================================================================================
//
//  PROCESS MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destroying a process.
 *
 * \param process The process to be destroyed.
 * \return void
 *
 * This function allows the process manager to completely destroy the given process.
 */
void ProcessManager::destroyProcess( ProcessID process )
{
   delete process;
}
//*************************************************************************************************

} // namespace pe
