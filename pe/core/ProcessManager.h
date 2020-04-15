//=================================================================================================
/*!
 *  \file pe/core/ProcessManager.h
 *  \brief Header file for the ProcessManager class
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

#ifndef _PE_CORE_PROCESSMANAGER_H_
#define _PE_CORE_PROCESSMANAGER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for process managers.
 * \ingroup core
 *
 * The ProcessManager class represents the interface for classes taking responsibility for
 * (remote) processes during an MPI parallel simulation. Processes can be added and removed
 * from process managers and the process manager takes full responsibility for the memory
 * management of all contained processes.\n
 * The ProcessManager class gives derived classes a certain number of additional rights for
 * the management of processes. The most important of these rights is the option to destroy
 * a process in case it is not needed anymore.
 */
class ProcessManager
{
public:
   //**Add/remove functions************************************************************************
   /*!\name Add/remove functions */
   //@{
   virtual void add   ( ProcessID process ) = 0;
   virtual void remove( ProcessID process ) = 0;
   //@}
   //**********************************************************************************************

protected:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~ProcessManager() = 0;
   //@}
   //**********************************************************************************************

   //**Process manager functions*******************************************************************
   /*!\name Process manager functions */
   //@{
   static void destroyProcess( ProcessID process );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
