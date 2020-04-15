//=================================================================================================
/*!
 *  \file pe/util/threadpool/Task.h
 *  \brief Header file for the Task base class
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

#ifndef _PE_UTIL_THREADPOOL_TASK_H_
#define _PE_UTIL_THREADPOOL_TASK_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/NonCopyable.h>


namespace pe {

namespace threadpool {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for executable user tasks.
 * \ingroup threads
 *
 * The Task class represents the base class for all user tasks.
 */
class Task : private NonCopyable
{
public:
   //**Constructor*********************************************************************************
   // No explicitly declared constructor.
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Task();
   //@}
   //**********************************************************************************************

   //**Execution functions*************************************************************************
   /*!\name Execution functions */
   //@{
   virtual void run() = 0;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace threadpool

} // namespace pe

#endif
