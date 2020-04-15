//=================================================================================================
/*!
 *  \file src/util/Thread.cpp
 *  \brief Source file for the Thread class
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

#include <boost/bind.hpp>
#include <pe/util/Assert.h>
#include <pe/util/Thread.h>
#include <pe/util/ThreadPool.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Starting a thread in a thread pool.
 *
 * \param pool Handle to the managing thread pool.
 *
 * This function creates a new thread in the given thread pool. The thread is kept alive until
 * explicitly killed by the managing thread pool.
 */
Thread::Thread( ThreadPool* pool )
   : terminated_( false )  // Thread termination flag
   , pool_      (  pool )  // Handle to the managing thread pool
   , thread_    (   0   )  // Handle to the thread of execution
{
   thread_.reset( new boost::thread( boost::bind( &Thread::run, this ) ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Thread class.
 */
Thread::~Thread()
{}
//*************************************************************************************************




//=================================================================================================
//
//  THREAD EXECUTION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Execution function for threads in a thread pool.
 *
 * This function is executed by any thread managed by a thread pool.
 */
void Thread::run()
{
   // Checking the thread pool handle
   pe_INTERNAL_ASSERT( pool_, "Uninitialized pool handle detected" );

   // Executing scheduled tasks
   while( pool_->executeTask() ) {}

   // Setting the termination flag
   terminated_ = true;
}
//*************************************************************************************************

} // namespace pe
