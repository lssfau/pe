//=================================================================================================
/*!
 *  \file src/util/Assert.cpp
 *  \brief Implementation of run time assertion macros
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

#if HAVE_MPI
#  include <mpi.h>
#endif
#include <cstdlib>
#include <pe/util/Assert.h>
#include <pe/util/Unused.h>
//#include <stdexcept>
#include <limits.h>
#ifdef __GLIBC__
#  include <pe/util/logging/ErrorSection.h>
#  include <execinfo.h>
#endif

namespace pe {


//*************************************************************************************************
/*!\brief Helper function for logging a stack trace.
 * \ingroup runtime_assert
 *
 * Only supported if pe is compiled with glibc and gcc. The function names can only be determined
 * if the library is compiled with -rdynamic. Demangling is not performed but tools like c++filt
 * can be used. If the heap is corrupted this function might not be able to log the stack trace.
 * The stack trace is logged as an error.
 */
void logStacktrace()
{
#ifdef __GLIBC__
   size_t size = 0, prevsize = size;
   while( size == prevsize ) {
      prevsize = size + 128;
      void *buffer[prevsize];
      size = backtrace(buffer, prevsize);

      if( size < prevsize ) {
         char **strings = backtrace_symbols(buffer, size);
         if (strings == NULL)
            return;

         pe_LOG_ERROR_SECTION( log ) {
            for( size_t i = 0; i < size; ++i )
               log << strings[i] << "\n";
         }

         free( strings );
         break;
      }
   }
#endif
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Assertion helper function.
 * \ingroup runtime_assert
 * \param err Supplies an error number. Only supported in MPI parallel programs.
 *
 * Abnormal process termination. Gracefully aborts MPI processes if necessary.
 */
void abort( int err )
{
   logStacktrace();
#if HAVE_MPI
   //throw std::runtime_error("Abort");
   MPI_Abort( MPI_COMM_WORLD, err );
#else
   ::abort();
   UNUSED( err );
#endif
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Assertion helper function.
 * \ingroup runtime_assert
 * \param exitcode Supplies an exit code.
 *
 * Normal process termination. Gracefully exits MPI processes if necessary.
 */
void exit( int exitcode )
{
#if HAVE_MPI
   int flagInit( 0 ), flagFinalize( 0 );;
   MPI_Initialized( &flagInit );
   MPI_Finalized( &flagFinalize );

   if( flagInit && !flagFinalize ) {
      // BEWARE: We cannot ensure that pending MPI operations completed since destructors will be executed after the exit.
      MPI_Finalize();
   }
#endif
   ::exit( exitcode );
}
//*************************************************************************************************

} // namespace pe
