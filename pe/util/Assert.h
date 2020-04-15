//=================================================================================================
/*!
 *  \file pe/util/Assert.h
 *  \brief Header file for run time assertion macros
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

#ifndef _PE_UTIL_ASSERT_H_
#define _PE_UTIL_ASSERT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/Assertion.h>
#if HAVE_MPI
#  if !defined(NDEBUG)
#     include <pe/util/logging/ErrorSection.h>
#  endif
#else
#  include <cassert>
#endif


namespace pe {

//=================================================================================================
//
//  RUN TIME ASSERTION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup assert Assertions
 * \ingroup util
 */
/*!\defgroup runtime_assert Run time assertions
 * \ingroup assert
 */
/*!\brief Assertion helper function.
 * \ingroup runtime_assert
 *
 * The ASSERT_MESSAGE function is a small helper function to assist in printing an informative
 * message in case an assert fires. This function builds on the ideas of Matthew Wilson, who
 * directly combines a C-string error message with the run time expression (Imperfect C++,
 * ISBN: 0321228774):

   \code
   assert( ... &&  "Error message" );
   assert( ... || !"Error message" );
   \endcode

 * However, both approaches fail to compile without warning on certain compilers. Therefore
 * this inline function is used instead of the direct approaches, which circumvents all compiler
 * warnings:

   \code
   assert( ... || ASSERT_MESSAGE( "Error message" ) );
   \endcode
 */
inline bool ASSERT_MESSAGE( const char* /*msg*/ )
{
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
PE_PUBLIC void logStacktrace();
PE_PUBLIC void abort( int err = -1 );
PE_PUBLIC void exit( int exitcode = 0 );
//*************************************************************************************************




//=================================================================================================
//
//  RUN TIME ASSERTION MACROS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Run time assertion macro for internal checks.
 * \ingroup runtime_assert
 *
 * In case of an invalid run time expression, the program execution is terminated.\n
 * The pe_INTERNAL_ASSERT macro can be disabled by setting the \a pe_USER_ASSERTION flag to
 * zero or by defining \a NDEBUG during the compilation.
 */
#if pe_INTERNAL_ASSERTION
#  if HAVE_MPI
#     if defined(NDEBUG)
#        define pe_INTERNAL_ASSERT(expr,msg)
#     else
#        define pe_INTERNAL_ASSERT(expr,msg) \
            if ( !( expr ) ) { \
               pe_LOG_ERROR_SECTION( eqmshpioka ) { \
                  eqmshpioka << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": Assertion `" #expr "' failed. " << ( msg ) << "\n"; \
                  eqmshpioka.commit(); \
               } \
               pe::abort( 1 ); \
            }
#     endif
#  else
#     define pe_INTERNAL_ASSERT(expr,msg) assert( ( expr ) || ASSERT_MESSAGE( msg ) )
#  endif
#else
#  define pe_INTERNAL_ASSERT(expr,msg)
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Run time assertion macro for user checks.
 * \ingroup runtime_assert
 *
 * In case of an invalid run time expression, the program execution is terminated.\n
 * The pe_USER_ASSERT macro can be disabled by setting the \a pe_USER_ASSERT flag to zero
 * or by defining \a NDEBUG during the compilation.
 */
#if pe_USER_ASSERTION
#  if HAVE_MPI
#     if defined(NDEBUG)
#        define pe_USER_ASSERT(expr,msg)
#     else
#        define pe_USER_ASSERT(expr,msg) \
            if ( !( expr ) ) { \
               pe_LOG_ERROR_SECTION( eqmshpioka ) { \
                  eqmshpioka << __FILE__ << ":" << __LINE__ << ": " << __FUNCTION__ << ": Assertion `" #expr "' failed. " << ( msg ) << "\n"; \
                  eqmshpioka.commit(); \
               } \
               pe::abort( 2 ); \
            }
#     endif
#  else
#     define pe_USER_ASSERT(expr,msg) assert( ( expr ) || ASSERT_MESSAGE( msg ) )
#  endif
#else
#  define pe_USER_ASSERT(expr,msg)
#endif
//*************************************************************************************************

} // namespace pe

#endif
