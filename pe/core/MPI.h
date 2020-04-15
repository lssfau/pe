//=================================================================================================
/*!
 *  \file pe/core/MPI.h
 *  \brief Wrapper file for the inclusion of the MPI header file
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

#ifndef _PE_CORE_MPI_H_
#define _PE_CORE_MPI_H_

#if HAVE_MPI

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <mpi.h>

#else

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/Assert.h>


#ifndef MPI_IN_PLACE
#define MPI_IN_PLACE ((void *) 1)
#endif

namespace pe {

//=================================================================================================
//
//  MPI TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Handle for a MPI communicator.
 * \ingroup mpi
 */
typedef int  MPI_Comm;
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Handle for a MPI data type.
 * \ingroup mpi
 */
typedef int  MPI_Datatype;
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Handle for a MPI operations.
 * \ingroup mpi
 */
typedef int  MPI_Op;
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Handle for a MPI communication request.
 * \ingroup mpi
 */
typedef int  MPI_Request;
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Handle for a MPI communication status.
 * \ingroup mpi
 */
struct MPI_Status
{
   explicit inline MPI_Status()
      : count     ( 0 )
      , MPI_SOURCE( 0 )
      , MPI_TAG   ( 0 )
      , MPI_ERROR ( 0 )
   {}

   int count;       // Size of the message
   int MPI_SOURCE;  // Source of the message
   int MPI_TAG;     // Tag of the message
   int MPI_ERROR;   // Error flag
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  MPI CONSTANTS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
const MPI_Datatype MPI_CHAR               =  1;
const MPI_Datatype MPI_SHORT              =  2;
const MPI_Datatype MPI_INT                =  3;
const MPI_Datatype MPI_LONG               =  4;
const MPI_Datatype MPI_UNSIGNED_CHAR      =  5;
const MPI_Datatype MPI_UNSIGNED_SHORT     =  6;
const MPI_Datatype MPI_UNSIGNED           =  7;
const MPI_Datatype MPI_UNSIGNED_LONG      =  8;
const MPI_Datatype MPI_FLOAT              =  9;
const MPI_Datatype MPI_DOUBLE             = 10;
const MPI_Datatype MPI_LONG_DOUBLE        = 11;
const MPI_Datatype MPI_UNSIGNED_LONG_LONG = 12;
const MPI_Datatype MPI_SIGNED_CHAR        = 13;
const MPI_Datatype MPI_LONG_LONG          = 14;

const MPI_Datatype MPI_ANY_SOURCE     = -2;
const MPI_Datatype MPI_ANY_TAG        = -1;

const MPI_Op       MPI_MIN            = 100;
const MPI_Op       MPI_MAX            = 101;
const MPI_Op       MPI_SUM            = 102;
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  MPI FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Synchronizes all MPI processes in the given communicator.
 * \ingroup mpi
 *
 * \param comm The MPI communicator.
 * \return The return value is 0.
 *
 * This function is a stub for the MPI_Barrier() function for non-parallel simulations. This
 * function merely exists to guarantee the compilation of the physics engine even if the
 * \a mpi.h header file is not available. Note however, that calling this function during a
 * non-parallel simulation is invalid!
 */
inline int MPI_Barrier( MPI_Comm /*comm*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
   return 0;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Waits for a collection of non-blocking operations to complete.
 * \ingroup mpi
 *
 * \param count The number of non-blocking operations.
 * \param requests Array of active requests of length \a count.
 * \param stats Array of status objects of length \a count.
 * \return The return value is 0.
 *
 * This function is a stub for the MPI_Waitall() function for non-parallel simulations. This
 * function merely exists to guarantee the compilation of the physics engine even if the
 * \a mpi.h header file is not available. Note however, that calling this function during a
 * non-parallel simulation is invalid!
 */
inline int MPI_Waitall( int /*count*/, MPI_Request* /*requests*/, MPI_Status* /*stats*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
   return 0;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Non-blocking send operation.
 * \ingroup mpi
 *
 * \param buf Initial address of the send buffer.
 * \param count Number of elements in the send buffer.
 * \param datatype Data type of each send buffer element.
 * \param dest Rank of the destination process in the given communicator.
 * \param tag Message tag.
 * \param comm MPI communicator.
 * \param request Communication request.
 * \return The return value is 0.
 *
 * This function is a stub for the MPI_Isend() function for non-parallel simulations. This
 * function merely exists to guarantee the compilation of the physics engine even if the
 * \a mpi.h header file is not available. Note however, that calling this function during
 * a non-parallel simulation is invalid!
 */
inline int MPI_Isend( void* /*buf*/, int /*count*/, MPI_Datatype /*datatype*/, int /*dest*/,
                      int /*tag*/, MPI_Comm /*comm*/, MPI_Request* /*request*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
   return 0;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Check for incoming MPI messages.
 * \ingroup mpi
 *
 * \param source Rank of the source process in the given communicator.
 * \param tag Message tag.
 * \param comm MPI communicator.
 * \param status Communication status.
 * \return The return value is 0.
 *
 * This function is a stub for the MPI_Probe() function for non-parallel simulations. This
 * function merely exists to guarantee the compilation of the physics engine even if the
 * \a mpi.h header file is not available. Note however, that calling this function during
 * a non-parallel simulation is invalid!
 */
inline int MPI_Probe( int /*source*/, int /*tag*/, MPI_Comm /*comm*/, MPI_Status* /*status*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
   return 0;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Retrieving the size of an incoming MPI message.
 * \ingroup mpi
 *
 * \param status MPI communication status.
 * \param datatype Data type of the elements to be received.
 * \param count The number of elements in the MPI message.
 * \return The return value is 0.
 *
 * This function is a stub for the MPI_Get_count() function for non-parallel simulations.
 * This function merely exists to guarantee the compilation of the physics engine even if
 * the \a mpi.h header file is not available. Note however, that calling this function
 * during a non-parallel simulation is invalid!
 */
inline int MPI_Get_count( MPI_Status* /*status*/, MPI_Datatype /*datatype*/, int* /*count*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
   return 0;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Blocking receive operation.
 * \ingroup mpi
 *
 * \param buf Initial address of the receive buffer.
 * \param count Number of elements to be received.
 * \param datatype Data type of each receive buffer element.
 * \param source Rank of the source process in the given communicator.
 * \param tag Message tag.
 * \param comm MPI communicator.
 * \param status Communication status.
 * \return The return value is 0.
 *
 * This function is a stub for the MPI_Recv() function for non-parallel simulations. This
 * function merely exists to guarantee the compilation of the physics engine even if the
 * \a mpi.h header file is not available. Note however, that calling this function during
 * a non-parallel simulation is invalid!
 */
inline int MPI_Recv( void* /*buf*/, int /*count*/, MPI_Datatype /*datatype*/,
                     int /*source*/, int /*tag*/, MPI_Comm /*comm*/, MPI_Status* /*status*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
   return 0;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Applies a reduction operation on all tasks in the group and places the result in one task.
 * \ingroup mpi
 *
 * \param sendbuf Initial address of the send buffer.
 * \param recvbuf Initial address of the receive buffer.
 * \param count Number of elements in the send buffer.
 * \param datatype Data type of each send buffer element.
 * \param op The MPI reduction operation.
 * \param root Rank of the root process.
 * \param comm MPI communicator.
 * \return The return value is 0.
 *
 * This function is a stub for the MPI_Reduce() function for non-parallel simulations. This
 * function merely exists to guarantee the compilation of the physics engine even if the
 * \a mpi.h header file is not available. Note however, that calling this function during
 * a non-parallel simulation is invalid!
 */
inline int MPI_Reduce( void* /*sendbuf*/, void* /*recvbuf*/, int /*count*/,
                       MPI_Datatype /*datatype*/, MPI_Op /*op*/, int /*root*/, MPI_Comm /*comm*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
   return 0;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif

#endif
