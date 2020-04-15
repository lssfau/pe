//=================================================================================================
/*!
 *  \file src/core/collisionsystem/MPICommunication.cpp
 *  \brief Source file for the MPICommunication class
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/collisionsystem/MPICommunication.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/MPITrait.h>
#include <pe/core/ProfilingSection.h>
#include <pe/math/Vector3.h>
#include <utility>



namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the MPICommunication class.
 *
 * \param processstorage Reference to the central processstorage.
 * \param domain Reference to the local domain.
 */
MPICommunication::MPICommunication( PS& processstorage, Domain& domain )
   : processstorage_( processstorage )
   , domain_( domain )
{
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the MPICommunication class.
 */
//MPICommunication::~MPICommunication()
//{}
//*************************************************************************************************




//=================================================================================================
//
//  COMMUNICATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Performs the MPI communication with all connected remote processes.
 *
 * \param tag Message tag for the MPI communications.
 * \return void
 *
 * This function flushes all send buffers to remote processes and awaits exactly one response per
 * sent message. The communication routine does \em not ensure that the received messages are
 * from distinct processes.
 */
void MPICommunication::communicate( int tag )
{
   const Iterator pbegin( processstorage_.begin() );
   const Iterator pend  ( processstorage_.end()   );

   size_t proc( 0 );  // Process counter

   // Acquiring the total number of connected processes
   const size_t size( processstorage_.size() );

   // Adjusting the number of MPI requests and status objects
   requests_.resize( size );
   stats_.resize   ( size );

   // Sending the gathered data to the remote processes
   for( Iterator process=pbegin; process!=pend; ++process, ++proc ) {
      process->send( tag, &requests_[proc] );
   }

   // Receiving data from the remote processes
   for( proc=0; proc<size; ++proc ) {
      receiveNextMessage( tag );
   }

   // Waiting for the non-blocking send operations to complete
   MPI_Waitall( static_cast<int>( size ), &requests_[0], &stats_[0] );

   pe_PROFILING_SECTION {
      for( Iterator process=pbegin; process!=pend; ++process ) {
         sentOverall_.transfered    ( process->getSendBuffer().size() );
         receivedOverall_.transfered( process->getRecvBuffer().size() );
      }
   }

   // Clearing the send buffers of the processes
   for( Iterator process=pbegin; process!=pend; ++process ) {
      process->clear();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Blocking receive operation of the next incoming MPI message.
 *
 * \param tag Message tag for the MPI communication.
 * \return Handle to the receiving process.
 *
 * This function receives the next incoming MPI message from any connected process and stores it
 * in the receive buffer of the process.
 */
ProcessID MPICommunication::receiveNextMessage( int tag )
{
   const MPI_Comm     comm( MPISettings::comm() );
   const MPI_Datatype type( MPITrait<byte>::getType() );
   MPI_Status status;
   int count( 0 );

   // Probing for the next incoming MPI message
   MPI_Probe( MPI_ANY_SOURCE, tag, comm, &status );

   // Estimating the size of the MPI message
   MPI_Get_count( &status,   // The communication status
                  type,      // Data type of the elements to be received
                  &count );  // The number of elements in the MPI message

   // Searching for the receiving MPI process
   Iterator process( processstorage_.find( status.MPI_SOURCE ) );
   pe_INTERNAL_ASSERT( process != processstorage_.end(), "Remote process is not connected" );
   pe_INTERNAL_ASSERT( status.MPI_SOURCE == process->rank_, "Invalid receiving process detected" );

   // Receiving the MPI message
   process->recv_.resize( count );
   MPI_Recv( process->recv_.ptr(),  // Initial address of the receive buffer
             count,                 // Number of elements to be received
             type,                  // Data type of each receive buffer element
             process->rank_,        // Rank of the source process
             tag,                   // MPI communication tag
             comm,                  // The MPI communicator
             &status );             // The communication status

   // Returning a handle to the receiving process
   return *process;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Finds the owner of the provided point.
 *
 * \param gpos The global position whose owner is to be determined.
 * \return Returns a pair of the rank and the handle of the process.
 *
 * Determines the neighboring process in whose subdomain the point is located. If it is located on
 * a shared surface, edge or corner then the process with the lowest rank is returned. The
 * routine is only correct if the point is actually located in the local process or in one of
 * specified neighboring subdomains. If it is located outside that region the result depends on
 * how exactly the neighboring subdomains are specified. Either the point is misleadingly found
 * found to be part of a neighboring process or found to be not part of any neighboring process.
 * The routine runs in linear time. If the point is not part of the specified process subdomains.
 * then -1 is returned as rank and a NULL handle. If it is part of the local subdomain then the
 * correct rank and a NULL handle is returned.
 */
std::pair<int, ProcessID> MPICommunication::findOwner( const Vec3& gpos )
{
   int ownerRank( std::numeric_limits<int>::max() );
   ProcessID ownerHandle( 0 );

   if( domain_.containsPoint( gpos ) )
      ownerRank = MPISettings::rank();

   for( PS::Iterator process=processstorage_.begin(); process!=processstorage_.end(); ++process ) {
      if( process->getRank() < ownerRank && process->containsPoint( gpos ) ) {
         ownerRank = process->getRank();
         ownerHandle = *process;
      }
   }

   if( ownerRank == std::numeric_limits<int>::max() )
      ownerRank = -1;

   return std::make_pair( ownerRank, ownerHandle );
}
//*************************************************************************************************


} // namespace pe
