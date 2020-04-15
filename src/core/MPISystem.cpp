//=================================================================================================
/*!
 *  \file src/core/MPISystem.cpp
 *  \brief Source file for the MPI communication system
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

#include <algorithm>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <vector>
#include <boost/numeric/conversion/cast.hpp>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPITag.h>
#include <pe/core/MPITrait.h>
#include <pe/core/ParallelTrait.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/RecvBuffer.h>
#include <pe/core/SendBuffer.h>
#include <pe/math/Vector3.h>
#include <pe/util/Assert.h>
#include <pe/util/Byte.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Logging.h>
#include <pe/util/Vector.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the MPISystem class.
 */
MPISystem::MPISystem()
   : ProcessManager()                        // Initialization of the ProcessManager base object
   , Singleton< MPISystem,logging::Logger,CollisionSystem<Config> >()  // Initialization of the Singleton base object
{
   // Logging the successful setup of the MPI system
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the MPI system instance";
   }
}
//*************************************************************************************************





//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the MPISystem class.
 *
 * During the destruction of the MPI system, all connected remote MPI processes are also
 * destroyed.
 */
MPISystem::~MPISystem()
{
   // Destroying the contained remote processes
   const Iterator pbegin( begin() );
   const Iterator pend  ( end()   );

   for( Iterator process=pbegin; process!=pend; ++process )
      destroyProcess( *process );

   // WARNING: Using friend relationship to clear process storage.
   theCollisionSystem()->processstorage_.clear();

   // Logging the successful destruction of the MPI system
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the MPI system instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  PROCESS MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a remote process to the MPI system.
 *
 * \param process The remote process to be added to the MPI system.
 * \return void
 *
 * This function adds the given remote process to the MPI system. The MPI system takes full
 * responsibility for the process, including the necessary memory management.
 *
 * \b Note: This function doesn't have to be called explicitly. It is automatically called
 * in case a new remote MPI process is created.
 */
void MPISystem::add( ProcessID process )
{
   // WARNING: Using friend relationship to add process to process storage.
   theCollisionSystem()->processstorage_.add( process );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a remote MPI process from the MPI system.
 *
 * \param process The process to be removed.
 * \return void
 *
 * This function is a requirement for all process manager instances and removes/deregisters
 * a process from its manager.
 *
 * \b Note: This function doesn't have to be called explicitly. It is automatically called in
 * case the process manager is changed or if the remote process is destroyed.
 */
void MPISystem::remove( ProcessID process )
{
   // Removing the process from the process storage
   const Iterator pos( std::find( begin(), end(), process ) );
   pe_INTERNAL_ASSERT( pos != end(), "Process is not contained in the process storage" );

   // WARNING: Using friend relationship to remove process from process storage.
   theCollisionSystem()->processstorage_.remove( pos );
}
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checking the MPI process setup.
 *
 * \return \a true if no setup error is detected, \a false if a setup error is encountered.
 * \exception std::runtime_error Invalid function call inside exclusive section.
 * \exception std::runtime_error Invalid connection.
 *
 * This function checks the system wide setup of the remote MPI processes and throws an
 * \a std::runtime_error exception in case any error is detected. Currently, only one possible
 * error case can be detected:
 *
 *  - two processes are only unidirectionally connected
 *  - the neighbor subdomain description contradicts the description of the local domain of the neighbor in the case of half spaces
 *
 * \b Note: This function must not be called from inside an exclusive section. Calling this
 * function inside an exclusive section results in a \a std::runtime_error exception!
 */
void MPISystem::checkProcesses() const
{
   using boost::numeric_cast;

   // Early exit in case of non-parallel simulations
   if( MPISettings::size() == 1 )
      return;

   // Checking if the function is called inside an exclusive section
   if( ExclusiveSection::isActive() )
      throw std::runtime_error( "Invalid function call inside exclusive section" );

   // Checking whether the configuration is parallel
   pe_INTERNAL_ASSERT( ParallelTrait<Config>::value == 1, "Non-parallel configuration detected" );

#if HAVE_MPI
   const int          p    ( MPISettings::size() );        // The total number of MPI processes
   const int          rank ( MPISettings::rank() );        // The rank of this MPI process
   const MPI_Comm     comm ( MPISettings::comm() );        // The current MPI communicator
         int          m    ( numeric_cast<int>( theCollisionSystem()->getProcessStorage().size() ) );

   // Checking whether connections are symmetric.
   int result;
   if( rank == 0 ) {
      std::vector<int> neighborsSizes( p );

      MPI_Gather(&m, 1, MPITrait<int>::getType(), &neighborsSizes[0], 1, MPITrait<int>::getType(), 0, comm );

      std::vector<int> neighborsOffsets( p + 1 );
      for( int i = 0; i < p; ++i )
         neighborsOffsets[i+1] = neighborsOffsets[i] + neighborsSizes[i];

      // Preparing the array containing all neighbor rank lists.
      std::vector<int> neighbors( neighborsOffsets[p] );
      {
         size_t i( 0 );
         for( ConstIterator it=begin(); it!=end(); ++it, ++i )
            neighbors[i] = it->getRank();

         std::sort(neighbors.begin(), neighbors.begin() + m );
      }

      MPI_Gatherv( MPI_IN_PLACE, m, MPITrait<int>::getType(), &neighbors[0], &neighborsSizes[0], &neighborsOffsets[0], MPITrait<int>::getType(), 0, comm );

      std::vector<bool> visited( p );

      // Note: Graph is not necessarily one component.
      bool symmetric( true );
      for( int i = 0; i < p; ++i ) {
         if( !visited[i] )
            symmetric &= isSymmetric( neighbors, neighborsOffsets, visited, i );
      }
      result = symmetric;
   }
   else {
      MPI_Gather(&m, 1, MPITrait<int>::getType(), 0, 0, 0, 0, comm );

      // Preparing the array containing this ranks neighbor rank list.
      std::vector<int> neighbors( m );
      {
         size_t i( 0 );
         for( ConstIterator it=begin(); it!=end(); ++it, ++i )
            neighbors[i] = it->getRank();
      }

      std::sort(neighbors.begin(), neighbors.begin() + m );
      MPI_Gatherv( &neighbors[0], m, MPITrait<int>::getType(), 0, 0, 0, 0, 0, comm );
   }

   // Root process broadcasts the result.
   MPI_Bcast( &result, 1, MPI_INT, 0, comm );

   if( result == 0 )
      throw std::runtime_error( "Process connections are asymmetric." );

   // Make a list of a HalfSpaces defining our subdomain and send it to all neighbors.
   std::vector<MPI_Request> requests( m );
   std::list< std::pair<Vec3, real> > localDescriptionOfLocalDomain;
   {
      theCollisionSystem()->getDomain().getGeometry()->extractHalfSpaces( localDescriptionOfLocalDomain );

      SendBuffer<NoEndiannessConversion> sendbuffer( 10000 );

      int numHalfspaces( numeric_cast<int>( localDescriptionOfLocalDomain.size() ) );
      sendbuffer << numHalfspaces;
      for( std::list< std::pair<Vec3, real> >::iterator it = localDescriptionOfLocalDomain.begin(); it != localDescriptionOfLocalDomain.end(); ++it ) {
         marshal( sendbuffer, it->first );
         marshal( sendbuffer, it->second );
      }

      size_t i( 0 );
      for( ConstIterator it=begin(); it!=end(); ++it, ++i ) {
         MPI_Isend( sendbuffer.ptr(), numeric_cast<int>( sendbuffer.size() ), MPITrait<byte>::getType(), it->getRank(), mpitagMPISystemCheckProcesses, comm,  &requests[i] );
      }
   }

   RecvBuffer<NoEndiannessConversion> recvbuffer;
   for( ConstIterator it=begin(); it!=end(); ++it ) {
      // Probing the size of the MPI message.
      MPI_Status status;
      MPI_Probe( it->getRank(), mpitagMPISystemCheckProcesses, comm, &status );

      int count;
      MPI_Get_count( &status, MPITrait<byte>::getType(), &count );  // The number of elements in the MPI message

      // Receiving the MPI message.
      recvbuffer.resize( count );
      MPI_Recv( recvbuffer.ptr(), count, MPITrait<byte>::getType(), it->getRank(), mpitagMPISystemCheckProcesses, comm, &status );

      // Parsing the MPI message.
      std::list< std::pair<Vec3, real> > remoteDescriptionOfRemoteDomain;

      int numHalfspaces;
      unmarshal( recvbuffer, numHalfspaces );
      for( int i = 0; i < numHalfspaces; ++i ) {
         Vec3 n;
         real d;
         unmarshal( recvbuffer, n );
         unmarshal( recvbuffer, d );

         remoteDescriptionOfRemoteDomain.push_back( std::make_pair( n, d - trans( n ) * it->getOffset() ) );
      }

      // Compare with local description of neighbor.
      std::list< std::pair<Vec3, real> > localDescriptionOfRemoteDomain;
      it->getGeometry()->extractHalfSpaces( localDescriptionOfRemoteDomain );

      // Note: _Typically_ all halfspaces in the local description of the remote domain are identical to the remote description of the remote domain.

      {
         std::vector<bool> deficient( localDescriptionOfRemoteDomain.size() );
         bool onedeficient( false );
         size_t i = 0;
         for( std::list< std::pair<Vec3, real> >::iterator it2 = localDescriptionOfRemoteDomain.begin(); it2 != localDescriptionOfRemoteDomain.end(); ++it2, ++i ) {
            bool found( false );
            for( std::list< std::pair<Vec3, real> >::iterator it3 = remoteDescriptionOfRemoteDomain.begin(); it3 != remoteDescriptionOfRemoteDomain.end(); ++it3 ) {
               if( *it2 == *it3 ) {
                  found = true;
                  break;
               }
            }

            if( !found ) {
               deficient[i] = true;
               onedeficient = true;
            }
         }

         if( onedeficient ) {
            pe_LOG_WARNING_SECTION( log ) {
               log << "The description of the remote domain of rank " << it->getRank() << " on process " << rank << " contains at least one halfspace not present in the description of the corresponding domain on process " << it->getRank() << ".\n";
               log << "Local description of the remote domain (possibly a superset):\n";
               i = 0;
               for( std::list< std::pair<Vec3, real> >::iterator it2 = localDescriptionOfRemoteDomain.begin(); it2 != localDescriptionOfRemoteDomain.end(); ++it2, ++i )
                  log << "   n = " << it2->first << ", d = " << it2->second << (deficient[i] ? "  *\n" : "\n");
               log << "Remote description of the remote domain (necessarily exact):\n";
               for( std::list< std::pair<Vec3, real> >::iterator it2 = remoteDescriptionOfRemoteDomain.begin(); it2 != remoteDescriptionOfRemoteDomain.end(); ++it2 )
                  log << "   n = " << it2->first << ", d = " << it2->second << "\n";
            }
         }
      }

      recvbuffer.clear();
   }

   // Waiting for the non-blocking send operations to complete
   std::vector<MPI_Status> stats( m );
   MPI_Waitall( m, &requests[0], &stats[0] );
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Determines whether a part of a graph represented by adjacency lists is symmetric.
 *
 * \param adjlists All adjacency lists concatenated into a single vector.
 * \param adjoffsets The offsets in adjlists where the adjacency list of each node starts. The vector must contain one additional entry at the end containing the length of the adjlists array.
 * \param visited A vector marking all nodes as true which were already checked for symmetry.
 * \param node The node whose symmetry is to be checked. Neighboring nodes which have not been checked yet are checked for symmetry recursively.
 * \return \a true if it is symmetric, \a false if it is not.
 *
 * Performs a depth-first search on the (unvisited) graph and checks for all outgoing edges of every
 * visited node whether a back-edge exists. A missing back-edge is logged as an error. The graph is
 * traversed completely (reachable from the initial node) so that all missing back-edges are found
 * even if an error is encountered.
 */
bool MPISystem::isSymmetric( const std::vector<int>& adjlists, const std::vector<int>& adjoffsets, std::vector<bool>& visited, int node ) const {
   visited[node] = true;

   bool symmetric( true );
   for( int i = adjoffsets[node]; i < adjoffsets[node+1]; ++i ) {
      int j( adjlists[i] );

      if( !std::binary_search( adjlists.begin() + adjoffsets[j], adjlists.begin() + adjoffsets[j+1], node ) ) {
         symmetric = false;

         pe_LOG_ERROR_SECTION( log ) {
            log << "Process " << node << " is connected to process " << j << " but not the other way round.\n";
         }
      }

      if( !visited[j] )
         symmetric &= isSymmetric( adjlists, adjoffsets, visited, j );
   }

   return symmetric;
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of the MPI system.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void MPISystem::print( std::ostream& os ) const
{
   os << " Number of connected MPI processes = " << theCollisionSystem()->getProcessStorage().size() << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the MPI system.
 * \ingroup mpi
 *
 * \param os Reference to the output stream.
 * \param ms Reference to a constant MPI system object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const MPISystem& ms )
{
   os << "--" << pe_BROWN << "MPI SYSTEM PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------\n";
   ms.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for MPI system handles.
 * \ingroup mpi
 *
 * \param os Reference to the output stream.
 * \param ms MPI system handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const MPISystemID& ms )
{
   os << "--" << pe_BROWN << "MPI SYSTEM PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------\n";
   ms->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for constant MPI system handles.
 * \ingroup mpi
 *
 * \param os Reference to the output stream.
 * \param ms Constant MPI system handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const ConstMPISystemID& ms )
{
   os << "--" << pe_BROWN << "MPI SYSTEM PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------\n";
   ms->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
