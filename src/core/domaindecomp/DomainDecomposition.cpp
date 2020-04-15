//=================================================================================================
/*!
 *  \file src/core/domaindecomp/DomainDecomposition.cpp
 *  \brief Source file for domain decomposition setup functions.
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#include <pe/core/domaindecomp/DomainDecomposition.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>


namespace pe {

//=================================================================================================
//
//  REMOTE PROCESS SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether a remote processes has already been connected.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \return \a true if the remote process is already connected, \a false if not.
 */
bool isConnected( int rank )
{
   const ProcessStorage<Config>& processstorage( theCollisionSystem()->getProcessStorage() );
   return processstorage.find( rank ) != processstorage.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a connection to a remote MPI process.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \param a The x-component of the plane's normal vector.
 * \param b The y-component of the plane's normal vector.
 * \param c The z-component of the plane's normal vector.
 * \param d The plane's displacement from the global origin.
 * \param offset The offset between the two processes.
 * \return void
 * \exception std::runtime_error Selected configuration is not MPI parallel.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid MPI rank.
 * \exception std::invalid_argument Remote process is already connected.
 *
 * This function establishes a connection to a remote MPI process. From the next time step on,
 * this process and the connected process will exchange information about rigid bodies crossing
 * the shared process boundary via MPI. The plane shaped boundary of the half space occupied by
 * the remote process is defined by the given plane normal \a (a,b,c) and the displacement from
 * the global origin \a d. The \a offset argument specifies the offset between the two processes
 * (as for instance necessary in case of a periodic boundary).\n
 * The connection between the two processes has to be established locally and on the remote
 * process \a rank. The defined boundaries between this and the remote process have to match,
 * i.e. have to be the reverse of each other:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, 1.0, 1.0, 0.0, std::sqrt(2.0) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, -1.0, -1.0, 0.0, -std::sqrt(2.0) );
   }
   \endcode

 * \image html connect.png
 * \image latex connect.eps "Setup of two processes" width=560pt
 *
 * The setup of the remote process is terminated in any of the following cases:
 *  - the given plane's normal is zero
 *  - the given rank is invalid (including the attempt to connect a process to itself)
 *  - if an already connected process is connected a second time
 * Any of these cases results in a \a std::invalid_argument exception.
 *
 * \b Note: It is only possible to connect to a remote MPI process in case the selected collision
 * solver provides an MPI parallel execution. The attempt to connect to a remote MPI
 * process with any other collision solver results in a \a std::runtime_error exception.
 */
void connect( int rank, real a, real b, real c, real d, const Vec3& offset )
{
   connect_backend( rank, std::auto_ptr<ProcessGeometry>( new HalfSpace( a, b, c, d ) ), offset );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a connection to a remote MPI process.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \param normal The normal vector of the plane, \f$ |n| > 0 \f$.
 * \param d The plane's displacement from the global origin.
 * \param offset The offset between the two processes.
 * \return void
 * \exception std::runtime_error Selected configuration is not MPI parallel.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid MPI rank.
 * \exception std::invalid_argument Remote process is already connected.
 *
 * This function establishes a connection to a remote MPI process. From the next time step on,
 * this process and the connected process will exchange information about rigid bodies crossing
 * the shared process boundary via MPI. The plane shaped boundary of the half space occupied by
 * the remote process is defined by the given plane normal \a normal and the displacement from
 * the global origin \a d. The \a offset argument specifies the offset between the two processes
 * (as for instance necessary in case of a periodic boundary).\n
 * The connection between the two processes has to be established locally and on the remote
 * process \a rank. The defined boundaries between this and the remote process have to match,
 * i.e. have to be the reverse of each other:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, Vec3( 1.0, 1.0, 0.0 ), std::sqrt(2.0) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, Vec3( -1.0, -1.0, 0.0 ), -std::sqrt(2.0) );
   }
   \endcode

 * \image html connect.png
 * \image latex connect.eps "Setup of two processes" width=560pt
 *
 * The setup of the remote process is terminated in any of the following cases:
 *  - the given plane's normal is zero
 *  - the given rank is invalid (including the attempt to connect a process to itself)
 *  - if an already connected process is connected a second time
 * Any of these cases results in a \a std::invalid_argument exception.
 *
 * \b Note: It is only possible to connect to a remote MPI process in case the selected collision
 * solver provides an MPI parallel execution. The attempt to connect to a remote MPI
 * process with any other collision solver results in a \a std::runtime_error exception.
 */
void connect( int rank, const Vec3& normal, real d, const Vec3& offset )
{
   connect_backend( rank, std::auto_ptr<ProcessGeometry>( new HalfSpace( normal, d ) ), offset );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a connection to a remote MPI process.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \param a The x-component of the plane's normal vector.
 * \param b The y-component of the plane's normal vector.
 * \param c The z-component of the plane's normal vector.
 * \param x The global x-position of the boundary point.
 * \param y The global y-position of the boundary point.
 * \param z The global z-position of the boundary point.
 * \param offset The offset between the two processes.
 * \return void
 * \exception std::runtime_error Selected configuration is not MPI parallel.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid MPI rank.
 * \exception std::invalid_argument Remote process is already connected.
 *
 * This function establishes a connection to a remote MPI process. From the next time step on,
 * this process and the connected process will exchange information about rigid bodies crossing
 * the shared process boundary via MPI. The plane shaped boundary of the half space occupied by
 * the remote process is defined by the given plane normal \a (a,b,c) and the global coordinate
 * \a (x,y,z) that is contained in the boundary plane. The \a offset argument specifies the offset
 * between the two processes (as for instance necessary in case of a periodic boundary).\n
 * The connection between the two processes has to be established locally and on the remote
 * process \a rank. The defined boundaries between this and the remote process have to match,
 * i.e. have to be the reverse of each other:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, 1.0, 1.0, 0.0, 0.0, 2.0, 0.0 );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, -1.0, -1.0, 0.0, 0.0, 2.0, 0.0 );
   }
   \endcode

 * \image html connect.png
 * \image latex connect.eps "Setup of two processes" width=560pt
 *
 * The setup of the remote process is terminated in any of the following cases:
 *  - the given plane's normal is zero
 *  - the given rank is invalid (including the attempt to connect a process to itself)
 *  - if an already connected process is connected a second time
 * Any of these cases results in a \a std::invalid_argument exception.
 *
 * \b Note: It is only possible to connect to a remote MPI process in case the selected collision
 * solver provides an MPI parallel execution. The attempt to connect to a remote MPI
 * process with any other collision solver results in a \a std::runtime_error exception.
 */
void connect( int rank, real a, real b, real c, real x, real y, real z, const Vec3& offset )
{
   connect_backend( rank, std::auto_ptr<ProcessGeometry>( new HalfSpace( a, b, c, x, y, z ) ), offset );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a connection to a remote MPI process.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \param normal The normal vector of the plane, \f$ |n| > 0 \f$.
 * \param gpos The global position of the boundary point.
 * \param offset The offset between the two processes.
 * \return void
 * \exception std::runtime_error Selected configuration is not MPI parallel.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid MPI rank.
 * \exception std::invalid_argument Remote process is already connected.
 *
 * This function establishes a connection to a remote MPI process. From the next time step on,
 * this process and the connected process will exchange information about rigid bodies crossing
 * the shared process boundary via MPI. The plane shaped boundary of the half space occupied by
 * the remote process is defined by the given plane normal \a normal and the global coordinate
 * \a gpos that is contained in the boundary plane. The \a offset argument specifies the offset
 * between the two processes (as for instance necessary in case of a periodic boundary).\n
 * The connection between the two processes has to be established locally and on the remote
 * process \a rank. The defined boundaries between this and the remote process have to match,
 * i.e. have to be the reverse of each other:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, Vec3( 1.0, 1.0, 0.0 ), Vec3( 0.0, 2.0, 0.0 ) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, Vec3( -1.0, -1.0, 0.0 ), Vec3( 0.0, 2.0, 0.0 ) );
   }
   \endcode

 * \image html connect.png
 * \image latex connect.eps "Setup of two processes" width=560pt
 *
 * The setup of the remote process is terminated in any of the following cases:
 *  - the given plane's normal is zero
 *  - the given rank is invalid (including the attempt to connect a process to itself)
 *  - if an already connected process is connected a second time
 * Any of these cases results in a \a std::invalid_argument exception.
 *
 * \b Note: It is only possible to connect to a remote MPI process in case the selected collision
 * solver provides an MPI parallel execution. The attempt to connect to a remote MPI
 * process with any other collision solver results in a \a std::runtime_error exception.
 */
void connect( int rank, const Vec3& normal, const Vec3& gpos, const Vec3& offset )
{
   connect_backend( rank, std::auto_ptr<ProcessGeometry>( new HalfSpace( normal, gpos ) ), offset );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a connection to a remote MPI process.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \param geometry Handle to the geometry/physical expansion of the remote MPI process.
 * \param offset The offset between the two processes.
 * \return void
 * \exception std::runtime_error Selected configuration is not MPI parallel.
 * \exception std::invalid_argument Invalid MPI rank.
 * \exception std::invalid_argument Remote process is already connected.
 *
 * This function establishes a connection to a remote MPI process. From the next time step on,
 * this process and the connected process will exchange information about rigid bodies crossing
 * the shared process boundary via MPI. The geometry of the remote process, i.e. the physical
 * expansion occupied by it, is specified by the process geometry \a geometry. The \a offset
 * argument specifies the offset between the two processes (as for instance necessary in case of
 * a periodic boundary).\n
 * The connection between the two processes has to be established locally and on the remote
 * process \a rank. The implicitly defined boundaries between this and the remote process have
 * to match, i.e. have to be the reverse of each other:

   \code
   // Connecting two MPI processes
   pe_EXCLUSIVE_SECTION( 0 ) {
      // Connecting the local process 0 with the remote process 1
      pe::connect( 1, pe::intersect( pe::HalfSpace( 1.0, 0.0, 0.0, 2.0 ),
                                     pe::HalfSpace( 0.0, 1.0, 0.0, 1.0 ) ) );
   }
   pe_EXCLUSIVE_SECTION( 1 ) {
      // Connecting the local process 1 with the remote process 0
      pe::connect( 0, pe::merge( pe::HalfSpace( -1.0, 0.0, 0.0, -2.0 ),
                                 pe::HalfSpace( 0.0, -1.0, 0.0, -1.0 ) ) );
   }
   \endcode

 * \image html connect2.png
 * \image latex connect2.eps "Setup of two processes" width=560pt
 *
 * The setup of the remote process is terminated in any of the following cases:
 *  - the given rank is invalid (including the attempt to connect a process to itself)
 *  - if an already connected process is connected a second time
 * Any of these cases results in a \a std::invalid_argument exception.
 *
 * \b Note: It is only possible to connect to a remote MPI process in case the selected collision
 * solver provides an MPI parallel execution. The attempt to connect to a remote MPI
 * process with any other collision solver results in a \a std::runtime_error exception.
 */
void connect_backend( int rank, std::auto_ptr<ProcessGeometry> geometry, const Vec3& offset )
{
   typedef ProcessStorage<Config>  PS;

   // Checking if the selected configuration provides an MPI parallel execution
   if( ParallelTrait<Config>::value == 0 )
      throw std::runtime_error( "Selected configuration is not MPI parallel" );

   // Checking the rank of the remote process
   if( rank < 0 || rank >= MPISettings::size() || rank == MPISettings::rank() )
      throw std::invalid_argument( "Invalid MPI rank" );

   const PS& processstorage( theCollisionSystem()->getProcessStorage() );

   // Checking if the remote process is already connected
   if( processstorage.find( rank ) != processstorage.end() )
      throw std::invalid_argument( "Remote process is already connected" );

   // Creating a new remote process
   ProcessID process = new Process( rank, geometry, offset );

   // Registering the new remote process
   theMPISystem()->add( process );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Disconnecting a remote MPI process.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \return void
 *
 * This function disconnects the remote MPI process \a rank from the local process. This will
 * stop any MPI communication between the two processes.
 */
void disconnect( int rank )
{
   const MPISystemID mpiSystem( theMPISystem() );

   typedef MPISystem::Iterator  Iterator;
   const Iterator begin( mpiSystem->begin() );
   const Iterator end  ( mpiSystem->end()   );

   for( Iterator it=begin; it!=end; ++it ) {
      if( it->getRank() == rank ) {
         ProcessID process( *it );
         mpiSystem->remove( process );
         delete process;
         break;
      }
   }
}
//*************************************************************************************************

} // namespace pe
