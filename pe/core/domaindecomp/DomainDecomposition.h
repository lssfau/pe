//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/DomainDecomposition.h
 *  \brief Header file for domain decomposition setup functions.
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

#ifndef _PE_CORE_DOMAINDECOMP_DOMAINDECOMPOSITION_H_
#define _PE_CORE_DOMAINDECOMP_DOMAINDECOMPOSITION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <memory>
#include <pe/core/CollisionSystem.h>
#include <pe/core/domaindecomp/ProcessGeometry.h>
#include <pe/core/MPI.h>
#include <pe/math/Vector3.h>


namespace pe {

//*************************************************************************************************
/*!\defgroup domaindecomp Domain Decomposition
 * \ingroup core
 *
 * \section mpi_general General
 *
 * The MPI module of the physics engine provides the necessary extensions for a distributed
 * memory parallelization via the message passing interface (MPI). This parallelization
 * allows for large scale rigid body simulations well beyond a billion rigid bodies that
 * cannot be handled (for instance due to memory restrictions) by a single computer.\n\n
 *
 *
 * \section mpi_setup Setup of the MPI communication
 *
 * An MPI parallel simulation of rigid bodies slightly differs from the standard simulations.
 * The basic component for MPI is the communication between remote processes. These processes
 * don't necessarily reside on the same machine and don't necessarily use the same memory.
 * Therefore the part of the simulation domain represented by a remote process cannot be
 * directly accessed by the local process. The only way to exchange rigid bodies close to the
 * boundary of the processes is a communication via MPI.\n
 * The communication to handle the rigid bodies in a parallel simulation is automatically
 * handled by the \a pe physics engine. However, the MPI communication has to be explicitly
 * established. The setup of a connection between the local and a remote MPI process is
 * done via the connect() functions:
 *
 * -# connect( int rank, real a, real b, real c, real d, const Vec3& offset );
 * -# connect( int rank, const Vec3& normal, real d, const Vec3& offset );
 * -# connect( int rank, real a, real b, real c, real x, real y, real z, const Vec3& offset );
 * -# connect( int rank, const Vec3& normal, const Vec3& gpos, const Vec3& offset );
 * -# connect( int rank, const Geometry& geometry, const Vec3& offset );
 *
 * The first four functions create a plane shaped boundary between the two processes that
 * divides the global space in two half spaces. One of these half spaces is the half space
 * of the local process, the other half space is the half space of the remote MPI process.
 * The plane is represented by the following equation:
 *
 *                                \f[ ax + by + cz = d , \f]
 *
 * where \a a, \a b and \a c are the x, y and z component of the normal vector. The normal
 * \a n of the plane is a normalized vector that always points towards the half space of
 * the remote process. \a d is the distance/displacement from the origin of the global world
 * frame to the plane. A positive value of \a d indicates that the origin of the global world
 * frame is inside the local process, whereas a negative value of \a d indicates that the
 * origin is inside the remote process. A value of 0 indicates that the origin is on the
 * surface of the plane. Any point on the surface of the plane is considered part of the
 * process with the lower rank.
 *
 * \image html process.png
 * \image latex process.eps "Remote MPI process" width=520pt
 *
 * The fifth connect() function allows for the individual definition of the space occupied
 * by the remote process. This for instance offers the option to intersect() or merge()
 * half spaces to define more complex process setups.\n
 * The \a offset argument offers the possibility to define boundaries with a certain offset
 * in-between, as for instance necessary for periodic boundaries. Rigid bodies crossing such
 * a process boundary are displaced by the specified offset. The default value for \a offset
 * is the zero vector, which corresponds to a standard, non-periodic boundary.\n
 * The following example demonstrates the setup of a parallel simulation with two MPI
 * processes. Process 0 occupies the half space of the negative x-axis, process 1 occupies
 * the half space of the positive x-axis. The process boundary between the two processes is
 * the yz-plane containing the origin of the global world frame:

   \code
   int main( int argc, char** argv )
   {
      using namespace pe;

      // Initialization of the MPI system
      // The MPI system must be initialized before any pe functionality is used. It is
      // recommended to make MPI_Init() the very first call of the main function.
      MPI_Init( &argc, &argv );

      // Setup of the remote processes
      // Since the same program is executed on every process, the setup of the remote
      // processes has to be individualized for every process. The following exclusive
      // sections are solely executed on the specified processes and perform the according
      // remote process setup depending on the rank of the local process.
      pe_EXCLUSIVE_SECTION( 0 )
      {
         // This function is called exclusively on process 0. It connects the process 0
         // with the remote process 1. The normal of the process boundary points towards
         // the remote process 1.
         pe::connect( 1, 1.0, 0.0, 0.0, 0.0 );
      }
      pe_EXCLUSIVE_SECTION( 1 )
      {
         // This function is called exclusively on process 1. It connects the process 1
         // with the remote process 0. The normal of the process boundary points towards
         // the remote process 0.
         pe::connect( 0, -1.0, 0.0, 0.0, 0.0 );
      }

      // Setup of the simulation
      ...

      // MPI finalizations
      MPI_Finalize();
   }
   \endcode

 * Note that for a correct parallel setup it is also necessary to explicitly connect adjacent
 * remote processes that have no common face with the local process. The following images
 * demonstrate the setup of a 2-dimensional (for reasons of simplicity) scenario with 4 MPI
 * processes. We are considering process 0 to be the local process. Although process 2 is
 * only diagonally connected to process 0 and has no common face/egde to this process, it
 * is still necessary to connect the two processes since rigid bodies may move directly from
 * process 0 to process 2 (and vice versa).
 *
 * \image html process2.png
 * \image latex process2.eps "Remote process setup for corner processes" width=800pt
 *
 * The same rule holds true in 3-dimensional scenarios for adjacent processes that are only
 * connected by a single edge or corner!\n\n
 *
 *
 * \section mpi_large_bodies Treatment of large rigid bodies
 *
 * Depending on the setup of the rigid body simulation, some rigid bodies may span several
 * processes. In this case, it is not only necessary to connect the directly neighboring
 * processes but additionally farther away processes. The following illustration shows the
 * necessary setup for a large particle agglomerate:
 *
 * \image html process3.png
 * \image latex process3.eps "Remote process setup for large rigid bodies" width=650pt
 */




//=================================================================================================
//
//  LOCAL PROCESS SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Local process setup functions */
//@{
template< typename Geometry >
void defineLocalDomain( const Geometry& geometry );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Defines the geometry of the local process.
 * \ingroup domaindecomp
 *
 * \param geometry The geometry/physical expansion of the local MPI process.
 * \return void
 * \exception std::runtime_error Selected configuration is not MPI parallel.
 *
 * Note that for a non-parallel simulation, this function will throw a runtime error since
 * there the local domain is always the whole space.
 */
template< typename Geometry >  // Type of the remote process geometry
void defineLocalDomain( const Geometry& geometry )
{
   // Checking if the selected configuration provides an MPI parallel execution
   if( ParallelTrait<Config>::value == 0 )
      throw std::runtime_error( "Selected configuration is not MPI parallel" );

   // WARNING: Using friend relationship to get non-constant reference to domain.
   theCollisionSystem()->domain_.geometry_ = std::auto_ptr<ProcessGeometry>( new Geometry( geometry ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  REMOTE PROCESS SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Remote process setup functions */
//@{
PE_PUBLIC bool isConnected( int rank );

PE_PUBLIC void connect( int rank, real a, real b, real c, real d, const Vec3& offset = Vec3() );
PE_PUBLIC void connect( int rank, const Vec3& normal, real d, const Vec3& offset = Vec3() );
PE_PUBLIC void connect( int rank, real a, real b, real c, real x, real y, real z, const Vec3& offset = Vec3() );
PE_PUBLIC void connect( int rank, const Vec3& normal, const Vec3& gpos, const Vec3& offset = Vec3() );

template< typename Geometry >
PE_PUBLIC void connect( int rank, const Geometry& geometry, const Vec3& offset = Vec3() );

PE_PUBLIC void disconnect( int rank );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
void PE_PUBLIC connect_backend( int rank, std::auto_ptr<ProcessGeometry> geometry, const Vec3& offset );
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a connection to a remote MPI process.
 * \ingroup domaindecomp
 *
 * \param rank The rank of the remote MPI process.
 * \param geometry The geometry/physical expansion of the remote MPI process.
 * \param offset The offset between the two processes.
 * \return void
 * \exception std::runtime_error Selected configuration is not MPI parallel.
 * \exception std::invalid_argument Invalid MPI rank.
 * \exception std::invalid_argument Remote process is already connected.
 *
 * This function establishes a connection to a remote MPI process. From the next time step on,
 * this process and the connected process will exchange information about rigid bodies crossing
 * the shared process boundary via MPI. The geometry of the remote process, i.e. the physical
 * expansion occupied by it, is specified by the process geometry \a geometry. The offset in
 * between the two processes is specified by the \a offset argument (the default is the zero
 * vector).\n
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
template< typename Geometry >  // Type of the remote process geometry
void connect( int rank, const Geometry& geometry, const Vec3& offset )
{
   connect_backend( rank, std::auto_ptr<ProcessGeometry>( new Geometry( geometry ) ), offset );
}
//*************************************************************************************************

} // namespace pe

#endif
