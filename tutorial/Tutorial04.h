//=================================================================================================
/*!
 *  \file Tutorial04.h
 *  \brief Physics Engine Tutorial 04
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

#ifndef _PE_TUTORIAL04_H_
#define _PE_TUTORIAL04_H_


//*************************************************************************************************
/*!\page tutorial04 Tutorial 4: MPI parallel simulations
 *
 * \image html mpiwell.jpg
 *
 * This tutorial introduces all necessary extensions for large-scale, parallel rigid body
 * simulations via the message passing interface (MPI). These extensions are a special feature
 * of the \b pe engine that enable simulations of any number of rigid bodies on any number of
 * processors/cores. The simulation illustrated above shows some time steps of the 'mpiwell'
 * example in the example directory (see "examples/mpiwell/"). For this particular simulation,
 * 3000 rigid bodies were simulated using four MPI processes. The colors indicate which MPI
 * process the rigid bodies consider their local process.\n
 * In this tutorial it is assumed that you already have some background on MPI, i.e. you are
 * familiar with MPI ranks and the basic MPI functionality (MPI_Init(), MPI_Finalize(), ...).
 * In case you are not, but you are interested in large-scale, parallel rigid body simulations,
 * it it suggested you refer to some MPI introduction first before reading this tutorial.\n
 * There are only a couple of changes between a MPI parallel rigid body simulation and a
 * standard simulation. This tutorial explains all these differences and demonstrates the
 * setup of a MPI simulation using the 'mpiwell' example as point of reference. However,
 * for simplicity reasons, the example is only treated in a simplified configuration, i.e.
 * we will only consider falling spheres instead of arbitrary falling primitives. In case
 * you are interested in taking a look on the complete source code (including all details
 * as for instance the POV-Ray visualization and some time measurements), it can be found in
 * "<install-path>/examples/mpiwell/".\n
 * Let's start by discussing the necessary header files. In comparison to previous tutorials
 * there is only one important change for MPI parallel simulations. In order to avoid errors
 * during the compilation with some MPI implementations, the MPI header files should always
 * be included first. This can be done by either including the <mpi.h> header file directly, 
 * or by including either <pe/core.h> or <pe/engine.h> first in your source file. All three
 * options work, the latter two have the advantage that the MPI parallelization can also be
 * turned off in the configuration files without having to remove the MPI header file from
 * the source code.

   \code
   // In our example, we choose to include the header file for the core of the physics engine
   // as first file. Additionally, we include the utility module from the pe engine and some
   // standard library headers. To make things easier for us, we will also use a using directive
   // for the pe namespace.
   #include <pe/core.h>
   #include <pe/util.h>
   #include <cmath>
   #include <cstddef>
   #include <iostream>
   using namespace pe;
   \endcode

 * Our next step is the discussion of the main() function and the MPI functionality contained
 * in it. For this tutorial, we will use a fixed number of four MPI processes, each covering
 * a quarter of the simulation domain as illustrated below (Note that the number of processes
 * and their location is an arbitrary choice). The center of our well will be the origin of
 * the global coordinate system. This is also the point, where the illustrated west-east and
 * north-south boundaries cross each other. Process 0 covers the lower-left part of the
 * simulation domain, process 1 the lower-right, process 2 the upper-left and process 3 the
 * upper-right part.
 *
 * \image html tutorial04.jpg
 *
 * Let's take a look at the main() function:

   \code
   int main( int argc, char** argv )
   {
      // The very first function in every MPI parallel rigid body simulation should be the
      // initialization of the MPI system. As in any other MPI parallel code, this is done
      // by calling the MPI_Init() function and passing it the command line arguments. This
      // function should always be called before any pe functionality is used. In case this
      // function is omitted, the pe engine will assume that the simulation is running with
      // a single process.
      MPI_Init( &argc, &argv );

      ...
   \endcode

 * The next part of the main function contains some necessary simulation parameters that are
 * pretty self-explanatory. In this part of the source code there is no indication that we are
 * in fact running a MPI parallel simulation.

   \code
      ...

      /////////////////////////////////////
      // Simulation parameter definition

      // Timing variables
      const size_t timesteps   (  6000 );  // Total number of time steps
      const real   timestepsize( 0.005 );  // Size of a single time step

      // Total number of falling rigid bodies
      const size_t N ( 3000  );

      // Well parameters
      const real   R ( 30.0 );  // Radius of the well
      const size_t B ( 40   );  // Number of bricks per level
      const size_t H (  5   );  // Number of levels

      // General object parameters
      const real density    ( 1.0  );  // Density of the random objects
      const real restitution( 0.2  );  // Coefficient of restitution for the random objects
      const real friction   ( 0.25 );  // Coefficient of static and dynamic friction

      // Sphere parameters
      const real sphereRadiusMin( 0.6 );  // Minimal radius of a random sphere
      const real sphereRadiusMax( 1.2 );  // Maximal radius of a random sphere


      ///////////////////////////////
      // Configuration of the well

      const real brickangle( real(2)*PI / B );
      const Quat fullrotation( Vec3( 0, 0, 1 ), brickangle );
      const Quat halfrotation( Vec3( 0, 0, 1 ), real(0.5)*brickangle );
      const real brickX( std::sqrt( real(2)*sqr(R)*( real(1) - std::cos(brickangle) ) ) );
      const real brickY( 0.5*brickX );
      const real brickZ( 0.4*brickX );


      ////////////////////
      // Material setup

      MaterialID bodyMaterial = createMaterial( "body", density, restitution, friction, friction );

      ...
   \endcode

 * The next step in the setup of our parallel simulation is the setup of the individual MPI
 * processes. In this section it becomes apparent that we are in fact using several processes
 * that need to be configured individually. For this we are using the \b pe_EXCLUSIVE_SECTION
 * to do some initializations exclusively on one particular MPI process.

   \code
      ...

      // The pe_EXCLUSIVE_SECTION introduces a section that is only executed by exactly one
      // process. For instance, the first of the following sections is executed exclusively
      // by process 0. All other processes are skipping this section entirely. Within the
      // exclusive section, process 0 is connecting to the three other processes according
      // to its location in the lower-left. In order to connect to a remote MPI process, the
      // connect() functions have to be used.
      // Let's consider the first of these functions, which connects process 0 to process 1.
      // The first parameter specifies the rank of the remote MPI process. The next parameter
      // specifies the space occupied by process 1 and thus implicitly the boundary between
      // the local and the remote process. From the point of view of process 0, process 1
      // occupies a complete half space, defined by the boundary plane normal (1,0,0) and the
      // displacement of the boundary plane from the origin of the global coordinate system.
      // Since in this particular example the global origin lies on the process boundary, the
      // displacement is always zero. Note that the boundary normal is always pointing towards
      // the remote MPI process!
      // The connection to process 2 can be defined exactly like the connection to process 1.
      // However, in this case we are using the convenient shortcut to the previous connect()
      // function, that can be used in case a remote process occupies a complete half space.
      // The first parameter again specifies the rank of the remote MPI process, the next three
      // parameters specify the boundary plane normal and the last parameter the displacement
      // of the boundary plane.
      // Connecting process 3 is slightly more complicated, since it is located to the upper
      // right from the point of view of process 0. In order to correctly define the space
      // occupied by this process and to specify the boundary between process 0 and process 3,
      // we have to intersect two half spaces. This is done by the intersect() function. This
      // function gets the two half spaces that have to be intersected as first and second
      // parameters. The resulting space is passed to the connect() function that establishes
      // the connection to the remote process 3.
      // The same setup is performed for all four processes, each connecting to the three other
      // MPI processes according to its location. For convenience, we are using the shortcut
      // version of the connect() function wherever possible.
      pe_EXCLUSIVE_SECTION( 0 ) {
         connect( 1, HalfSpace( 1.0, 0.0, 0.0, 0.0 ) );
         connect( 2, 0.0, 1.0, 0.0, 0.0 );
         connect( 3, intersect( HalfSpace( 1.0, 0.0, 0.0, 0.0 ),
                                HalfSpace( 0.0, 1.0, 0.0, 0.0 ) ) );
      }
      pe_EXCLUSIVE_SECTION( 1 ) {
         connect( 0, -1.0, 0.0, 0.0, 0.0 );
         connect( 2, intersect( HalfSpace( -1.0, 0.0, 0.0, 0.0 ),
                                HalfSpace(  0.0, 1.0, 0.0, 0.0 ) ) );
         connect( 3,  0.0, 1.0, 0.0, 0.0 );
      }
      pe_EXCLUSIVE_SECTION( 2 ) {
         connect( 0, 0.0, -1.0, 0.0, 0.0 );
         connect( 1, intersect( HalfSpace( 1.0,  0.0, 0.0, 0.0 ),
                                HalfSpace( 0.0, -1.0, 0.0, 0.0 ) ) );
         connect( 3, 1.0,  0.0, 0.0, 0.0 );
      }
      pe_EXCLUSIVE_SECTION( 3 ) {
         connect( 0, intersect( HalfSpace( -1.0,  0.0, 0.0, 0.0 ),
                                HalfSpace(  0.0, -1.0, 0.0, 0.0 ) ) );
         connect( 1,  0.0, -1.0, 0.0, 0.0 );
         connect( 2, -1.0,  0.0, 0.0, 0.0 );
      }

      // Every single connect function creates a communication channel to a remote MPI process.
      // All these connections are managed by the MPI system. This class offers all necessary
      // functionality to check the total number of MPI processes, the rank of the local process,
      // and to set the MPI communicator or the message tag for the interal MPI communication.
      // Similar to the theWorld() function, the theMPISystem() function returns a handle to
      // the MPI system.
      MPISystemID mpiSystem = theMPISystem();

      // The following function is a convenient way to check the setup of the MPI processes.
      // This function is able to detect missing connections (i.e. single-sided connections)
      // and to a small extend boundary mismatches. Note however, that this function may NOT
      // appear inside an exclusive section, because in this function it is assumed that all
      // MPI processes are involved in the setup check. In case it is used inside an exclusive
      // section, an exception is thrown to indicate the error.
      mpiSystem->checkProcesses();

      ...
   \endcode

 * The following part of the main() function deals with the setup of the rigid bodies. For a
 * MPI parallel simulation, there are two important differences to a non-parallel simulation:
 *
 *  -# finite rigid bodies may only be created if their center of mass is inside the process
 *     boundaries
 *  -# infinite rigid bodies have to be created on all MPI processes
 *
 * Whereas the setup of infinite rigid bodies (as for instance planes) does hardly change in
 * comparison to a non-parallel simulation, the setup of finite rigid bodies (e.g. spheres,
 * boxes, capsules, ...) changes considerably. As an example, on process 0 it is only possible
 * to create finite rigid bodies whose center of mass are inside the lower-left quarter of the
 * simulation domain. The attempt to create a rigid body outside the process boundaries (so
 * logically in the domain of a remote MPI process) results in an exception.\n
 * There is a simple reasoning behind the first rule: each rigid body in the simulation should
 * be represented exactly once, i.e. a particular rigid body should be created on exactly one
 * MPI process. Otherwise it is not possible to distinguish between the fact that the same body
 * was created on several processes and two different rigid bodies that were just created in
 * the same location. Note however, that it is allowed to arbitrarily translate/rotate a rigid
 * body after the creation, i.e. it is even allowed to move the center of mass of the rigid
 * body into other processes.\n
 * The second rule stems from the fact that infinite rigid bodies don't move and are therefore
 * not considered during the inter-process MPI communication. More specifically, the programmer
 * is responsible to create an infinite rigid body on all processes that contain the infinite
 * body and the body should be translated/rotated equally on all processes.\n
 * These rules are also the reason that the process setup is performed before the setup of the
 * rigid bodies. Note however that is also possible to create the rigid bodies first and then
 * do the process setup. However, in this case it is the responsibility of the programmer to
 * assure that every finite rigid body is created only on one process. This may for example be
 * achieved by creating all bodies on one process and synchronizing afterwards (see below).
 * This however may lead to memory and performance problems for a very large number of rigid
 * bodies. So it is suggested to perform the setup as demonstrated in this tutorial.\n
 * The setup of the rigid bodies themselves is not changed in comparison to a non-parallel
 * simulation. However, in order to check if a rigid body is inside the domain of the local
 * MPI process, the World::containsPoint() function can be used to check whether the center
 * of mass of the body is contained in the local domain. The setup of the "mpiwell" example
 * is performed by every process trying to create all rigid bodies. In case the center of
 * mass of the rigid body is contained in the local domain, the rigid body is created. This
 * results in every rigid body created exactly once on only one MPI process.

   \code
      ...

      ////////////////////////////
      // Simulation world setup

      // Creating and initializing the pe simulation world
      WorldID world = theWorld();
      world->setGravity( 0.0, 0.0, -2.0 );
      world->setDamping( 0.9 );

      // Setup of the ground plane
      // The pe_GLOBAL_SECTION introduces a special section that is executed on all processes.
      // Rigid bodies created inside a global section are considered global bodies, i.e. they
      // are known on all processes. However, global bodies are always immobile, i.e. their
      // global position is always fixed. Therefore the global section should only be used
      // for large, static rigid bodies that span multiple processes, as for instance the
      // ground plane of our simulation scenario. Note that since global rigid bodies are
      // known on all processes, it is not allowed to modify these bodies within an exclusive
      // section. For example, translations and rotations can only be applied within a section
      // that is executed by all processes, but not inside an exclusive section.
      pe_GLOBAL_SECTION {
         createPlane( 0, 0.0, 0.0, 1.0, 0.0, granite );
      }

      // Setup of the well
      Vec3 gpos;
      Quat rot;

      for( size_t h=0; h<H; ++h ) {
         rot  = ( h % 2 )?( fullrotation ):( halfrotation );
         gpos = rot.rotate( Vec3( 0.0, R+real(0.5)*brickY, ( h+real(0.5) ) * brickZ ) );
         for( size_t i=0; i<B; ++i )
         {
            // The containsPoint() function checks whether the given coordinate is contained
            // in the local domain. In case it is, the rigid body can be safely created.
            if( world->containsPoint( gpos ) ) {
               BoxID brick = createBox( 0, gpos, brickX, brickY, brickZ, granite );
               brick->rotate( rot );
               brick->setFixed();
            }
            gpos = fullrotation.rotate( gpos );
            rot  = fullrotation * rot;
         }
      }

      // Setup of the rigid bodies
      const real   dx  ( real(2)*sphereRadiusMax );
      const size_t X   ( std::ceil( 1.8*R/dx ) );
      const real   xmin( real(0.5)*(X-1)*dx );

      size_t count( 0 );
      unsigned int id( 0 );

      gpos.set( -xmin, -xmin, real(3)*brickZ*H );

      while( count < N )
      {
         for( size_t i=0; i<X; ++i ) {
            for( size_t j=0; j<X; ++j )
            {
               if( ( sqr(gpos[0]) + sqr(gpos[1]) ) > 0.81*R*R ) {
                  gpos[0] += dx;
                  continue;
               }

               // Again, this check is performed to make sure the rigid body is only created on one
               // MPI process. In case the test returns true, the sphere can be safely created on
               // this process.
               if( world->containsPoint( gpos ) )
               {
                  const real radius( rand<real>( sphereRadiusMin, sphereRadiusMax ) );
                  SphereID sphere = createSphere( ++id, gpos, radius, bodyMaterial );
                  sphere->setLinearVel ( rand<real>(-1.0,1.0), rand<real>(-1.0,1.0), rand<real>(-0.2,0.2) );
                  sphere->setAngularVel( rand<real>(-0.1,0.1), rand<real>(-0.1,0.1), rand<real>(-0.1,0.1) );
                  sphere->rotate( rand<real>(-PI,PI), rand<real>(-PI,PI), rand<real>(-PI,PI) );
               }

               ++count;

               if( count == N ) {
                  i = j = X;
                  break;
               }

               gpos[0] += dx;
            }

            gpos[0]  = -xmin;
            gpos[1] += dx;
         }

         gpos[1]  = -xmin;
         gpos[2] += dx;
      }

      // Due to the fact that we are dealing with rigid bodies that have a certain volume, it is
      // necessary to consider the fact that although the center of mass of a rigid body is inside
      // the local domain, the rigid body may still extend into the domain of several other remote
      // MPI processes. It is also allowed to translate a rigid body such that its center of mass
      // lies inside a remote MPI process. In order to make rigid bodies that (partially) lie inside
      // a remote process known on the remote process, it is necessary to synchronize the rigid
      // bodies among the processes. This task is performed by the World::synchronize() function.
      // This function should always be called after the rigid body setup phase or in case a rigid body
      // has some local modifications (translations, rotations, fixation, visibility,...). Note
      // however, that also this function may NOT appear inside an exclusive section, because in
      // this function it is assumed that all MPI processes are involved in the synchronization
      // process. In case it is used inside an exclusive section, an exception is thrown to indicate
      // the error.
      world->synchronize();
   \endcode

 * After the simulation setup, we are ready to start the simulation. This is done exactly as in the
 * case of the non-parallel simulation by either calling the World::run() function, or by calling
 * the World::simulationStep() function for a single step in the simulation world. During each time
 * step of the simulation, the MPI processes will communicate with each other to handle the movement
 * and the occurring collisions between the rigid bodies in parallel. However, all of this happens
 * automatically, so no special treatment has to be performed.\n
 * After the simulation, the MPI system has to be finalized via the MPI_Finalize() function as in
 * any other MPI parallel code.

   \code
      ...

      ///////////////////////////
      // Rigid body simulation

      world->run( timesteps, timestepsize );


      //////////////////////
      // MPI Finalization

      MPI_Finalize();
   }
   \endcode

 * This concludes the fourth tutorial. Hopefully this gave you an impression of how MPI parallel
 * rigid body simulations can be run with the \b pe physics engine. For more information, please
 * refer to the \ref mpi module of the engine that for example contains more details about special
 * scenarios or to specific class descriptions as for instance the pe::MPISystem class or the
 * pe::Process class.
 */
//*************************************************************************************************

#endif
