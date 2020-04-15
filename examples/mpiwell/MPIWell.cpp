//=================================================================================================
/*!
 *  \file MPIWell.cpp
 *  \brief Example file for the pe physics engine
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

#include <cmath>
#include <cstddef>
#include <cstring>
#include <iostream>
#include <pe/engine.h>
#include <pe/support.h>
using namespace pe;
using namespace pe::povray;
using namespace pe::timing;




// Assert statically that only the FFD solver is used since parameters are tuned for it.
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::FFDSolver>::Config TargetConfig;
pe_CONSTRAINT_MUST_BE_SAME_TYPE(TargetConfig, Config);




// WARNING: The MPIWell example is currently broken since the domain decomposition has not been
// updated yet.
pe_STATIC_ASSERT( false );




//=================================================================================================
//
//  POVRAY CAMERA ANIMATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Animation for the POV-Ray camera.
 *
 * This class animates the POV-Ray camera for the MPIWell example. The camera starts at a low
 * point above the well and rises in a circular motion around the well to its maximum location.
 */
class CircularMotion : public CameraAnimation
{
 public:
   //**Constructor*********************************************************************************
   explicit CircularMotion( real dz, real angle )
      : dz_( dz )
      , q_ ( Vec3(0,0,1), angle )
   {}
   //**********************************************************************************************

   //**Update functions****************************************************************************
   virtual void updateLocation( Vec3& location )
   {
      location[2] += dz_;
      location     = q_.rotate( location );
   }
   //**********************************************************************************************

 private:
   //**Member variables****************************************************************************
   real dz_;
   Quat q_;
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the well example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The mpiwell example is a round well made of several layers of fixed boxes. Particles are
 * poured into the well.
 */
int main( int argc, char** argv )
{
   ////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////
   // Simulation parameter definition

   // Timing variables
   const size_t timesteps   (  6000 );  // Total number of time steps
   const size_t visspacing  (    10 );  // Spacing between two POV-Ray images
   const real   timestepsize( 0.005 );  // Size of a single time step

   // Total number of falling rigid bodies
   const size_t N ( 3000  );

   // Common well parameters
   const bool   square( true );  // 'true' for a square well, 'false' for a round well
   const size_t B     (  40  );  // Number of bricks per level
   const size_t H     (   5  );  // Number of levels

   // Round well parameters
   // The example supports either 2, 4, 7, 19, 37, 61 or 91 MPI processes. Any other
   // number of processes will result in an error message and abort the execution.
   const real   R ( 30.0 );      // Radius of the well
   const size_t processes( 4 );  // Total number of MPI processes

   // Square well parameters
   const real L         ( 60.0 );  // Side length of the square length
   const real G         (  0.4 );  // Size of the gap between two bricks
   const real processesX(  2   );  // Number of processes in x-direction
   const real processesY(  2   );  // Number of processes in y-direction

   // Randomness
   const real probSphere ( 0.0 );  // Probability for a random sphere
   const real probBox    ( 0.0 );  // Probability for a random box
   const real probCapsule( 0.0 );  // Probability for a random capsule
   const real probTristar( 1.0 );  // Probability for a random tristar union

   // General object parameters
   const real density    ( 1.0  );  // Density of the random objects
   const real restitution( 0.16 );  // Coefficient of restitution for the random objects
   const real friction   ( 0.25 );  // Coefficient of static and dynamic friction
   const real poisson    ( 0.3  );  // Poisson's ratio of the object material
   const real young      ( 300  );  // Young's modulus of the object material
   const real stiffness  ( 1e6  );  // The stiffness in normal direction of the contact region
   const real dampingN   ( 1e5  );  // The damping coefficient in normal direction of the contact region
   const real dampingT   ( 2e5  );  // The damping coefficient in tangential direction of the contact region

   // Sphere parameters
   const real sphereRadiusMin( 0.6 );  // Minimal radius of a random sphere
   const real sphereRadiusMax( 1.2 );  // Maximal radius of a random sphere

   // Box parameters
   const real boxLengthMin( 1.5 );  // Minimal side length of a random box
   const real boxLengthMax( 2.5 );  // Maximal side length of a random box

   // Capsule settings
   const real capsuleLengthMin( 1.1 );  // Minimal length of a random capsule
   const real capsuleLengthMax( 1.9 );  // Maximal length of a random capsule
   const real capsuleRadiusMin( 0.6 );  // Minimal radius of a random capsule
   const real capsuleRadiusMax( 0.9 );  // Maximal radius of a random capsule

   // Visualization
   bool povray( true );  // Switches the POV-Ray visualization on and off

   // POV-Ray options
   const Vec3        location( -45.0, -45.0, 50.0 );  // Starting location and end height
   const Vec3        focus   (   0.0,   0.0, -5.0 );  // Focus point of the POV-Ray camera
   const std::string file    ( "./video/pic%.pov" );  // Name template for the POV-Ray files


   ////////////////////
   // Initial setups

   // Checking the probabilities
   if( probSphere + probBox + probCapsule + probTristar != 1.0 ) {
      std::cerr << pe_RED
                << "\n Invalid probabilities! Please check the probability settings!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the number of bricks per level
   if( square && B % 4 != 0 ) {
      std::cerr << pe_RED
                << "\n Invalid number of bricks per level!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;


   ///////////////////////////////
   // Configuration of the well

   // Parameters for the square well
   const real num( B/4 );
   const real hL ( L * real(0.5) );
   const real hG ( G * real(0.5) );

   // Parameters for the round well
   const real brickangle( real(2)*M_PI / B );
   const Quat fullrotation( Vec3( 0, 0, 1 ), brickangle );
   const Quat halfrotation( Vec3( 0, 0, 1 ), real(0.5)*brickangle );

   // Common parameters
   const real brickX( square ? L / ( num-real(0.5) )
                             : std::sqrt( real(2)*sq(R)*( real(1) - std::cos(brickangle) ) ) );
   const real brickY( real(0.5)*brickX );
   const real brickZ( real(0.4)*brickX );


   ////////////////////
   // Material setup

   MaterialID bodyMaterial = createMaterial( "body", density, restitution,
                                             friction, friction, poisson, young,
                                             stiffness, dampingN, dampingT );


   /////////////////////////////////
   // POV-Ray visualization setup

   WriterID pov;

   PlainTexture groundTexture( PlainTexture( ColorPigment( 0.5, 0.5, 0.5 ) ) );

   if( povray )
   {
      // Configurating the POV-Ray writer
      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->include( "settings.inc" );
      pov->setFilename( file );
      pov->setBackground( Color( 1.0, 1.0, 1.0 ) );

      // Configuring the POV-Ray camera
      CameraID camera = theCamera();
      const Vec3 tmp( location[0], location[1], real(2)*brickZ*H );
      camera->setLocation( tmp   );
      camera->setFocus   ( focus );
      camera->animate    ( CircularMotion( ( location[2]-tmp[2] )/real(timesteps),
                                           M_PI/real(timesteps) ) );
   }


   ////////////////////////////////
   // Setup of the MPI processes

   // Setup of the connections for the square well
   if( square )
   {
      const real dx( L / processesX );
      const real dy( L / processesY );

      int dims   [] = { processesX, processesY };
      int periods[] = { false, false };

      int rank;           // Rank of the neighboring process
      int center[2];      // Definition of the coordinates array 'center'
      MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

      MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
      theMPISystem()->setComm( cartcomm );
      MPI_Cart_coords( cartcomm, theMPISystem()->getRank(), 2, center );

      int west     [] = { center[0]-1, center[1]   };
      int east     [] = { center[0]+1, center[1]   };
      int south    [] = { center[0]  , center[1]-1 };
      int north    [] = { center[0]  , center[1]+1 };
      int southwest[] = { center[0]-1, center[1]-1 };
      int southeast[] = { center[0]+1, center[1]-1 };
      int northwest[] = { center[0]-1, center[1]+1 };
      int northeast[] = { center[0]+1, center[1]+1 };

      if( west[0] >= 0  ) {
         MPI_Cart_rank( cartcomm, west, &rank );
         connect( rank, Vec3( -1.0, 0.0, 0.0 ), hL-center[0]*dx );
      }
      if( east[0] < processesX ) {
         MPI_Cart_rank( cartcomm, east, &rank );
         connect( rank, Vec3( 1.0, 0.0, 0.0 ), -hL+east[0]*dx );
      }
      if( south[1] >= 0 ) {
         MPI_Cart_rank( cartcomm, south, &rank );
         connect( rank, Vec3( 0.0, -1.0, 0.0 ), hL-center[1]*dy );
      }
      if( north[1] < processesY ) {
         MPI_Cart_rank( cartcomm, north, &rank );
         connect( rank, Vec3( 0.0, 1.0, 0.0 ), -hL+north[1]*dy );
      }
      if( southwest[0] >= 0 && southwest[1] >= 0 ) {
         MPI_Cart_rank( cartcomm, southwest, &rank );
         const Vec3 gpos( -hL+center[0]*dx, -hL+center[1]*dy, 0.0 );
         connect( rank, intersect( HalfSpace( Vec3( -1.0,  0.0, 0.0 ), gpos ),
                                   HalfSpace( Vec3(  0.0, -1.0, 0.0 ), gpos ) ) );
      }
      if( southeast[0] < processesX && southeast[1] >= 0 ) {
         MPI_Cart_rank( cartcomm, southeast, &rank );
         const Vec3 gpos( -hL+east[0]*dx, -hL+center[1]*dy, 0.0 );
         connect( rank, intersect( HalfSpace( Vec3( 1.0,  0.0, 0.0 ), gpos ),
                                   HalfSpace( Vec3( 0.0, -1.0, 0.0 ), gpos ) ) );
      }
      if( northwest[0] >= 0 && northwest[1] < processesY ) {
         MPI_Cart_rank( cartcomm, northwest, &rank );
         const Vec3 gpos( -hL+center[0]*dx, -hL+north[1]*dy, 0.0 );
         connect( rank, intersect( HalfSpace( Vec3( -1.0, 0.0, 0.0 ), gpos ),
                                   HalfSpace( Vec3(  0.0, 1.0, 0.0 ), gpos ) ) );
      }
      if( northeast[0] < processesX && northeast[1] < processesY ) {
         MPI_Cart_rank( cartcomm, northeast, &rank );
         const Vec3 gpos( -hL+east[0]*dx, -hL+north[1]*dy, 0.0 );
         connect( rank, intersect( HalfSpace( Vec3( 1.0, 0.0, 0.0 ), gpos ),
                                   HalfSpace( Vec3( 0.0, 1.0, 0.0 ), gpos ) ) );
      }

      if( povray ) {
         std::ostringstream texture;
         texture << "Texture" << theMPISystem()->getRank()+1;
         pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ), groundTexture ) );
      }
   }

   // Setup of the connections for the round well
   else
   {
      /* Connecting the remote MPI processes (2 processes)
      //  0 | 1
      */
      if( processes == 2 ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            connect( 1, 1.0, 0.0, 0.0, 0.0 );
         }
         pe_EXCLUSIVE_SECTION( 1 ) {
            connect( 0, -1.0, 0.0, 0.0, 0.0 );
         }
      }

      /* Connecting the remote MPI processes (4 processes)
      //  2 | 3
      // ___|___
      //    |
      //  0 | 1
      */
      else if( processes == 4 ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            connect( 1, 1.0, 0.0, 0.0, 0.0 );
            connect( 2, 0.0, 1.0, 0.0, 0.0 );
            connect( 3, 1.0, 1.0, 0.0, 0.0 );
         }
         pe_EXCLUSIVE_SECTION( 1 ) {
            connect( 0, -1.0, 0.0, 0.0, 0.0 );
            connect( 2, -1.0, 1.0, 0.0, 0.0 );
            connect( 3,  0.0, 1.0, 0.0, 0.0 );
         }
         pe_EXCLUSIVE_SECTION( 2 ) {
            connect( 0, 0.0, -1.0, 0.0, 0.0 );
            connect( 1, 1.0, -1.0, 0.0, 0.0 );
            connect( 3, 1.0,  0.0, 0.0, 0.0 );
         }
         pe_EXCLUSIVE_SECTION( 3 ) {
            connect( 0, -1.0, -1.0, 0.0, 0.0 );
            connect( 1,  0.0, -1.0, 0.0, 0.0 );
            connect( 2, -1.0,  0.0, 0.0, 0.0 );
         }
      }

      /* Connecting the remote MPI processes (7 processes)
      //     \ 1  /
      //     \___/
      //  6  /   \ 2
      // ___/ 0  \___
      //    \    /
      //  5 \___/  3
      //    /   \
      //   /  4 \
      */
      else if( processes == 7 ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            connect( 1,  0.0     ,  1.0, 0.0,  R/3.0 );
            connect( 2,  0.866025,  0.5, 0.0,  R/3.0 );
            connect( 3,  0.866025, -0.5, 0.0,  R/3.0 );
            connect( 4,  0.0     , -1.0, 0.0,  R/3.0 );
            connect( 5, -0.866025, -0.5, 0.0,  R/3.0 );
            connect( 6, -0.866025,  0.5, 0.0,  R/3.0 );
         }
         pe_EXCLUSIVE_SECTION( 1 ) {
            connect( 0,  0.0     , -1.0, 0.0, -R/3.0 );
            connect( 2,  0.866025, -0.5, 0.0,    0.0 );
            connect( 6, -0.866025, -0.5, 0.0,    0.0 );
         }
         pe_EXCLUSIVE_SECTION( 2 ) {
            connect( 0, -0.866025, -0.5, 0.0, -R/3.0 );
            connect( 1, -0.866025,  0.5, 0.0,    0.0 );
            connect( 3,  0.0     , -1.0, 0.0,    0.0 );
         }
         pe_EXCLUSIVE_SECTION( 3 ) {
            connect( 0, -0.866025,  0.5, 0.0, -R/3.0 );
            connect( 2,  0.0     ,  1.0, 0.0,    0.0 );
            connect( 4, -0.866025, -0.5, 0.0,    0.0 );
         }
         pe_EXCLUSIVE_SECTION( 4 ) {
            connect( 0,  0.0     ,  1.0, 0.0, -R/3.0 );
            connect( 3,  0.866025,  0.5, 0.0,    0.0 );
            connect( 5, -0.866025,  0.5, 0.0,    0.0 );
         }
         pe_EXCLUSIVE_SECTION( 5 ) {
            connect( 0,  0.866025,  0.5, 0.0, -R/3.0 );
            connect( 4,  0.866025, -0.5, 0.0,    0.0 );
            connect( 6,  0.0     ,  1.0, 0.0,    0.0 );
         }
         pe_EXCLUSIVE_SECTION( 6 ) {
            connect( 0,  0.866025, -0.5, 0.0, -R/3.0 );
            connect( 1,  0.866025,  0.5, 0.0,    0.0 );
            connect( 5,  0.0     , -1.0, 0.0,    0.0 );
         }
      }

      /* Connecting the remote MPI processes (19 processes)
      //             \ 7  /
      //             \___/
      //        \ 18 /   \ 8  /
      //        \___/    \___/
      //     17 /   \ 1  /   \ 9
      //    ___/    \___/    \___
      //       \ 6  /   \ 2  /
      //    16 \___/ 0  \___/ 10
      //       /   \    /   \
      //   ___/  5 \___/  3 \___
      //      \    /   \    /
      //   15 \___/  4 \___/ 11
      //      /   \    /   \
      //     / 14 \___/ 12 \
      //          /   \
      //         / 13 \
      */
      else if( processes == 19 ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            connect(  1,  0.0     ,  1.0, 0.0,  0.2*R );
            connect(  2,  0.866025,  0.5, 0.0,  0.2*R );
            connect(  3,  0.866025, -0.5, 0.0,  0.2*R );
            connect(  4,  0.0     , -1.0, 0.0,  0.2*R );
            connect(  5, -0.866025, -0.5, 0.0,  0.2*R );
            connect(  6, -0.866025,  0.5, 0.0,  0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 1 ) {
            connect(  0,  0.0     , -1.0, 0.0, -0.2*R );
            connect(  2,  0.866025, -0.5, 0.0,  0.0 );
            connect(  6, -0.866025, -0.5, 0.0,  0.0 );
            connect(  7,  0.0     ,  1.0, 0.0,  0.6*R );
            connect(  8,  0.866025,  0.5, 0.0,  0.4*R );
            connect( 18, -0.866025,  0.5, 0.0,  0.4*R );
         }
         pe_EXCLUSIVE_SECTION( 2 ) {
            connect(  0, -0.866025, -0.5, 0.0, -0.2*R );
            connect(  1, -0.866025,  0.5, 0.0,    0.0 );
            connect(  3,  0.0     , -1.0, 0.0,    0.0 );
            connect(  8,  0.0     ,  1.0, 0.0,  0.4*R );
            connect(  9,  0.866025,  0.5, 0.0,  0.6*R );
            connect( 10,  0.866025, -0.5, 0.0,  0.4*R );
         }
         pe_EXCLUSIVE_SECTION( 3 ) {
            connect(  0, -0.866025,  0.5, 0.0, -0.2*R );
            connect(  2,  0.0     ,  1.0, 0.0,    0.0 );
            connect(  4, -0.866025, -0.5, 0.0,    0.0 );
            connect( 10,  0.866025,  0.5, 0.0,  0.4*R );
            connect( 11,  0.866025, -0.5, 0.0,  0.6*R );
            connect( 12,  0.0,      -1.0, 0.0,  0.4*R );
         }
         pe_EXCLUSIVE_SECTION( 4 ) {
            connect(  0,  0.0     ,  1.0, 0.0, -R/5.0 );
            connect(  3,  0.866025,  0.5, 0.0,    0.0 );
            connect(  5, -0.866025,  0.5, 0.0,    0.0 );
            connect( 12,  0.866025, -0.5, 0.0,  0.4*R );
            connect( 13,  0.0,      -1.0, 0.0,  0.6*R );
            connect( 14, -0.866025, -0.5, 0.0,  0.4*R );
         }
         pe_EXCLUSIVE_SECTION( 5 ) {
            connect(  0,  0.866025,  0.5, 0.0, -0.2*R );
            connect(  4,  0.866025, -0.5, 0.0,    0.0 );
            connect(  6,  0.0     ,  1.0, 0.0,    0.0 );
            connect( 14,  0.0     , -1.0, 0.0,  0.4*R );
            connect( 15, -0.866025, -0.5, 0.0,  0.6*R );
            connect( 16, -0.866025,  0.5, 0.0,  0.4*R );
         }
         pe_EXCLUSIVE_SECTION( 6 ) {
            connect(  0,  0.866025, -0.5, 0.0, -0.2*R );
            connect(  1,  0.866025,  0.5, 0.0,    0.0 );
            connect(  5,  0.0     , -1.0, 0.0,    0.0 );
            connect( 16, -0.866025, -0.5, 0.0,  0.4*R );
            connect( 17, -0.866025,  0.5, 0.0,  0.6*R );
            connect( 18,  0.0     ,  1.0, 0.0,  0.4*R );
         }
         pe_EXCLUSIVE_SECTION( 7 ) {
            connect(  1,  0.0     , -1.0, 0.0, -0.6*R );
            connect(  8,  0.866025, -0.5, 0.0, -0.2*R );
            connect( 18, -0.866025, -0.5, 0.0, -0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 8 ) {
            connect(  1, -0.866025, -0.5, 0.0, -0.4*R );
            connect(  2,  0.0     , -1.0, 0.0, -0.4*R );
            connect(  7, -0.866025,  0.5, 0.0,  0.2*R );
            connect(  9,  0.866025, -0.5, 0.0,  0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 9 ) {
            connect(  2, -0.866025, -0.5, 0.0, -0.6*R );
            connect(  8, -0.866025,  0.5, 0.0, -0.2*R );
            connect( 10,  0.0     , -1.0, 0.0, -0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 10 ) {
            connect(  2, -0.866025,  0.5, 0.0, -0.4*R );
            connect(  3, -0.866025, -0.5, 0.0, -0.4*R );
            connect(  9,  0.0     ,  1.0, 0.0,  0.2*R );
            connect( 11,  0.0     , -1.0, 0.0,  0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 11 ) {
            connect(  3, -0.866025,  0.5, 0.0, -0.6*R );
            connect( 10,  0.0     ,  1.0, 0.0, -0.2*R );
            connect( 12, -0.866025, -0.5, 0.0, -0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 12 ) {
            connect(  3,  0.0     ,  1.0, 0.0, -0.4*R );
            connect(  4, -0.866025,  0.5, 0.0, -0.4*R );
            connect( 11,  0.866025,  0.5, 0.0,  0.2*R );
            connect( 13, -0.866025, -0.5, 0.0,  0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 13 ) {
            connect(  4,  0.0     ,  1.0, 0.0, -0.6*R );
            connect( 12,  0.866025,  0.5, 0.0, -0.2*R );
            connect( 14, -0.866025,  0.5, 0.0, -0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 14 ) {
            connect(  4,  0.866025,  0.5, 0.0, -0.4*R );
            connect(  5,  0.0     ,  1.0, 0.0, -0.4*R );
            connect( 13,  0.866025, -0.5, 0.0,  0.2*R );
            connect( 15, -0.866025,  0.5, 0.0,  0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 15 ) {
            connect(  5,  0.866025,  0.5, 0.0, -0.6*R );
            connect( 14,  0.866025, -0.5, 0.0, -0.2*R );
            connect( 16,  0.0     ,  1.0, 0.0, -0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 16 ) {
            connect(  5,  0.866025, -0.5, 0.0, -0.4*R );
            connect(  6,  0.866025,  0.5, 0.0, -0.4*R );
            connect( 15,  0.0     , -1.0, 0.0,  0.2*R );
            connect( 17,  0.0     ,  1.0, 0.0,  0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 17 ) {
            connect(  6,  0.866025, -0.5, 0.0, -0.6*R );
            connect( 16,  0.0     , -1.0, 0.0, -0.2*R );
            connect( 18,  0.866025,  0.5, 0.0, -0.2*R );
         }
         pe_EXCLUSIVE_SECTION( 18 ) {
            connect(  1,  0.866025, -0.5, 0.0, -0.4*R );
            connect(  6,  0.0     , -1.0, 0.0, -0.4*R );
            connect(  7,  0.866025,  0.5, 0.0,  0.2*R );
            connect( 17, -0.866025, -0.5, 0.0,  0.2*R );
         }
      }

      /* Connecting the remote MPI processes (37 processes)
      //                   \ 19 /
      //                   \___/
      //              \ 36 /   \ 20 /
      //              \___/    \___/
      //         \ 35 /   \ 7  /   \ 21 /
      //         \___/    \___/    \___/
      //      34 /   \ 18 /   \ 8  /   \ 22
      //     ___/    \___/    \___/    \___
      //        \ 17 /   \ 1  /   \ 9  /
      //        \___/    \___/    \___/
      //     33 /   \ 6  /   \ 2  /   \ 23
      //    ___/ 16 \___/ 0  \___/ 10 \___
      //       \    /   \    /   \    /
      //       \___/  5 \___/  3 \___/
      //    32 /   \    /   \    /   \ 24
      //   ___/ 15 \___/  4 \___/ 11 \___
      //      \    /   \    /   \    /
      //   31 \___/ 14 \___/ 12 \___/ 25
      //      /   \    /   \    /   \
      //     / 30 \___/ 13 \___/ 26 \
      //          /   \    /   \
      //         / 29 \___/ 27 \
      //              /   \
      //             / 28 \
      */
      else if( processes == 37 ) {
         const real d[] = { 0.15*R, 0.3*R, 0.45*R, 0.6*R, 0.75*R };

         pe_EXCLUSIVE_SECTION( 0 ) {
            connect(  1,  0.0     ,  1.0, 0.0,  d[0] );
            connect(  2,  0.866025,  0.5, 0.0,  d[0] );
            connect(  3,  0.866025, -0.5, 0.0,  d[0] );
            connect(  4,  0.0     , -1.0, 0.0,  d[0] );
            connect(  5, -0.866025, -0.5, 0.0,  d[0] );
            connect(  6, -0.866025,  0.5, 0.0,  d[0] );
         }
         pe_EXCLUSIVE_SECTION( 1 ) {
            connect(  0,  0.0     , -1.0, 0.0, -d[0] );
            connect(  2,  0.866025, -0.5, 0.0,  0.0  );
            connect(  6, -0.866025, -0.5, 0.0,  0.0  );
            connect(  7,  0.0     ,  1.0, 0.0,  d[2] );
            connect(  8,  0.866025,  0.5, 0.0,  d[1] );
            connect( 18, -0.866025,  0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 2 ) {
            connect(  0, -0.866025, -0.5, 0.0, -d[0] );
            connect(  1, -0.866025,  0.5, 0.0,  0.0  );
            connect(  3,  0.0     , -1.0, 0.0,  0.0  );
            connect(  8,  0.0     ,  1.0, 0.0,  d[1] );
            connect(  9,  0.866025,  0.5, 0.0,  d[2] );
            connect( 10,  0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 3 ) {
            connect(  0, -0.866025,  0.5, 0.0, -d[0] );
            connect(  2,  0.0     ,  1.0, 0.0,  0.0  );
            connect(  4, -0.866025, -0.5, 0.0,  0.0  );
            connect( 10,  0.866025,  0.5, 0.0,  d[1] );
            connect( 11,  0.866025, -0.5, 0.0,  d[2] );
            connect( 12,  0.0,      -1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 4 ) {
            connect(  0,  0.0     ,  1.0, 0.0, -d[0] );
            connect(  3,  0.866025,  0.5, 0.0,  0.0  );
            connect(  5, -0.866025,  0.5, 0.0,  0.0  );
            connect( 12,  0.866025, -0.5, 0.0,  d[1] );
            connect( 13,  0.0,      -1.0, 0.0,  d[2] );
            connect( 14, -0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 5 ) {
            connect(  0,  0.866025,  0.5, 0.0, -d[0] );
            connect(  4,  0.866025, -0.5, 0.0,  0.0  );
            connect(  6,  0.0     ,  1.0, 0.0,  0.0  );
            connect( 14,  0.0     , -1.0, 0.0,  d[1] );
            connect( 15, -0.866025, -0.5, 0.0,  d[2] );
            connect( 16, -0.866025,  0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 6 ) {
            connect(  0,  0.866025, -0.5, 0.0, -d[0] );
            connect(  1,  0.866025,  0.5, 0.0,  0.0  );
            connect(  5,  0.0     , -1.0, 0.0,  0.0  );
            connect( 16, -0.866025, -0.5, 0.0,  d[1] );
            connect( 17, -0.866025,  0.5, 0.0,  d[2] );
            connect( 18,  0.0     ,  1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 7 ) {
            connect(  1,  0.0     , -1.0, 0.0, -d[2] );
            connect(  8,  0.866025, -0.5, 0.0, -d[0] );
            connect( 18, -0.866025, -0.5, 0.0, -d[0] );
            connect( 19,  0.0     ,  1.0, 0.0,  d[4] );
            connect( 20,  0.866025,  0.5, 0.0,  d[2] );
            connect( 36, -0.866025,  0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 8 ) {
            connect(  1, -0.866025, -0.5, 0.0, -d[1] );
            connect(  2,  0.0     , -1.0, 0.0, -d[1] );
            connect(  7, -0.866025,  0.5, 0.0,  d[0] );
            connect(  9,  0.866025, -0.5, 0.0,  d[0] );
            connect( 20,  0.0     ,  1.0, 0.0,  d[3] );
            connect( 21,  0.866025,  0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 9 ) {
            connect(  2, -0.866025, -0.5, 0.0, -d[2] );
            connect(  8, -0.866025,  0.5, 0.0, -d[0] );
            connect( 10,  0.0     , -1.0, 0.0, -d[0] );
            connect( 21,  0.0     ,  1.0, 0.0,  d[2] );
            connect( 22,  0.866025,  0.5, 0.0,  d[4] );
            connect( 23,  0.866025, -0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 10 ) {
            connect(  2, -0.866025,  0.5, 0.0, -d[1] );
            connect(  3, -0.866025, -0.5, 0.0, -d[1] );
            connect(  9,  0.0     ,  1.0, 0.0,  d[0] );
            connect( 11,  0.0     , -1.0, 0.0,  d[0] );
            connect( 23,  0.866025,  0.5, 0.0,  d[3] );
            connect( 24,  0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 11 ) {
            connect(  3, -0.866025,  0.5, 0.0, -d[2] );
            connect( 10,  0.0     ,  1.0, 0.0, -d[0] );
            connect( 12, -0.866025, -0.5, 0.0, -d[0] );
            connect( 24,  0.866025,  0.5, 0.0,  d[2] );
            connect( 25,  0.866025, -0.5, 0.0,  d[4] );
            connect( 26,  0.0     , -1.0, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 12 ) {
            connect(  3,  0.0     ,  1.0, 0.0, -d[1] );
            connect(  4, -0.866025,  0.5, 0.0, -d[1] );
            connect( 11,  0.866025,  0.5, 0.0,  d[0] );
            connect( 13, -0.866025, -0.5, 0.0,  d[0] );
            connect( 26,  0.866025, -0.5, 0.0,  d[3] );
            connect( 27,  0.0     , -1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 13 ) {
            connect(  4,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 12,  0.866025,  0.5, 0.0, -d[0] );
            connect( 14, -0.866025,  0.5, 0.0, -d[0] );
            connect( 27,  0.866025, -0.5, 0.0,  d[2] );
            connect( 28,  0.0     , -1.0, 0.0,  d[4] );
            connect( 29, -0.866025, -0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 14 ) {
            connect(  4,  0.866025,  0.5, 0.0, -d[1] );
            connect(  5,  0.0     ,  1.0, 0.0, -d[1] );
            connect( 13,  0.866025, -0.5, 0.0,  d[0] );
            connect( 15, -0.866025,  0.5, 0.0,  d[0] );
            connect( 29,  0.0     , -1.0, 0.0,  d[3] );
            connect( 30, -0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 15 ) {
            connect(  5,  0.866025,  0.5, 0.0, -d[2] );
            connect( 14,  0.866025, -0.5, 0.0, -d[0] );
            connect( 16,  0.0     ,  1.0, 0.0, -d[0] );
            connect( 30,  0.0     , -1.0, 0.0,  d[2] );
            connect( 31, -0.866025, -0.5, 0.0,  d[4] );
            connect( 32, -0.866025,  0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 16 ) {
            connect(  5,  0.866025, -0.5, 0.0, -d[1] );
            connect(  6,  0.866025,  0.5, 0.0, -d[1] );
            connect( 15,  0.0     , -1.0, 0.0,  d[0] );
            connect( 17,  0.0     ,  1.0, 0.0,  d[0] );
            connect( 32, -0.866025, -0.5, 0.0,  d[3] );
            connect( 33, -0.866025,  0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 17 ) {
            connect(  6,  0.866025, -0.5, 0.0, -d[2] );
            connect( 16,  0.0     , -1.0, 0.0, -d[0] );
            connect( 18,  0.866025,  0.5, 0.0, -d[0] );
            connect( 33, -0.866025, -0.5, 0.0,  d[2] );
            connect( 34, -0.866025,  0.5, 0.0,  d[4] );
            connect( 35,  0.0     ,  1.0, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 18 ) {
            connect(  1,  0.866025, -0.5, 0.0, -d[1] );
            connect(  6,  0.0     , -1.0, 0.0, -d[1] );
            connect(  7,  0.866025,  0.5, 0.0,  d[0] );
            connect( 17, -0.866025, -0.5, 0.0,  d[0] );
            connect( 35, -0.866025,  0.5, 0.0,  d[3] );
            connect( 36,  0.0     ,  1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 19 ) {
            connect(  7,  0.0     , -1.0, 0.0, -d[4] );
            connect( 20,  0.866025, -0.5, 0.0, -d[1] );
            connect( 36, -0.866025, -0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 20 ) {
            connect(  7, -0.866025, -0.5, 0.0, -d[2] );
            connect(  8,  0.0     , -1.0, 0.0, -d[3] );
            connect( 19, -0.866025,  0.5, 0.0,  d[1] );
            connect( 21,  0.866025, -0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 21 ) {
            connect(  8, -0.866025, -0.5, 0.0, -d[3] );
            connect(  9,  0.0     , -1.0, 0.0, -d[2] );
            connect( 20, -0.866025,  0.5, 0.0,  0.0  );
            connect( 22,  0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 22 ) {
            connect(  9, -0.866025, -0.5, 0.0, -d[4] );
            connect( 21, -0.866025,  0.5, 0.0, -d[1] );
            connect( 23,  0.0     , -1.0, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 23 ) {
            connect(  9, -0.866025,  0.5, 0.0, -d[2] );
            connect( 10, -0.866025, -0.5, 0.0, -d[3] );
            connect( 22,  0.0     ,  1.0, 0.0,  d[1] );
            connect( 24,  0.0     , -1.0, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 24 ) {
            connect( 10, -0.866025,  0.5, 0.0, -d[3] );
            connect( 11, -0.866025, -0.5, 0.0, -d[2] );
            connect( 23,  0.0     ,  1.0, 0.0,  0.0  );
            connect( 25,  0.0     , -1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 25 ) {
            connect( 11, -0.866025,  0.5, 0.0, -d[4] );
            connect( 24,  0.0     ,  1.0, 0.0, -d[1] );
            connect( 26, -0.866025, -0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 26 ) {
            connect( 11,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 12, -0.866025,  0.5, 0.0, -d[3] );
            connect( 25,  0.866025,  0.5, 0.0,  d[1] );
            connect( 27, -0.866025, -0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 27 ) {
            connect( 12,  0.0     ,  1.0, 0.0, -d[3] );
            connect( 13, -0.866025,  0.5, 0.0, -d[2] );
            connect( 26,  0.866025,  0.5, 0.0,  0.0  );
            connect( 28, -0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 28 ) {
            connect( 13,  0.0     ,  1.0, 0.0, -d[4] );
            connect( 27,  0.866025,  0.5, 0.0, -d[1] );
            connect( 29, -0.866025,  0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 29 ) {
            connect( 13,  0.866025,  0.5, 0.0, -d[2] );
            connect( 14,  0.0     ,  1.0, 0.0, -d[3] );
            connect( 28,  0.866025, -0.5, 0.0,  d[1] );
            connect( 30, -0.866025,  0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 30 ) {
            connect( 14,  0.866025,  0.5, 0.0, -d[3] );
            connect( 15,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 29,  0.866025, -0.5, 0.0,  0.0  );
            connect( 31, -0.866025,  0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 31 ) {
            connect( 15,  0.866025,  0.5, 0.0, -d[4] );
            connect( 30,  0.866025, -0.5, 0.0, -d[1] );
            connect( 32,  0.0     ,  1.0, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 32 ) {
            connect( 15,  0.866025, -0.5, 0.0, -d[2] );
            connect( 16,  0.866025,  0.5, 0.0, -d[3] );
            connect( 31,  0.0     , -1.0, 0.0,  d[1] );
            connect( 33,  0.0     ,  1.0, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 33 ) {
            connect( 16,  0.866025, -0.5, 0.0, -d[3] );
            connect( 17,  0.866025,  0.5, 0.0, -d[2] );
            connect( 32,  0.0     , -1.0, 0.0,  0.0  );
            connect( 34,  0.0     ,  1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 34 ) {
            connect( 17,  0.866025, -0.5, 0.0, -d[4] );
            connect( 33,  0.0     , -1.0, 0.0, -d[1] );
            connect( 35,  0.866025,  0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 35 ) {
            connect( 17,  0.0     , -1.0, 0.0, -d[2] );
            connect( 18,  0.866025, -0.5, 0.0, -d[3] );
            connect( 34, -0.866025, -0.5, 0.0,  d[1] );
            connect( 36,  0.866025,  0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 36 ) {
            connect(  7,  0.866025, -0.5, 0.0, -d[2] );
            connect( 18,  0.0     , -1.0, 0.0, -d[3] );
            connect( 19,  0.866025,  0.5, 0.0,  d[1] );
            connect( 35, -0.866025, -0.5, 0.0,  0.0  );
         }
      }

      /* Connecting the remote MPI processes (61 processes)
      //                         \ 37 /
      //                         \___/
      //                    \ 60 /   \ 38 /
      //                    \___/    \___/
      //               \ 59 /   \ 19 /   \ 39 /
      //               \___/    \___/    \___/
      //          \ 58 /   \ 36 /   \ 20 /   \ 40 /
      //          \___/    \___/    \___/    \___/
      //       57 /   \ 35 /   \ 7  /   \ 21 /   \ 41
      //      ___/    \___/    \___/    \___/    \___
      //         \ 34 /   \ 18 /   \ 8  /   \ 22 /
      //         \___/    \___/    \___/    \___/
      //      56 /   \ 17 /   \ 1  /   \ 9  /   \ 42
      //     ___/    \___/    \___/    \___/    \___
      //        \ 33 /   \ 6  /   \ 2  /   \ 23 /
      //     55 \___/ 16 \___/ 0  \___/ 10 \___/ 43
      //        /   \    /   \    /   \    /   \
      //    ___/    \___/  5 \___/  3 \___/    \___
      //       \ 32 /   \    /   \    /   \ 24 /
      //    54 \___/ 15 \___/  4 \___/ 11 \___/ 44
      //       /   \    /   \    /   \    /   \
      //   ___/ 31 \___/ 14 \___/ 12 \___/ 25 \___
      //      \    /   \    /   \    /   \    /
      //   53 \___/ 30 \___/ 13 \___/ 26 \___/ 45
      //      /   \    /   \    /   \    /   \
      //     / 52 \___/ 29 \___/ 27 \___/ 46 \
      //          /   \    /   \    /   \
      //         / 51 \___/ 28 \___/ 47 \
      //              /   \    /   \
      //             / 50 \___/ 48 \
      //                  /   \
      //                 / 49 \
      */
      else if( processes == 61 ) {
         const real disp[] = { 0.112*R, 0.224*R, 0.336*R, 0.448*R, 0.560*R, 0.672*R, 0.784*R };

         pe_EXCLUSIVE_SECTION( 0 ) {
            connect(  1,  0.0     ,  1.0, 0.0,  disp[0] );
            connect(  2,  0.866025,  0.5, 0.0,  disp[0] );
            connect(  3,  0.866025, -0.5, 0.0,  disp[0] );
            connect(  4,  0.0     , -1.0, 0.0,  disp[0] );
            connect(  5, -0.866025, -0.5, 0.0,  disp[0] );
            connect(  6, -0.866025,  0.5, 0.0,  disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 1 ) {
            connect(  0,  0.0     , -1.0, 0.0, -disp[0] );
            connect(  2,  0.866025, -0.5, 0.0,  0.0     );
            connect(  6, -0.866025, -0.5, 0.0,  0.0     );
            connect(  7,  0.0     ,  1.0, 0.0,  disp[2] );
            connect(  8,  0.866025,  0.5, 0.0,  disp[1] );
            connect( 18, -0.866025,  0.5, 0.0,  disp[1] );
         }
         pe_EXCLUSIVE_SECTION( 2 ) {
            connect(  0, -0.866025, -0.5, 0.0, -disp[0] );
            connect(  1, -0.866025,  0.5, 0.0,  0.0     );
            connect(  3,  0.0     , -1.0, 0.0,  0.0     );
            connect(  8,  0.0     ,  1.0, 0.0,  disp[1] );
            connect(  9,  0.866025,  0.5, 0.0,  disp[2] );
            connect( 10,  0.866025, -0.5, 0.0,  disp[1] );
         }
         pe_EXCLUSIVE_SECTION( 3 ) {
            connect(  0, -0.866025,  0.5, 0.0, -disp[0] );
            connect(  2,  0.0     ,  1.0, 0.0,  0.0     );
            connect(  4, -0.866025, -0.5, 0.0,  0.0     );
            connect( 10,  0.866025,  0.5, 0.0,  disp[1] );
            connect( 11,  0.866025, -0.5, 0.0,  disp[2] );
            connect( 12,  0.0,      -1.0, 0.0,  disp[1] );
         }
         pe_EXCLUSIVE_SECTION( 4 ) {
            connect(  0,  0.0     ,  1.0, 0.0, -disp[0] );
            connect(  3,  0.866025,  0.5, 0.0,  0.0     );
            connect(  5, -0.866025,  0.5, 0.0,  0.0     );
            connect( 12,  0.866025, -0.5, 0.0,  disp[1] );
            connect( 13,  0.0,      -1.0, 0.0,  disp[2] );
            connect( 14, -0.866025, -0.5, 0.0,  disp[1] );
         }
         pe_EXCLUSIVE_SECTION( 5 ) {
            connect(  0,  0.866025,  0.5, 0.0, -disp[0] );
            connect(  4,  0.866025, -0.5, 0.0,  0.0     );
            connect(  6,  0.0     ,  1.0, 0.0,  0.0     );
            connect( 14,  0.0     , -1.0, 0.0,  disp[1] );
            connect( 15, -0.866025, -0.5, 0.0,  disp[2] );
            connect( 16, -0.866025,  0.5, 0.0,  disp[1] );
         }
         pe_EXCLUSIVE_SECTION( 6 ) {
            connect(  0,  0.866025, -0.5, 0.0, -disp[0] );
            connect(  1,  0.866025,  0.5, 0.0,  0.0     );
            connect(  5,  0.0     , -1.0, 0.0,  0.0     );
            connect( 16, -0.866025, -0.5, 0.0,  disp[1] );
            connect( 17, -0.866025,  0.5, 0.0,  disp[2] );
            connect( 18,  0.0     ,  1.0, 0.0,  disp[1] );
         }
         pe_EXCLUSIVE_SECTION( 7 ) {
            connect(  1,  0.0     , -1.0, 0.0, -disp[2] );
            connect(  8,  0.866025, -0.5, 0.0, -disp[0] );
            connect( 18, -0.866025, -0.5, 0.0, -disp[0] );
            connect( 19,  0.0     ,  1.0, 0.0,  disp[4] );
            connect( 20,  0.866025,  0.5, 0.0,  disp[2] );
            connect( 36, -0.866025,  0.5, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 8 ) {
            connect(  1, -0.866025, -0.5, 0.0, -disp[1] );
            connect(  2,  0.0     , -1.0, 0.0, -disp[1] );
            connect(  7, -0.866025,  0.5, 0.0,  disp[0] );
            connect(  9,  0.866025, -0.5, 0.0,  disp[0] );
            connect( 20,  0.0     ,  1.0, 0.0,  disp[3] );
            connect( 21,  0.866025,  0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 9 ) {
            connect(  2, -0.866025, -0.5, 0.0, -disp[2] );
            connect(  8, -0.866025,  0.5, 0.0, -disp[0] );
            connect( 10,  0.0     , -1.0, 0.0, -disp[0] );
            connect( 21,  0.0     ,  1.0, 0.0,  disp[2] );
            connect( 22,  0.866025,  0.5, 0.0,  disp[4] );
            connect( 23,  0.866025, -0.5, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 10 ) {
            connect(  2, -0.866025,  0.5, 0.0, -disp[1] );
            connect(  3, -0.866025, -0.5, 0.0, -disp[1] );
            connect(  9,  0.0     ,  1.0, 0.0,  disp[0] );
            connect( 11,  0.0     , -1.0, 0.0,  disp[0] );
            connect( 23,  0.866025,  0.5, 0.0,  disp[3] );
            connect( 24,  0.866025, -0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 11 ) {
            connect(  3, -0.866025,  0.5, 0.0, -disp[2] );
            connect( 10,  0.0     ,  1.0, 0.0, -disp[0] );
            connect( 12, -0.866025, -0.5, 0.0, -disp[0] );
            connect( 24,  0.866025,  0.5, 0.0,  disp[2] );
            connect( 25,  0.866025, -0.5, 0.0,  disp[4] );
            connect( 26,  0.0     , -1.0, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 12 ) {
            connect(  3,  0.0     ,  1.0, 0.0, -disp[1] );
            connect(  4, -0.866025,  0.5, 0.0, -disp[1] );
            connect( 11,  0.866025,  0.5, 0.0,  disp[0] );
            connect( 13, -0.866025, -0.5, 0.0,  disp[0] );
            connect( 26,  0.866025, -0.5, 0.0,  disp[3] );
            connect( 27,  0.0     , -1.0, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 13 ) {
            connect(  4,  0.0     ,  1.0, 0.0, -disp[2] );
            connect( 12,  0.866025,  0.5, 0.0, -disp[0] );
            connect( 14, -0.866025,  0.5, 0.0, -disp[0] );
            connect( 27,  0.866025, -0.5, 0.0,  disp[2] );
            connect( 28,  0.0     , -1.0, 0.0,  disp[4] );
            connect( 29, -0.866025, -0.5, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 14 ) {
            connect(  4,  0.866025,  0.5, 0.0, -disp[1] );
            connect(  5,  0.0     ,  1.0, 0.0, -disp[1] );
            connect( 13,  0.866025, -0.5, 0.0,  disp[0] );
            connect( 15, -0.866025,  0.5, 0.0,  disp[0] );
            connect( 29,  0.0     , -1.0, 0.0,  disp[3] );
            connect( 30, -0.866025, -0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 15 ) {
            connect(  5,  0.866025,  0.5, 0.0, -disp[2] );
            connect( 14,  0.866025, -0.5, 0.0, -disp[0] );
            connect( 16,  0.0     ,  1.0, 0.0, -disp[0] );
            connect( 30,  0.0     , -1.0, 0.0,  disp[2] );
            connect( 31, -0.866025, -0.5, 0.0,  disp[4] );
            connect( 32, -0.866025,  0.5, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 16 ) {
            connect(  5,  0.866025, -0.5, 0.0, -disp[1] );
            connect(  6,  0.866025,  0.5, 0.0, -disp[1] );
            connect( 15,  0.0     , -1.0, 0.0,  disp[0] );
            connect( 17,  0.0     ,  1.0, 0.0,  disp[0] );
            connect( 32, -0.866025, -0.5, 0.0,  disp[3] );
            connect( 33, -0.866025,  0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 17 ) {
            connect(  6,  0.866025, -0.5, 0.0, -disp[2] );
            connect( 16,  0.0     , -1.0, 0.0, -disp[0] );
            connect( 18,  0.866025,  0.5, 0.0, -disp[0] );
            connect( 33, -0.866025, -0.5, 0.0,  disp[2] );
            connect( 34, -0.866025,  0.5, 0.0,  disp[4] );
            connect( 35,  0.0     ,  1.0, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 18 ) {
            connect(  1,  0.866025, -0.5, 0.0, -disp[1] );
            connect(  6,  0.0     , -1.0, 0.0, -disp[1] );
            connect(  7,  0.866025,  0.5, 0.0,  disp[0] );
            connect( 17, -0.866025, -0.5, 0.0,  disp[0] );
            connect( 35, -0.866025,  0.5, 0.0,  disp[3] );
            connect( 36,  0.0     ,  1.0, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 19 ) {
            connect(  7,  0.0     , -1.0, 0.0, -disp[4] );
            connect( 20,  0.866025, -0.5, 0.0, -disp[1] );
            connect( 36, -0.866025, -0.5, 0.0, -disp[1] );
            connect( 37,  0.0     ,  1.0, 0.0,  disp[6] );
            connect( 38,  0.866025,  0.5, 0.0,  disp[3] );
            connect( 60, -0.866025,  0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 20 ) {
            connect(  7, -0.866025, -0.5, 0.0, -disp[2] );
            connect(  8,  0.0     , -1.0, 0.0, -disp[3] );
            connect( 19, -0.866025,  0.5, 0.0,  disp[1] );
            connect( 21,  0.866025, -0.5, 0.0,  0.0     );
            connect( 38,  0.0     ,  1.0, 0.0,  disp[5] );
            connect( 39,  0.866025,  0.5, 0.0,  disp[4] );
         }
         pe_EXCLUSIVE_SECTION( 21 ) {
            connect(  8, -0.866025, -0.5, 0.0, -disp[3] );
            connect(  9,  0.0     , -1.0, 0.0, -disp[2] );
            connect( 20, -0.866025,  0.5, 0.0,  0.0     );
            connect( 22,  0.866025, -0.5, 0.0,  disp[1] );
            connect( 39,  0.0     ,  1.0, 0.0,  disp[4] );
            connect( 40,  0.866025,  0.5, 0.0,  disp[5] );
         }
         pe_EXCLUSIVE_SECTION( 22 ) {
            connect(  9, -0.866025, -0.5, 0.0, -disp[4] );
            connect( 21, -0.866025,  0.5, 0.0, -disp[1] );
            connect( 23,  0.0     , -1.0, 0.0, -disp[1] );
            connect( 40,  0.0     ,  1.0, 0.0,  disp[3] );
            connect( 41,  0.866025,  0.5, 0.0,  disp[6] );
            connect( 42,  0.866025, -0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 23 ) {
            connect(  9, -0.866025,  0.5, 0.0, -disp[2] );
            connect( 10, -0.866025, -0.5, 0.0, -disp[3] );
            connect( 22,  0.0     ,  1.0, 0.0,  disp[1] );
            connect( 24,  0.0     , -1.0, 0.0,  0.0     );
            connect( 42,  0.866025,  0.5, 0.0,  disp[5] );
            connect( 43,  0.866025, -0.5, 0.0,  disp[4] );
         }
         pe_EXCLUSIVE_SECTION( 24 ) {
            connect( 10, -0.866025,  0.5, 0.0, -disp[3] );
            connect( 11, -0.866025, -0.5, 0.0, -disp[2] );
            connect( 23,  0.0     ,  1.0, 0.0,  0.0     );
            connect( 25,  0.0     , -1.0, 0.0,  disp[1] );
            connect( 43,  0.866025,  0.5, 0.0,  disp[4] );
            connect( 44,  0.866025, -0.5, 0.0,  disp[5] );
         }
         pe_EXCLUSIVE_SECTION( 25 ) {
            connect( 11, -0.866025,  0.5, 0.0, -disp[4] );
            connect( 24,  0.0     ,  1.0, 0.0, -disp[1] );
            connect( 26, -0.866025, -0.5, 0.0, -disp[1] );
            connect( 44,  0.866025,  0.5, 0.0,  disp[3] );
            connect( 45,  0.866025, -0.5, 0.0,  disp[6] );
            connect( 46,  0.0     , -1.0, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 26 ) {
            connect( 11,  0.0     ,  1.0, 0.0, -disp[2] );
            connect( 12, -0.866025,  0.5, 0.0, -disp[3] );
            connect( 25,  0.866025,  0.5, 0.0,  disp[1] );
            connect( 27, -0.866025, -0.5, 0.0,  0.0     );
            connect( 46,  0.866025, -0.5, 0.0,  disp[5] );
            connect( 47,  0.0     , -1.0, 0.0,  disp[4] );
         }
         pe_EXCLUSIVE_SECTION( 27 ) {
            connect( 12,  0.0     ,  1.0, 0.0, -disp[3] );
            connect( 13, -0.866025,  0.5, 0.0, -disp[2] );
            connect( 26,  0.866025,  0.5, 0.0,  0.0     );
            connect( 28, -0.866025, -0.5, 0.0,  disp[1] );
            connect( 47,  0.866025, -0.5, 0.0,  disp[4] );
            connect( 48,  0.0     , -1.0, 0.0,  disp[5] );
         }
         pe_EXCLUSIVE_SECTION( 28 ) {
            connect( 13,  0.0     ,  1.0, 0.0, -disp[4] );
            connect( 27,  0.866025,  0.5, 0.0, -disp[1] );
            connect( 29, -0.866025,  0.5, 0.0, -disp[1] );
            connect( 48,  0.866025, -0.5, 0.0,  disp[3] );
            connect( 49,  0.0     , -1.0, 0.0,  disp[6] );
            connect( 50, -0.866025, -0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 29 ) {
            connect( 13,  0.866025,  0.5, 0.0, -disp[2] );
            connect( 14,  0.0     ,  1.0, 0.0, -disp[3] );
            connect( 28,  0.866025, -0.5, 0.0,  disp[1] );
            connect( 30, -0.866025,  0.5, 0.0,  0.0     );
            connect( 50,  0.0     , -1.0, 0.0,  disp[5] );
            connect( 51, -0.866025, -0.5, 0.0,  disp[4] );
         }
         pe_EXCLUSIVE_SECTION( 30 ) {
            connect( 14,  0.866025,  0.5, 0.0, -disp[3] );
            connect( 15,  0.0     ,  1.0, 0.0, -disp[2] );
            connect( 29,  0.866025, -0.5, 0.0,  0.0     );
            connect( 31, -0.866025,  0.5, 0.0,  disp[1] );
            connect( 51,  0.0     , -1.0, 0.0,  disp[4] );
            connect( 52, -0.866025, -0.5, 0.0,  disp[5] );
         }
         pe_EXCLUSIVE_SECTION( 31 ) {
            connect( 15,  0.866025,  0.5, 0.0, -disp[4] );
            connect( 30,  0.866025, -0.5, 0.0, -disp[1] );
            connect( 32,  0.0     ,  1.0, 0.0, -disp[1] );
            connect( 52,  0.0     , -1.0, 0.0,  disp[3] );
            connect( 53, -0.866025, -0.5, 0.0,  disp[6] );
            connect( 54, -0.866025,  0.5, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 32 ) {
            connect( 15,  0.866025, -0.5, 0.0, -disp[2] );
            connect( 16,  0.866025,  0.5, 0.0, -disp[3] );
            connect( 31,  0.0     , -1.0, 0.0,  disp[1] );
            connect( 33,  0.0     ,  1.0, 0.0,  0.0     );
            connect( 54, -0.866025, -0.5, 0.0,  disp[5] );
            connect( 55, -0.866025,  0.5, 0.0,  disp[4] );
         }
         pe_EXCLUSIVE_SECTION( 33 ) {
            connect( 16,  0.866025, -0.5, 0.0, -disp[3] );
            connect( 17,  0.866025,  0.5, 0.0, -disp[2] );
            connect( 32,  0.0     , -1.0, 0.0,  0.0     );
            connect( 34,  0.0     ,  1.0, 0.0,  disp[1] );
            connect( 55, -0.866025, -0.5, 0.0,  disp[4] );
            connect( 56, -0.866025,  0.5, 0.0,  disp[5] );
         }
         pe_EXCLUSIVE_SECTION( 34 ) {
            connect( 17,  0.866025, -0.5, 0.0, -disp[4] );
            connect( 33,  0.0     , -1.0, 0.0, -disp[1] );
            connect( 35,  0.866025,  0.5, 0.0, -disp[1] );
            connect( 56, -0.866025, -0.5, 0.0,  disp[3] );
            connect( 57, -0.866025,  0.5, 0.0,  disp[6] );
            connect( 58,  0.0     ,  1.0, 0.0,  disp[3] );
         }
         pe_EXCLUSIVE_SECTION( 35 ) {
            connect( 17,  0.0     , -1.0, 0.0, -disp[2] );
            connect( 18,  0.866025, -0.5, 0.0, -disp[3] );
            connect( 34, -0.866025, -0.5, 0.0,  disp[1] );
            connect( 36,  0.866025,  0.5, 0.0,  0.0     );
            connect( 58, -0.866025,  0.5, 0.0,  disp[5] );
            connect( 59,  0.0     ,  1.0, 0.0,  disp[4] );
         }
         pe_EXCLUSIVE_SECTION( 36 ) {
            connect(  7,  0.866025, -0.5, 0.0, -disp[2] );
            connect( 18,  0.0     , -1.0, 0.0, -disp[3] );
            connect( 19,  0.866025,  0.5, 0.0,  disp[1] );
            connect( 35, -0.866025, -0.5, 0.0,  0.0     );
            connect( 59, -0.866025,  0.5, 0.0,  disp[4] );
            connect( 60,  0.0     ,  1.0, 0.0,  disp[5] );
         }
         pe_EXCLUSIVE_SECTION( 37 ) {
            connect( 19,  0.0     , -1.0, 0.0, -disp[6] );
            connect( 38,  0.866025, -0.5, 0.0, -disp[2] );
            connect( 60, -0.866025, -0.5, 0.0, -disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 38 ) {
            connect( 19, -0.866025, -0.5, 0.0, -disp[3] );
            connect( 20,  0.0     , -1.0, 0.0, -disp[5] );
            connect( 37, -0.866025,  0.5, 0.0,  disp[2] );
            connect( 39,  0.866025, -0.5, 0.0, -disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 39 ) {
            connect( 20, -0.866025, -0.5, 0.0, -disp[4] );
            connect( 21,  0.0     , -1.0, 0.0, -disp[4] );
            connect( 38, -0.866025,  0.5, 0.0,  disp[0] );
            connect( 40,  0.866025, -0.5, 0.0,  disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 40 ) {
            connect( 21, -0.866025, -0.5, 0.0, -disp[5] );
            connect( 22,  0.0     , -1.0, 0.0, -disp[3] );
            connect( 39, -0.866025,  0.5, 0.0, -disp[0] );
            connect( 41,  0.866025, -0.5, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 41 ) {
            connect( 22, -0.866025, -0.5, 0.0, -disp[6] );
            connect( 40, -0.866025,  0.5, 0.0, -disp[2] );
            connect( 42,  0.0     , -1.0, 0.0, -disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 42 ) {
            connect( 22, -0.866025,  0.5, 0.0, -disp[3] );
            connect( 23, -0.866025, -0.5, 0.0, -disp[5] );
            connect( 41,  0.0     ,  1.0, 0.0,  disp[2] );
            connect( 43,  0.0     , -1.0, 0.0, -disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 43 ) {
            connect( 23, -0.866025,  0.5, 0.0, -disp[4] );
            connect( 24, -0.866025, -0.5, 0.0, -disp[4] );
            connect( 42,  0.0     ,  1.0, 0.0,  disp[0] );
            connect( 44,  0.0     , -1.0, 0.0,  disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 44 ) {
            connect( 24, -0.866025,  0.5, 0.0, -disp[5] );
            connect( 25, -0.866025, -0.5, 0.0, -disp[3] );
            connect( 43,  0.0     ,  1.0, 0.0, -disp[0] );
            connect( 45,  0.0     , -1.0, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 45 ) {
            connect( 25, -0.866025,  0.5, 0.0, -disp[6] );
            connect( 44,  0.0     ,  1.0, 0.0, -disp[2] );
            connect( 46, -0.866025, -0.5, 0.0, -disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 46 ) {
            connect( 25,  0.0     ,  1.0, 0.0, -disp[3] );
            connect( 26, -0.866025,  0.5, 0.0, -disp[5] );
            connect( 45,  0.866025,  0.5, 0.0,  disp[2] );
            connect( 47, -0.866025, -0.5, 0.0, -disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 47 ) {
            connect( 26,  0.0     ,  1.0, 0.0, -disp[4] );
            connect( 27, -0.866025,  0.5, 0.0, -disp[4] );
            connect( 46,  0.866025,  0.5, 0.0,  disp[0] );
            connect( 48, -0.866025, -0.5, 0.0,  disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 48 ) {
            connect( 27,  0.0     ,  1.0, 0.0, -disp[5] );
            connect( 28, -0.866025,  0.5, 0.0, -disp[3] );
            connect( 47,  0.866025,  0.5, 0.0, -disp[0] );
            connect( 49, -0.866025, -0.5, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 49 ) {
            connect( 28,  0.0     ,  1.0, 0.0, -disp[6] );
            connect( 48,  0.866025,  0.5, 0.0, -disp[2] );
            connect( 50, -0.866025,  0.5, 0.0, -disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 50 ) {
            connect( 28,  0.866025,  0.5, 0.0, -disp[3] );
            connect( 29,  0.0     ,  1.0, 0.0, -disp[5] );
            connect( 49,  0.866025, -0.5, 0.0,  disp[2] );
            connect( 51, -0.866025,  0.5, 0.0, -disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 51 ) {
            connect( 29,  0.866025,  0.5, 0.0, -disp[4] );
            connect( 30,  0.0     ,  1.0, 0.0, -disp[4] );
            connect( 50,  0.866025, -0.5, 0.0,  disp[0] );
            connect( 52, -0.866025,  0.5, 0.0,  disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 52 ) {
            connect( 30,  0.866025,  0.5, 0.0, -disp[5] );
            connect( 31,  0.0     ,  1.0, 0.0, -disp[3] );
            connect( 51,  0.866025, -0.5, 0.0, -disp[0] );
            connect( 53, -0.866025,  0.5, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 53 ) {
            connect( 31,  0.866025,  0.5, 0.0, -disp[6] );
            connect( 52,  0.866025, -0.5, 0.0, -disp[2] );
            connect( 54,  0.0     ,  1.0, 0.0, -disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 54 ) {
            connect( 31,  0.866025, -0.5, 0.0, -disp[3] );
            connect( 32,  0.866025,  0.5, 0.0, -disp[5] );
            connect( 53,  0.0     , -1.0, 0.0,  disp[2] );
            connect( 55,  0.0     ,  1.0, 0.0, -disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 55 ) {
            connect( 32,  0.866025, -0.5, 0.0, -disp[4] );
            connect( 33,  0.866025,  0.5, 0.0, -disp[4] );
            connect( 54,  0.0     , -1.0, 0.0,  disp[0] );
            connect( 56,  0.0     ,  1.0, 0.0,  disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 56 ) {
            connect( 33,  0.866025, -0.5, 0.0, -disp[5] );
            connect( 34,  0.866025,  0.5, 0.0, -disp[3] );
            connect( 55,  0.0     , -1.0, 0.0, -disp[0] );
            connect( 57,  0.0     ,  1.0, 0.0,  disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 57 ) {
            connect( 34,  0.866025, -0.5, 0.0, -disp[6] );
            connect( 56,  0.0     , -1.0, 0.0, -disp[2] );
            connect( 58,  0.866025,  0.5, 0.0, -disp[2] );
         }
         pe_EXCLUSIVE_SECTION( 58 ) {
            connect( 34,  0.0     , -1.0, 0.0, -disp[3] );
            connect( 35,  0.866025, -0.5, 0.0, -disp[5] );
            connect( 57, -0.866025, -0.5, 0.0,  disp[2] );
            connect( 59,  0.866025,  0.5, 0.0, -disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 59 ) {
            connect( 35,  0.0     , -1.0, 0.0, -disp[4] );
            connect( 36,  0.866025, -0.5, 0.0, -disp[4] );
            connect( 58, -0.866025, -0.5, 0.0,  disp[0] );
            connect( 60,  0.866025,  0.5, 0.0,  disp[0] );
         }
         pe_EXCLUSIVE_SECTION( 60 ) {
            connect( 19,  0.866025, -0.5, 0.0, -disp[3] );
            connect( 36,  0.0     , -1.0, 0.0, -disp[5] );
            connect( 37,  0.866025,  0.5, 0.0,  disp[2] );
            connect( 59, -0.866025, -0.5, 0.0, -disp[0] );
         }
      }

      /* Connecting the remote MPI processes (91 processes)
      //                               \ 61 /
      //                               \___/
      //                          \ 90 /   \ 62 /
      //                          \___/    \___/
      //                     \ 89 /   \ 37 /   \ 63 /
      //                     \___/    \___/    \___/
      //                \ 88 /   \ 60 /   \ 38 /   \ 64 /
      //                \___/    \___/    \___/    \___/
      //           \ 87 /   \ 59 /   \ 19 /   \ 39 /   \ 65 /
      //           \___/    \___/    \___/    \___/    \___/
      //        86 /   \ 58 /   \ 36 /   \ 20 /   \ 40 /   \ 66
      //       ___/    \___/    \___/    \___/    \___/    \___
      //          \ 57 /   \ 35 /   \ 7  /   \ 21 /   \ 41 /
      //          \___/    \___/    \___/    \___/    \___/
      //       85 /   \ 34 /   \ 18 /   \ 8  /   \ 22 /   \ 67
      //      ___/    \___/    \___/    \___/    \___/    \___
      //         \ 56 /   \ 17 /   \ 1  /   \ 9  /   \ 42 /
      //         \___/    \___/    \___/    \___/    \___/
      //      84 /   \ 33 /   \ 6  /   \ 2  /   \ 23 /   \ 68
      //     ___/ 55 \___/ 16 \___/ 0  \___/ 10 \___/ 43 \___
      //        \    /   \    /   \    /   \    /   \    /
      //        \___/    \___/  5 \___/  3 \___/    \___/
      //     83 /   \ 32 /   \    /   \    /   \ 24 /   \ 69
      //    ___/ 54 \___/ 15 \___/  4 \___/ 11 \___/ 44 \___
      //       \    /   \    /   \    /   \    /   \    /
      //    82 \___/ 31 \___/ 14 \___/ 12 \___/ 25 \___/ 70
      //       /   \    /   \    /   \    /   \    /   \
      //   ___/ 53 \___/ 30 \___/ 13 \___/ 26 \___/ 45 \___
      //      \    /   \    /   \    /   \    /   \    /
      //   81 \___/ 52 \___/ 29 \___/ 27 \___/ 46 \___/ 71
      //      /   \    /   \    /   \    /   \    /   \
      //     / 80 \___/ 51 \___/ 28 \___/ 47 \___/ 72 \
      //          /   \    /   \    /   \    /   \
      //         / 79 \___/ 50 \___/ 48 \___/ 73 \
      //              /   \    /   \    /   \
      //             / 78 \___/ 49 \___/ 74 \
      //                  /   \    /   \
      //                 / 77 \___/ 75 \
      //                      /   \
      //                     / 76 \
      */
      else if( processes == 91 ) {
         const real d[] = { 0.091*R, 0.182*R, 0.273*R, 0.364*R, 0.455*R, 0.546*R, 0.637*R, 0.728*R, 0.819*R };

         pe_EXCLUSIVE_SECTION( 0 ) {
            connect(  1,  0.0     ,  1.0, 0.0,  d[0] );
            connect(  2,  0.866025,  0.5, 0.0,  d[0] );
            connect(  3,  0.866025, -0.5, 0.0,  d[0] );
            connect(  4,  0.0     , -1.0, 0.0,  d[0] );
            connect(  5, -0.866025, -0.5, 0.0,  d[0] );
            connect(  6, -0.866025,  0.5, 0.0,  d[0] );
         }
         pe_EXCLUSIVE_SECTION( 1 ) {
            connect(  0,  0.0     , -1.0, 0.0, -d[0] );
            connect(  2,  0.866025, -0.5, 0.0,  0.0  );
            connect(  6, -0.866025, -0.5, 0.0,  0.0  );
            connect(  7,  0.0     ,  1.0, 0.0,  d[2] );
            connect(  8,  0.866025,  0.5, 0.0,  d[1] );
            connect( 18, -0.866025,  0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 2 ) {
            connect(  0, -0.866025, -0.5, 0.0, -d[0] );
            connect(  1, -0.866025,  0.5, 0.0,  0.0  );
            connect(  3,  0.0     , -1.0, 0.0,  0.0  );
            connect(  8,  0.0     ,  1.0, 0.0,  d[1] );
            connect(  9,  0.866025,  0.5, 0.0,  d[2] );
            connect( 10,  0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 3 ) {
            connect(  0, -0.866025,  0.5, 0.0, -d[0] );
            connect(  2,  0.0     ,  1.0, 0.0,  0.0  );
            connect(  4, -0.866025, -0.5, 0.0,  0.0  );
            connect( 10,  0.866025,  0.5, 0.0,  d[1] );
            connect( 11,  0.866025, -0.5, 0.0,  d[2] );
            connect( 12,  0.0,      -1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 4 ) {
            connect(  0,  0.0     ,  1.0, 0.0, -d[0] );
            connect(  3,  0.866025,  0.5, 0.0,  0.0  );
            connect(  5, -0.866025,  0.5, 0.0,  0.0  );
            connect( 12,  0.866025, -0.5, 0.0,  d[1] );
            connect( 13,  0.0,      -1.0, 0.0,  d[2] );
            connect( 14, -0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 5 ) {
            connect(  0,  0.866025,  0.5, 0.0, -d[0] );
            connect(  4,  0.866025, -0.5, 0.0,  0.0  );
            connect(  6,  0.0     ,  1.0, 0.0,  0.0  );
            connect( 14,  0.0     , -1.0, 0.0,  d[1] );
            connect( 15, -0.866025, -0.5, 0.0,  d[2] );
            connect( 16, -0.866025,  0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 6 ) {
            connect(  0,  0.866025, -0.5, 0.0, -d[0] );
            connect(  1,  0.866025,  0.5, 0.0,  0.0  );
            connect(  5,  0.0     , -1.0, 0.0,  0.0  );
            connect( 16, -0.866025, -0.5, 0.0,  d[1] );
            connect( 17, -0.866025,  0.5, 0.0,  d[2] );
            connect( 18,  0.0     ,  1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 7 ) {
            connect(  1,  0.0     , -1.0, 0.0, -d[2] );
            connect(  8,  0.866025, -0.5, 0.0, -d[0] );
            connect( 18, -0.866025, -0.5, 0.0, -d[0] );
            connect( 19,  0.0     ,  1.0, 0.0,  d[4] );
            connect( 20,  0.866025,  0.5, 0.0,  d[2] );
            connect( 36, -0.866025,  0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 8 ) {
            connect(  1, -0.866025, -0.5, 0.0, -d[1] );
            connect(  2,  0.0     , -1.0, 0.0, -d[1] );
            connect(  7, -0.866025,  0.5, 0.0,  d[0] );
            connect(  9,  0.866025, -0.5, 0.0,  d[0] );
            connect( 20,  0.0     ,  1.0, 0.0,  d[3] );
            connect( 21,  0.866025,  0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 9 ) {
            connect(  2, -0.866025, -0.5, 0.0, -d[2] );
            connect(  8, -0.866025,  0.5, 0.0, -d[0] );
            connect( 10,  0.0     , -1.0, 0.0, -d[0] );
            connect( 21,  0.0     ,  1.0, 0.0,  d[2] );
            connect( 22,  0.866025,  0.5, 0.0,  d[4] );
            connect( 23,  0.866025, -0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 10 ) {
            connect(  2, -0.866025,  0.5, 0.0, -d[1] );
            connect(  3, -0.866025, -0.5, 0.0, -d[1] );
            connect(  9,  0.0     ,  1.0, 0.0,  d[0] );
            connect( 11,  0.0     , -1.0, 0.0,  d[0] );
            connect( 23,  0.866025,  0.5, 0.0,  d[3] );
            connect( 24,  0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 11 ) {
            connect(  3, -0.866025,  0.5, 0.0, -d[2] );
            connect( 10,  0.0     ,  1.0, 0.0, -d[0] );
            connect( 12, -0.866025, -0.5, 0.0, -d[0] );
            connect( 24,  0.866025,  0.5, 0.0,  d[2] );
            connect( 25,  0.866025, -0.5, 0.0,  d[4] );
            connect( 26,  0.0     , -1.0, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 12 ) {
            connect(  3,  0.0     ,  1.0, 0.0, -d[1] );
            connect(  4, -0.866025,  0.5, 0.0, -d[1] );
            connect( 11,  0.866025,  0.5, 0.0,  d[0] );
            connect( 13, -0.866025, -0.5, 0.0,  d[0] );
            connect( 26,  0.866025, -0.5, 0.0,  d[3] );
            connect( 27,  0.0     , -1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 13 ) {
            connect(  4,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 12,  0.866025,  0.5, 0.0, -d[0] );
            connect( 14, -0.866025,  0.5, 0.0, -d[0] );
            connect( 27,  0.866025, -0.5, 0.0,  d[2] );
            connect( 28,  0.0     , -1.0, 0.0,  d[4] );
            connect( 29, -0.866025, -0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 14 ) {
            connect(  4,  0.866025,  0.5, 0.0, -d[1] );
            connect(  5,  0.0     ,  1.0, 0.0, -d[1] );
            connect( 13,  0.866025, -0.5, 0.0,  d[0] );
            connect( 15, -0.866025,  0.5, 0.0,  d[0] );
            connect( 29,  0.0     , -1.0, 0.0,  d[3] );
            connect( 30, -0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 15 ) {
            connect(  5,  0.866025,  0.5, 0.0, -d[2] );
            connect( 14,  0.866025, -0.5, 0.0, -d[0] );
            connect( 16,  0.0     ,  1.0, 0.0, -d[0] );
            connect( 30,  0.0     , -1.0, 0.0,  d[2] );
            connect( 31, -0.866025, -0.5, 0.0,  d[4] );
            connect( 32, -0.866025,  0.5, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 16 ) {
            connect(  5,  0.866025, -0.5, 0.0, -d[1] );
            connect(  6,  0.866025,  0.5, 0.0, -d[1] );
            connect( 15,  0.0     , -1.0, 0.0,  d[0] );
            connect( 17,  0.0     ,  1.0, 0.0,  d[0] );
            connect( 32, -0.866025, -0.5, 0.0,  d[3] );
            connect( 33, -0.866025,  0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 17 ) {
            connect(  6,  0.866025, -0.5, 0.0, -d[2] );
            connect( 16,  0.0     , -1.0, 0.0, -d[0] );
            connect( 18,  0.866025,  0.5, 0.0, -d[0] );
            connect( 33, -0.866025, -0.5, 0.0,  d[2] );
            connect( 34, -0.866025,  0.5, 0.0,  d[4] );
            connect( 35,  0.0     ,  1.0, 0.0,  d[2] );
         }
         pe_EXCLUSIVE_SECTION( 18 ) {
            connect(  1,  0.866025, -0.5, 0.0, -d[1] );
            connect(  6,  0.0     , -1.0, 0.0, -d[1] );
            connect(  7,  0.866025,  0.5, 0.0,  d[0] );
            connect( 17, -0.866025, -0.5, 0.0,  d[0] );
            connect( 35, -0.866025,  0.5, 0.0,  d[3] );
            connect( 36,  0.0     ,  1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 19 ) {
            connect(  7,  0.0     , -1.0, 0.0, -d[4] );
            connect( 20,  0.866025, -0.5, 0.0, -d[1] );
            connect( 36, -0.866025, -0.5, 0.0, -d[1] );
            connect( 37,  0.0     ,  1.0, 0.0,  d[6] );
            connect( 38,  0.866025,  0.5, 0.0,  d[3] );
            connect( 60, -0.866025,  0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 20 ) {
            connect(  7, -0.866025, -0.5, 0.0, -d[2] );
            connect(  8,  0.0     , -1.0, 0.0, -d[3] );
            connect( 19, -0.866025,  0.5, 0.0,  d[1] );
            connect( 21,  0.866025, -0.5, 0.0,  0.0  );
            connect( 38,  0.0     ,  1.0, 0.0,  d[5] );
            connect( 39,  0.866025,  0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 21 ) {
            connect(  8, -0.866025, -0.5, 0.0, -d[3] );
            connect(  9,  0.0     , -1.0, 0.0, -d[2] );
            connect( 20, -0.866025,  0.5, 0.0,  0.0  );
            connect( 22,  0.866025, -0.5, 0.0,  d[1] );
            connect( 39,  0.0     ,  1.0, 0.0,  d[4] );
            connect( 40,  0.866025,  0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 22 ) {
            connect(  9, -0.866025, -0.5, 0.0, -d[4] );
            connect( 21, -0.866025,  0.5, 0.0, -d[1] );
            connect( 23,  0.0     , -1.0, 0.0, -d[1] );
            connect( 40,  0.0     ,  1.0, 0.0,  d[3] );
            connect( 41,  0.866025,  0.5, 0.0,  d[6] );
            connect( 42,  0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 23 ) {
            connect(  9, -0.866025,  0.5, 0.0, -d[2] );
            connect( 10, -0.866025, -0.5, 0.0, -d[3] );
            connect( 22,  0.0     ,  1.0, 0.0,  d[1] );
            connect( 24,  0.0     , -1.0, 0.0,  0.0  );
            connect( 42,  0.866025,  0.5, 0.0,  d[5] );
            connect( 43,  0.866025, -0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 24 ) {
            connect( 10, -0.866025,  0.5, 0.0, -d[3] );
            connect( 11, -0.866025, -0.5, 0.0, -d[2] );
            connect( 23,  0.0     ,  1.0, 0.0,  0.0  );
            connect( 25,  0.0     , -1.0, 0.0,  d[1] );
            connect( 43,  0.866025,  0.5, 0.0,  d[4] );
            connect( 44,  0.866025, -0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 25 ) {
            connect( 11, -0.866025,  0.5, 0.0, -d[4] );
            connect( 24,  0.0     ,  1.0, 0.0, -d[1] );
            connect( 26, -0.866025, -0.5, 0.0, -d[1] );
            connect( 44,  0.866025,  0.5, 0.0,  d[3] );
            connect( 45,  0.866025, -0.5, 0.0,  d[6] );
            connect( 46,  0.0     , -1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 26 ) {
            connect( 11,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 12, -0.866025,  0.5, 0.0, -d[3] );
            connect( 25,  0.866025,  0.5, 0.0,  d[1] );
            connect( 27, -0.866025, -0.5, 0.0,  0.0  );
            connect( 46,  0.866025, -0.5, 0.0,  d[5] );
            connect( 47,  0.0     , -1.0, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 27 ) {
            connect( 12,  0.0     ,  1.0, 0.0, -d[3] );
            connect( 13, -0.866025,  0.5, 0.0, -d[2] );
            connect( 26,  0.866025,  0.5, 0.0,  0.0  );
            connect( 28, -0.866025, -0.5, 0.0,  d[1] );
            connect( 47,  0.866025, -0.5, 0.0,  d[4] );
            connect( 48,  0.0     , -1.0, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 28 ) {
            connect( 13,  0.0     ,  1.0, 0.0, -d[4] );
            connect( 27,  0.866025,  0.5, 0.0, -d[1] );
            connect( 29, -0.866025,  0.5, 0.0, -d[1] );
            connect( 48,  0.866025, -0.5, 0.0,  d[3] );
            connect( 49,  0.0     , -1.0, 0.0,  d[6] );
            connect( 50, -0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 29 ) {
            connect( 13,  0.866025,  0.5, 0.0, -d[2] );
            connect( 14,  0.0     ,  1.0, 0.0, -d[3] );
            connect( 28,  0.866025, -0.5, 0.0,  d[1] );
            connect( 30, -0.866025,  0.5, 0.0,  0.0  );
            connect( 50,  0.0     , -1.0, 0.0,  d[5] );
            connect( 51, -0.866025, -0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 30 ) {
            connect( 14,  0.866025,  0.5, 0.0, -d[3] );
            connect( 15,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 29,  0.866025, -0.5, 0.0,  0.0  );
            connect( 31, -0.866025,  0.5, 0.0,  d[1] );
            connect( 51,  0.0     , -1.0, 0.0,  d[4] );
            connect( 52, -0.866025, -0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 31 ) {
            connect( 15,  0.866025,  0.5, 0.0, -d[4] );
            connect( 30,  0.866025, -0.5, 0.0, -d[1] );
            connect( 32,  0.0     ,  1.0, 0.0, -d[1] );
            connect( 52,  0.0     , -1.0, 0.0,  d[3] );
            connect( 53, -0.866025, -0.5, 0.0,  d[6] );
            connect( 54, -0.866025,  0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 32 ) {
            connect( 15,  0.866025, -0.5, 0.0, -d[2] );
            connect( 16,  0.866025,  0.5, 0.0, -d[3] );
            connect( 31,  0.0     , -1.0, 0.0,  d[1] );
            connect( 33,  0.0     ,  1.0, 0.0,  0.0  );
            connect( 54, -0.866025, -0.5, 0.0,  d[5] );
            connect( 55, -0.866025,  0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 33 ) {
            connect( 16,  0.866025, -0.5, 0.0, -d[3] );
            connect( 17,  0.866025,  0.5, 0.0, -d[2] );
            connect( 32,  0.0     , -1.0, 0.0,  0.0  );
            connect( 34,  0.0     ,  1.0, 0.0,  d[1] );
            connect( 55, -0.866025, -0.5, 0.0,  d[4] );
            connect( 56, -0.866025,  0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 34 ) {
            connect( 17,  0.866025, -0.5, 0.0, -d[4] );
            connect( 33,  0.0     , -1.0, 0.0, -d[1] );
            connect( 35,  0.866025,  0.5, 0.0, -d[1] );
            connect( 56, -0.866025, -0.5, 0.0,  d[3] );
            connect( 57, -0.866025,  0.5, 0.0,  d[6] );
            connect( 58,  0.0     ,  1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 35 ) {
            connect( 17,  0.0     , -1.0, 0.0, -d[2] );
            connect( 18,  0.866025, -0.5, 0.0, -d[3] );
            connect( 34, -0.866025, -0.5, 0.0,  d[1] );
            connect( 36,  0.866025,  0.5, 0.0,  0.0  );
            connect( 58, -0.866025,  0.5, 0.0,  d[5] );
            connect( 59,  0.0     ,  1.0, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 36 ) {
            connect(  7,  0.866025, -0.5, 0.0, -d[2] );
            connect( 18,  0.0     , -1.0, 0.0, -d[3] );
            connect( 19,  0.866025,  0.5, 0.0,  d[1] );
            connect( 35, -0.866025, -0.5, 0.0,  0.0  );
            connect( 59, -0.866025,  0.5, 0.0,  d[4] );
            connect( 60,  0.0     ,  1.0, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 37 ) {
            connect( 19,  0.0     , -1.0, 0.0, -d[6] );
            connect( 38,  0.866025, -0.5, 0.0, -d[2] );
            connect( 60, -0.866025, -0.5, 0.0, -d[2] );
            connect( 61,  0.0     ,  1.0, 0.0,  d[8] );
            connect( 62,  0.866025,  0.5, 0.0,  d[4] );
            connect( 90, -0.866025,  0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 38 ) {
            connect( 19, -0.866025, -0.5, 0.0, -d[3] );
            connect( 20,  0.0     , -1.0, 0.0, -d[5] );
            connect( 37, -0.866025,  0.5, 0.0,  d[2] );
            connect( 39,  0.866025, -0.5, 0.0, -d[0] );
            connect( 62,  0.0     ,  1.0, 0.0,  d[7] );
            connect( 63,  0.866025,  0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 39 ) {
            connect( 20, -0.866025, -0.5, 0.0, -d[4] );
            connect( 21,  0.0     , -1.0, 0.0, -d[4] );
            connect( 38, -0.866025,  0.5, 0.0,  d[0] );
            connect( 40,  0.866025, -0.5, 0.0,  d[0] );
            connect( 63,  0.0     ,  1.0, 0.0,  d[6] );
            connect( 64,  0.866025,  0.5, 0.0,  d[6] );
         }
         pe_EXCLUSIVE_SECTION( 40 ) {
            connect( 21, -0.866025, -0.5, 0.0, -d[5] );
            connect( 22,  0.0     , -1.0, 0.0, -d[3] );
            connect( 39, -0.866025,  0.5, 0.0, -d[0] );
            connect( 41,  0.866025, -0.5, 0.0,  d[2] );
            connect( 64,  0.0     ,  1.0, 0.0,  d[5] );
            connect( 65,  0.866025,  0.5, 0.0,  d[7] );
         }
         pe_EXCLUSIVE_SECTION( 41 ) {
            connect( 22, -0.866025, -0.5, 0.0, -d[6] );
            connect( 40, -0.866025,  0.5, 0.0, -d[2] );
            connect( 42,  0.0     , -1.0, 0.0, -d[2] );
            connect( 65,  0.0     ,  1.0, 0.0,  d[4] );
            connect( 66,  0.866025,  0.5, 0.0,  d[8] );
            connect( 67,  0.866025, -0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 42 ) {
            connect( 22, -0.866025,  0.5, 0.0, -d[3] );
            connect( 23, -0.866025, -0.5, 0.0, -d[5] );
            connect( 41,  0.0     ,  1.0, 0.0,  d[2] );
            connect( 43,  0.0     , -1.0, 0.0, -d[0] );
            connect( 67,  0.866025,  0.5, 0.0,  d[7] );
            connect( 68,  0.866025, -0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 43 ) {
            connect( 23, -0.866025,  0.5, 0.0, -d[4] );
            connect( 24, -0.866025, -0.5, 0.0, -d[4] );
            connect( 42,  0.0     ,  1.0, 0.0,  d[0] );
            connect( 44,  0.0     , -1.0, 0.0,  d[0] );
            connect( 68,  0.866025,  0.5, 0.0,  d[6] );
            connect( 69,  0.866025, -0.5, 0.0,  d[6] );
         }
         pe_EXCLUSIVE_SECTION( 44 ) {
            connect( 24, -0.866025,  0.5, 0.0, -d[5] );
            connect( 25, -0.866025, -0.5, 0.0, -d[3] );
            connect( 43,  0.0     ,  1.0, 0.0, -d[0] );
            connect( 45,  0.0     , -1.0, 0.0,  d[2] );
            connect( 69,  0.866025,  0.5, 0.0,  d[5] );
            connect( 70,  0.866025, -0.5, 0.0,  d[7] );
         }
         pe_EXCLUSIVE_SECTION( 45 ) {
            connect( 25, -0.866025,  0.5, 0.0, -d[6] );
            connect( 44,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 46, -0.866025, -0.5, 0.0, -d[2] );
            connect( 70,  0.866025,  0.5, 0.0,  d[4] );
            connect( 71,  0.866025, -0.5, 0.0,  d[8] );
            connect( 72,  0.0     , -1.0, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 46 ) {
            connect( 25,  0.0     ,  1.0, 0.0, -d[3] );
            connect( 26, -0.866025,  0.5, 0.0, -d[5] );
            connect( 45,  0.866025,  0.5, 0.0,  d[2] );
            connect( 47, -0.866025, -0.5, 0.0, -d[0] );
            connect( 72,  0.866025, -0.5, 0.0,  d[7] );
            connect( 73,  0.0     , -1.0, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 47 ) {
            connect( 26,  0.0     ,  1.0, 0.0, -d[4] );
            connect( 27, -0.866025,  0.5, 0.0, -d[4] );
            connect( 46,  0.866025,  0.5, 0.0,  d[0] );
            connect( 48, -0.866025, -0.5, 0.0,  d[0] );
            connect( 73,  0.866025, -0.5, 0.0,  d[6] );
            connect( 74,  0.0     , -1.0, 0.0,  d[6] );
         }
         pe_EXCLUSIVE_SECTION( 48 ) {
            connect( 27,  0.0     ,  1.0, 0.0, -d[5] );
            connect( 28, -0.866025,  0.5, 0.0, -d[3] );
            connect( 47,  0.866025,  0.5, 0.0, -d[0] );
            connect( 49, -0.866025, -0.5, 0.0,  d[2] );
            connect( 74,  0.866025, -0.5, 0.0,  d[5] );
            connect( 75,  0.0     , -1.0, 0.0,  d[7] );
         }
         pe_EXCLUSIVE_SECTION( 49 ) {
            connect( 28,  0.0     ,  1.0, 0.0, -d[6] );
            connect( 48,  0.866025,  0.5, 0.0, -d[2] );
            connect( 50, -0.866025,  0.5, 0.0, -d[2] );
            connect( 75,  0.866025, -0.5, 0.0,  d[4] );
            connect( 76,  0.0     , -1.0, 0.0,  d[8] );
            connect( 77, -0.866025, -0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 50 ) {
            connect( 28,  0.866025,  0.5, 0.0, -d[3] );
            connect( 29,  0.0     ,  1.0, 0.0, -d[5] );
            connect( 49,  0.866025, -0.5, 0.0,  d[2] );
            connect( 51, -0.866025,  0.5, 0.0, -d[0] );
            connect( 77,  0.0     , -1.0, 0.0,  d[7] );
            connect( 78, -0.866025, -0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 51 ) {
            connect( 29,  0.866025,  0.5, 0.0, -d[4] );
            connect( 30,  0.0     ,  1.0, 0.0, -d[4] );
            connect( 50,  0.866025, -0.5, 0.0,  d[0] );
            connect( 52, -0.866025,  0.5, 0.0,  d[0] );
            connect( 78,  0.0     , -1.0, 0.0,  d[6] );
            connect( 79, -0.866025, -0.5, 0.0,  d[6] );
         }
         pe_EXCLUSIVE_SECTION( 52 ) {
            connect( 30,  0.866025,  0.5, 0.0, -d[5] );
            connect( 31,  0.0     ,  1.0, 0.0, -d[3] );
            connect( 51,  0.866025, -0.5, 0.0, -d[0] );
            connect( 53, -0.866025,  0.5, 0.0,  d[2] );
            connect( 79,  0.0     , -1.0, 0.0,  d[5] );
            connect( 80, -0.866025, -0.5, 0.0,  d[7] );
         }
         pe_EXCLUSIVE_SECTION( 53 ) {
            connect( 31,  0.866025,  0.5, 0.0, -d[6] );
            connect( 52,  0.866025, -0.5, 0.0, -d[2] );
            connect( 54,  0.0     ,  1.0, 0.0, -d[2] );
            connect( 80,  0.0     , -1.0, 0.0,  d[4] );
            connect( 81, -0.866025, -0.5, 0.0,  d[8] );
            connect( 82, -0.866025,  0.5, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 54 ) {
            connect( 31,  0.866025, -0.5, 0.0, -d[3] );
            connect( 32,  0.866025,  0.5, 0.0, -d[5] );
            connect( 53,  0.0     , -1.0, 0.0,  d[2] );
            connect( 55,  0.0     ,  1.0, 0.0, -d[0] );
            connect( 82, -0.866025, -0.5, 0.0,  d[7] );
            connect( 83, -0.866025,  0.5, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 55 ) {
            connect( 32,  0.866025, -0.5, 0.0, -d[4] );
            connect( 33,  0.866025,  0.5, 0.0, -d[4] );
            connect( 54,  0.0     , -1.0, 0.0,  d[0] );
            connect( 56,  0.0     ,  1.0, 0.0,  d[0] );
            connect( 83, -0.866025, -0.5, 0.0,  d[6] );
            connect( 84, -0.866025,  0.5, 0.0,  d[6] );
         }
         pe_EXCLUSIVE_SECTION( 56 ) {
            connect( 33,  0.866025, -0.5, 0.0, -d[5] );
            connect( 34,  0.866025,  0.5, 0.0, -d[3] );
            connect( 55,  0.0     , -1.0, 0.0, -d[0] );
            connect( 57,  0.0     ,  1.0, 0.0,  d[2] );
            connect( 84, -0.866025, -0.5, 0.0,  d[5] );
            connect( 85, -0.866025,  0.5, 0.0,  d[7] );
         }
         pe_EXCLUSIVE_SECTION( 57 ) {
            connect( 34,  0.866025, -0.5, 0.0, -d[6] );
            connect( 56,  0.0     , -1.0, 0.0, -d[2] );
            connect( 58,  0.866025,  0.5, 0.0, -d[2] );
            connect( 85, -0.866025, -0.5, 0.0,  d[4] );
            connect( 86, -0.866025,  0.5, 0.0,  d[8] );
            connect( 87,  0.0     ,  1.0, 0.0,  d[4] );
         }
         pe_EXCLUSIVE_SECTION( 58 ) {
            connect( 34,  0.0     , -1.0, 0.0, -d[3] );
            connect( 35,  0.866025, -0.5, 0.0, -d[5] );
            connect( 57, -0.866025, -0.5, 0.0,  d[2] );
            connect( 59,  0.866025,  0.5, 0.0, -d[0] );
            connect( 87, -0.866025,  0.5, 0.0,  d[7] );
            connect( 88,  0.0     ,  1.0, 0.0,  d[5] );
         }
         pe_EXCLUSIVE_SECTION( 59 ) {
            connect( 35,  0.0     , -1.0, 0.0, -d[4] );
            connect( 36,  0.866025, -0.5, 0.0, -d[4] );
            connect( 58, -0.866025, -0.5, 0.0,  d[0] );
            connect( 60,  0.866025,  0.5, 0.0,  d[0] );
            connect( 88, -0.866025,  0.5, 0.0,  d[6] );
            connect( 89,  0.0     ,  1.0, 0.0,  d[6] );
         }
         pe_EXCLUSIVE_SECTION( 60 ) {
            connect( 19,  0.866025, -0.5, 0.0, -d[3] );
            connect( 36,  0.0     , -1.0, 0.0, -d[5] );
            connect( 37,  0.866025,  0.5, 0.0,  d[2] );
            connect( 59, -0.866025, -0.5, 0.0, -d[0] );
            connect( 89, -0.866025,  0.5, 0.0,  d[5] );
            connect( 90,  0.0     ,  1.0, 0.0,  d[7] );
         }
         pe_EXCLUSIVE_SECTION( 61 ) {
            connect( 37,  0.0     , -1.0, 0.0, -d[8] );
            connect( 62,  0.866025, -0.5, 0.0, -d[3] );
            connect( 90, -0.866025, -0.5, 0.0, -d[3] );
         }
         pe_EXCLUSIVE_SECTION( 62 ) {
            connect( 37, -0.866025, -0.5, 0.0, -d[4] );
            connect( 38,  0.0     , -1.0, 0.0, -d[7] );
            connect( 61, -0.866025,  0.5, 0.0,  d[3] );
            connect( 63,  0.866025, -0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 63 ) {
            connect( 38, -0.866025, -0.5, 0.0, -d[5] );
            connect( 39,  0.0     , -1.0, 0.0, -d[6] );
            connect( 62, -0.866025,  0.5, 0.0,  d[1] );
            connect( 64,  0.866025, -0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 64 ) {
            connect( 39, -0.866025, -0.5, 0.0, -d[6] );
            connect( 40,  0.0     , -1.0, 0.0, -d[5] );
            connect( 63, -0.866025,  0.5, 0.0,  0.0  );
            connect( 65,  0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 65 ) {
            connect( 40, -0.866025, -0.5, 0.0, -d[7] );
            connect( 41,  0.0     , -1.0, 0.0, -d[4] );
            connect( 64, -0.866025,  0.5, 0.0, -d[1] );
            connect( 66,  0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 66 ) {
            connect( 41, -0.866025, -0.5, 0.0, -d[8] );
            connect( 65, -0.866025,  0.5, 0.0, -d[3] );
            connect( 67,  0.0     , -1.0, 0.0, -d[3] );
         }
         pe_EXCLUSIVE_SECTION( 67 ) {
            connect( 41, -0.866025,  0.5, 0.0, -d[4] );
            connect( 42, -0.866025, -0.5, 0.0, -d[7] );
            connect( 66,  0.0     ,  1.0, 0.0,  d[3] );
            connect( 68,  0.0     , -1.0, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 68 ) {
            connect( 42, -0.866025,  0.5, 0.0, -d[5] );
            connect( 43, -0.866025, -0.5, 0.0, -d[6] );
            connect( 67,  0.0     ,  1.0, 0.0,  d[1] );
            connect( 69,  0.0     , -1.0, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 69 ) {
            connect( 43, -0.866025,  0.5, 0.0, -d[6] );
            connect( 44, -0.866025, -0.5, 0.0, -d[5] );
            connect( 68,  0.0     ,  1.0, 0.0,  0.0  );
            connect( 70,  0.0     , -1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 70 ) {
            connect( 44, -0.866025,  0.5, 0.0, -d[7] );
            connect( 45, -0.866025, -0.5, 0.0, -d[4] );
            connect( 69,  0.0     ,  1.0, 0.0, -d[1] );
            connect( 71,  0.0     , -1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 71 ) {
            connect( 45, -0.866025,  0.5, 0.0, -d[8] );
            connect( 70,  0.0     ,  1.0, 0.0, -d[3] );
            connect( 72, -0.866025, -0.5, 0.0, -d[3] );
         }
         pe_EXCLUSIVE_SECTION( 72 ) {
            connect( 45,  0.0     ,  1.0, 0.0, -d[4] );
            connect( 46, -0.866025,  0.5, 0.0, -d[7] );
            connect( 71,  0.866025,  0.5, 0.0,  d[3] );
            connect( 73, -0.866025, -0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 73 ) {
            connect( 46,  0.0     ,  1.0, 0.0, -d[5] );
            connect( 47, -0.866025,  0.5, 0.0, -d[6] );
            connect( 72,  0.866025,  0.5, 0.0,  d[1] );
            connect( 74, -0.866025, -0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 74 ) {
            connect( 47,  0.0     ,  1.0, 0.0, -d[6] );
            connect( 48, -0.866025,  0.5, 0.0, -d[5] );
            connect( 73,  0.866025,  0.5, 0.0,  0.0  );
            connect( 75, -0.866025, -0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 75 ) {
            connect( 48,  0.0     ,  1.0, 0.0, -d[7] );
            connect( 49, -0.866025,  0.5, 0.0, -d[4] );
            connect( 74,  0.866025,  0.5, 0.0, -d[1] );
            connect( 76, -0.866025, -0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 76 ) {
            connect( 49,  0.0     ,  1.0, 0.0, -d[8] );
            connect( 75,  0.866025,  0.5, 0.0, -d[3] );
            connect( 77, -0.866025,  0.5, 0.0, -d[3] );
         }
         pe_EXCLUSIVE_SECTION( 77 ) {
            connect( 49,  0.866025,  0.5, 0.0, -d[4] );
            connect( 50,  0.0     ,  1.0, 0.0, -d[7] );
            connect( 76,  0.866025, -0.5, 0.0,  d[3] );
            connect( 78, -0.866025,  0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 78 ) {
            connect( 50,  0.866025,  0.5, 0.0, -d[5] );
            connect( 51,  0.0     ,  1.0, 0.0, -d[6] );
            connect( 77,  0.866025, -0.5, 0.0,  d[1] );
            connect( 79, -0.866025,  0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 79 ) {
            connect( 51,  0.866025,  0.5, 0.0, -d[6] );
            connect( 52,  0.0     ,  1.0, 0.0, -d[5] );
            connect( 78,  0.866025, -0.5, 0.0,  0.0  );
            connect( 80, -0.866025,  0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 80 ) {
            connect( 52,  0.866025,  0.5, 0.0, -d[7] );
            connect( 53,  0.0     ,  1.0, 0.0, -d[4] );
            connect( 79,  0.866025, -0.5, 0.0, -d[1] );
            connect( 81, -0.866025,  0.5, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 81 ) {
            connect( 53,  0.866025,  0.5, 0.0, -d[8] );
            connect( 80,  0.866025, -0.5, 0.0, -d[3] );
            connect( 82,  0.0     ,  1.0, 0.0, -d[3] );
         }
         pe_EXCLUSIVE_SECTION( 82 ) {
            connect( 53,  0.866025, -0.5, 0.0, -d[4] );
            connect( 54,  0.866025,  0.5, 0.0, -d[7] );
            connect( 81,  0.0     , -1.0, 0.0,  d[3] );
            connect( 83,  0.0     ,  1.0, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 83 ) {
            connect( 54,  0.866025, -0.5, 0.0, -d[5] );
            connect( 55,  0.866025,  0.5, 0.0, -d[6] );
            connect( 82,  0.0     , -1.0, 0.0,  d[1] );
            connect( 84,  0.0     ,  1.0, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 84 ) {
            connect( 55,  0.866025, -0.5, 0.0, -d[6] );
            connect( 56,  0.866025,  0.5, 0.0, -d[5] );
            connect( 83,  0.0     , -1.0, 0.0,  0.0  );
            connect( 85,  0.0     ,  1.0, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 85 ) {
            connect( 56,  0.866025, -0.5, 0.0, -d[7] );
            connect( 57,  0.866025,  0.5, 0.0, -d[4] );
            connect( 84,  0.0     , -1.0, 0.0, -d[1] );
            connect( 86,  0.0     ,  1.0, 0.0,  d[3] );
         }
         pe_EXCLUSIVE_SECTION( 86 ) {
            connect( 57,  0.866025, -0.5, 0.0, -d[8] );
            connect( 85,  0.0     , -1.0, 0.0, -d[3] );
            connect( 87,  0.866025,  0.5, 0.0, -d[3] );
         }
         pe_EXCLUSIVE_SECTION( 87 ) {
            connect( 57,  0.0     , -1.0, 0.0, -d[4] );
            connect( 58,  0.866025, -0.5, 0.0, -d[7] );
            connect( 86, -0.866025, -0.5, 0.0,  d[3] );
            connect( 88,  0.866025,  0.5, 0.0, -d[1] );
         }
         pe_EXCLUSIVE_SECTION( 88 ) {
            connect( 58,  0.0     , -1.0, 0.0, -d[5] );
            connect( 59,  0.866025, -0.5, 0.0, -d[6] );
            connect( 87, -0.866025, -0.5, 0.0,  d[1] );
            connect( 89,  0.866025,  0.5, 0.0,  0.0  );
         }
         pe_EXCLUSIVE_SECTION( 89 ) {
            connect( 59,  0.0     , -1.0, 0.0, -d[6] );
            connect( 60,  0.866025, -0.5, 0.0, -d[5] );
            connect( 88, -0.866025, -0.5, 0.0,  0.0  );
            connect( 90,  0.866025,  0.5, 0.0,  d[1] );
         }
         pe_EXCLUSIVE_SECTION( 90 ) {
            connect( 37,  0.866025, -0.5, 0.0, -d[4] );
            connect( 60,  0.0     , -1.0, 0.0, -d[7] );
            connect( 61,  0.866025,  0.5, 0.0,  d[3] );
            connect( 89, -0.866025, -0.5, 0.0, -d[1] );
         }
      }

      // Handling an invalid number of processes
      else {
         std::cerr << "\n Invalid number of processes selected!\n" << std::endl;
         pe::exit( EXIT_FAILURE );
      }

      // Setting the texture policy
      if( povray ) {
         std::ostringstream texture;
         texture << "Texture" << theMPISystem()->getRank();
         pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ), groundTexture ) );
      }
   }

#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
#endif


   ////////////////////////////
   // Simulation world setup

   // Creating and initializing the pe simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );
   world->setDamping( 0.9 );

   // Setup of the ground plane
   pe_GLOBAL_SECTION {
      createPlane( 0, 0.0, 0.0, 1.0, 0.0, granite );
   }

   // Setup of the square well
   if( square )
   {
      const real start ( hL + real(0.5) * brickY );

      real start1( -hL - hG );
      real start2( -hL + real(0.5)*brickX + hG );
      real z( brickZ * real(0.5) );
      BoxID brick;
      Vec3 gpos;

      for( size_t h=0; h<H; ++h ) {
         for( size_t i=0; i<num; ++i )
         {
            // Northern wall
            gpos.set( start1+i*brickX,  start, z );
            if( world->ownsPoint( gpos ) ) {
               brick = createBox( 0, gpos, brickX-G, brickY, brickZ, granite );
               brick->setFixed( true );
            }

            // Southern wall
            gpos.set( start2+i*brickX, -start, z );
            if( world->ownsPoint( gpos ) ) {
               brick = createBox( 0, gpos, brickX-G, brickY, brickZ, granite );
               brick->setFixed( true );
            }

            // Western wall
            gpos.set( -start, start1+i*brickX, z );
            if( world->ownsPoint( gpos ) ) {
               brick = createBox( 0, gpos, brickY, brickX-G, brickZ, granite );
               brick->setFixed( true );
            }

            // Eastern wall
            gpos.set( start, start2+i*brickX, z );
            if( world->ownsPoint( gpos ) ) {
               brick = createBox( 0, gpos, brickY, brickX-G, brickZ, granite );
               brick->setFixed( true );
            }
         }
         std::swap( start1, start2 );
         z += brickZ;
      }
   }

   // Setup of the round well
   else
   {
      Vec3 gpos;
      Quat rot;

      for( size_t h=0; h<H; ++h ) {
         rot  = ( h % 2 )?( fullrotation ):( halfrotation );
         gpos = rot.rotate( Vec3( 0.0, R+real(0.5)*brickY, ( h+real(0.5) ) * brickZ ) );
         for( size_t i=0; i<B; ++i ) {
            if( world->ownsPoint( gpos ) ) {
               BoxID brick = createBox( 0, gpos, brickX, brickY, brickZ, granite );
               brick->rotate( rot );
               brick->setFixed( true );
            }
            gpos = fullrotation.rotate( gpos );
            rot  = fullrotation * rot;
         }
      }
   }

   // Setup of the rigid bodies
   const real dx( max( real(2)*sphereRadiusMax, std::sqrt(real(3))*boxLengthMax, real(2)*(capsuleLengthMax+capsuleRadiusMax) ) );
   const size_t X( square ? ( real(0.8)*L/dx ) : ( std::ceil( real(1.8)*R/dx ) ) );
   const real xmin( real(0.5)*(X-1)*dx );

   size_t count( 0 );
   unsigned int id( 0 );

   Vec3 gpos( -xmin, -xmin, real(3)*brickZ*H );

   while( count < N )
   {
      for( size_t i=0; i<X; ++i ) {
         for( size_t j=0; j<X; ++j )
         {
            if( !square && ( sq(gpos[0]) + sq(gpos[1]) ) > 0.81*R*R ) {
               gpos[0] += dx;
               continue;
            }

            if( world->ownsPoint( gpos ) )
            {
               const real object( rand<real>() );

               if( object < probSphere ) {
                  const real radius( rand<real>( sphereRadiusMin, sphereRadiusMax ) );
                  SphereID sphere = createSphere( ++id, gpos, radius, bodyMaterial );
                  sphere->setLinearVel ( rand<real>(-1.0,1.0), rand<real>(-1.0,1.0), rand<real>(-0.2,0.2) );
                  sphere->setAngularVel( rand<real>(-0.1,0.1), rand<real>(-0.1,0.1), rand<real>(-0.1,0.1) );
                  sphere->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
               }

               else if( object < probSphere+probBox ) {
                  const real lenx( rand<real>( boxLengthMin, boxLengthMax ) );
                  const real leny( rand<real>( boxLengthMin, boxLengthMax ) );
                  const real lenz( rand<real>( boxLengthMin, boxLengthMax ) );
                  BoxID box = createBox( ++id, gpos, lenx, leny, lenz, bodyMaterial );
                  box->setLinearVel ( rand<real>(-1.0,1.0), rand<real>(-1.0,1.0), rand<real>(-0.2,0.2) );
                  box->setAngularVel( rand<real>(-0.1,0.1), rand<real>(-0.1,0.1), rand<real>(-0.1,0.1) );
                  box->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
               }

               else if( object < probSphere+probBox+probCapsule ) {
                  const real radius( rand<real>( capsuleRadiusMin, capsuleRadiusMax ) );
                  const real length( rand<real>( capsuleLengthMin, capsuleLengthMax ) );
                  CapsuleID capsule = createCapsule( ++id, gpos, radius, length, bodyMaterial );
                  capsule->setLinearVel ( rand<real>(-1.0,1.0), rand<real>(-1.0,1.0), rand<real>(-0.2,0.2) );
                  capsule->setAngularVel( rand<real>(-0.1,0.1), rand<real>(-0.1,0.1), rand<real>(-0.1,0.1) );
                  capsule->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
               }

               else {
                  const real radius( rand<real>( capsuleRadiusMin, capsuleRadiusMax ) );
                  const real length( rand<real>( capsuleLengthMin, capsuleLengthMax ) );
                  UnionID tristar = createTristar( ++id, gpos, radius, length, bodyMaterial );
                  tristar->setLinearVel ( rand<real>(-1.0,1.0), rand<real>(-1.0,1.0), rand<real>(-0.2,0.2) );
                  tristar->setAngularVel( rand<real>(-0.1,0.1), rand<real>(-0.1,0.1), rand<real>(-0.1,0.1) );
                  tristar->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
               }
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

   // Synchronization of the MPI processes
   world->synchronize();

   // Output of the simulation settings
   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "--------------------------------------------------------------\n"
                << " Total number of falling rigid bodies    = " << count << "\n"
                << " Number of bricks contained in the well  = " << H*B << "\n";
      if( square ) {
         std::cout << " MPI Processes in x-direction            = " << processesX << "\n"
                   << " MPI Processes in y-direction            = " << processesY << "\n";
      }
      else {
         std::cout << " Total number of MPI processes           = " << processes << "\n";
      }
      std::cout << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << "\n"
                << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////
   // Simulation loop

   timing::WcTimer totalTime;
   timing::WcTimer simTime;

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
                << "---------------------------------------------------------" << std::endl;
   }

   totalTime.start();

   for( unsigned int timestep=0; timestep<timesteps; ++timestep )
   {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\r Simulating time step " <<  timestep+1 << "   " << std::flush;
      }

      simTime.start();
      world->simulationStep( timestepsize );
      simTime.end();
   }

   totalTime.end();

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n\n"
                << " WC-Time:  Pure simulation : " << simTime.total() << "\n"
                << "           Total time      : " << totalTime.total() << "\n";
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   //////////////////////
   // MPI Finalization

   MPI_Finalize();
}
//*************************************************************************************************
