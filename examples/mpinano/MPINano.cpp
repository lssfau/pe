//=================================================================================================
/*!
 *  \file MPINano.cpp
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

#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;




// Assert statically that only the FFD solver or a hard contact solver is used since parameters are tuned for them.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B, C) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value | pe::IsSame<A,C>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::FFDSolver>::Config TargetConfig1;
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config TargetConfig2;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig1, TargetConfig2);




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mpinano example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * Particles in a non-periodic box with non-zero initial velocities.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Simulation parameters

   const real   radius    (  0.5  );  // The radius of spherical particles
   const real   spacing   (  2.0  );  // Initial spacing inbetween two spherical particles
   const real   velocity  (  0.02 );  // Initial maximum velocity of the spherical particles

   const size_t timesteps ( 10000 );  // Total number of time steps
   const real   stepsize  (  0.01 );  // Size of a single time step

   const size_t seed      ( 12345 );  // Seed for the random number generation

         bool   povray    ( false );  // Switches the POV-Ray visualization on and off
   const size_t visspacing(    10 );  // Number of time steps inbetween two POV-Ray files

   const bool   strong    ( false );  // Compile time switch between strong and weak scaling

   const bool spheres     ( false );  // Switch between spheres and granular particles


   /////////////////////////////////////////////////////
   // Initial setups

   // Checking the ratio of the particle radius and the spacing
   if( real(2.1)*radius >= spacing ) {
      std::cerr << pe_RED << "\n Invalid particle/spacing ratio!\n\n" << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ("particles", value< std::vector<int> >()->multitoken()->required(), "number of particles in x-, y- and z-dimension")
      ("processes", value< std::vector<int> >()->multitoken()->required(), "number of processes in x-, y- and z-dimension")
   ;
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;

   const int nx( vm[ "particles" ].as< std::vector<int> >()[0] );
   const int ny( vm[ "particles" ].as< std::vector<int> >()[1] );
   const int nz( vm[ "particles" ].as< std::vector<int> >()[2] );
   const int px( vm[ "processes" ].as< std::vector<int> >()[0] );
   const int py( vm[ "processes" ].as< std::vector<int> >()[1] );
   const int pz( vm[ "processes" ].as< std::vector<int> >()[2] );

   if( nx <= 0 ) {
      std::cerr << " Invalid number of particles in x-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( ny <= 0 ) {
      std::cerr << " Invalid number of particles in y-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( nz <= 0 ) {
      std::cerr << " Invalid number of particles in z-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( px <= 0 ) {
      std::cerr << " Invalid number of processes in x-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( py <= 0 ) {
      std::cerr << " Invalid number of processes in y-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( pz <= 0 ) {
      std::cerr << " Invalid number of processes in z-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( nx % px != 0 ) {
      std::cerr << " Bad ratio between number of particles and number of processes in x-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( ny % py != 0 ) {
      std::cerr << " Bad ratio between number of particles and number of processes in y-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }
   if( nz % pz != 0 ) {
      std::cerr << " Bad ratio between number of particles and number of processes in z-dimension!\n";
      pe::exit( EXIT_FAILURE );
   }

   if( MPISettings::size() < px*py*pz ) {
      std::cerr << " Number of available processes is smaller than the number of processes specified on the command line." << std::endl;
      pe::exit(EXIT_FAILURE);
   }

   const real lx( nx * spacing );  // Length of the simulation domain in x-dimension
   const real ly( ny * spacing );  // Length of the simulation domain in y-dimension
   const real lz( nz * spacing );  // Length of the simulation domain in z-dimension

   setSeed( seed );  // Setup of the random number generation

   MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   WorldID     world     = theWorld();
   MPISystemID mpisystem = theMPISystem();


   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 3D Regular Domain Decomposition

   const real dx( lx / px );
   const real dy( ly / py );
   const real dz( lz / pz );

   int dims   [] = { px   , py   , pz    };
   int periods[] = { false, false, false };

   int rank;           // Rank of the neighboring process
   int center[3];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 3, dims, periods, false, &cartcomm );
   if( cartcomm == MPI_COMM_NULL ) {
      MPI_Finalize();
      return 0;
   }

   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 3, center );

   int west           [] = { center[0]-1, center[1]  , center[2]   };
   int east           [] = { center[0]+1, center[1]  , center[2]   };
   int south          [] = { center[0]  , center[1]-1, center[2]   };
   int north          [] = { center[0]  , center[1]+1, center[2]   };
   int southwest      [] = { center[0]-1, center[1]-1, center[2]   };
   int southeast      [] = { center[0]+1, center[1]-1, center[2]   };
   int northwest      [] = { center[0]-1, center[1]+1, center[2]   };
   int northeast      [] = { center[0]+1, center[1]+1, center[2]   };

   int bottom         [] = { center[0]  , center[1]  , center[2]-1 };
   int bottomwest     [] = { center[0]-1, center[1]  , center[2]-1 };
   int bottomeast     [] = { center[0]+1, center[1]  , center[2]-1 };
   int bottomsouth    [] = { center[0]  , center[1]-1, center[2]-1 };
   int bottomnorth    [] = { center[0]  , center[1]+1, center[2]-1 };
   int bottomsouthwest[] = { center[0]-1, center[1]-1, center[2]-1 };
   int bottomsoutheast[] = { center[0]+1, center[1]-1, center[2]-1 };
   int bottomnorthwest[] = { center[0]-1, center[1]+1, center[2]-1 };
   int bottomnortheast[] = { center[0]+1, center[1]+1, center[2]-1 };

   int top            [] = { center[0]  , center[1]  , center[2]+1 };
   int topwest        [] = { center[0]-1, center[1]  , center[2]+1 };
   int topeast        [] = { center[0]+1, center[1]  , center[2]+1 };
   int topsouth       [] = { center[0]  , center[1]-1, center[2]+1 };
   int topnorth       [] = { center[0]  , center[1]+1, center[2]+1 };
   int topsouthwest   [] = { center[0]-1, center[1]-1, center[2]+1 };
   int topsoutheast   [] = { center[0]+1, center[1]-1, center[2]+1 };
   int topnorthwest   [] = { center[0]-1, center[1]+1, center[2]+1 };
   int topnortheast   [] = { center[0]+1, center[1]+1, center[2]+1 };

   // Specify local domain
   defineLocalDomain( intersect(
      intersect(
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ),
      HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
      HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
      HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
      HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );

   // Connecting the west neighbor
   if( west[0] >= 0 ) {
      MPI_Cart_rank( cartcomm, west, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the east neighbor
   if( east[0] < px ) {
      MPI_Cart_rank( cartcomm, east, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the south neighbor
   if( south[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, south, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the north neighbor
   if( north[1] < py ) {
      MPI_Cart_rank( cartcomm, north, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the bottom neighbor
   if( bottom[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottom, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
   }

   // Connecting the top neighbor
   if( top[2] < pz ) {
      MPI_Cart_rank( cartcomm, top, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,0,+1), +top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
   }

   // Connecting the south-west neighbor
   if( southwest[0] >= 0 && southwest[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the south-east neighbor
   if( southeast[0] < px && southeast[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the north-west neighbor
   if( northwest[0] >= 0 && northwest[1] < py ) {
      MPI_Cart_rank( cartcomm, northwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the north-east neighbor
   if( northeast[0] < px && northeast[1] < py ) {
      MPI_Cart_rank( cartcomm, northeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,+1), +center[2]*dz ),
         HalfSpace( Vec3(0,0,-1), -top[2]*dz ) ) );
   }

   // Connecting the bottom-west neighbor
   if( bottomwest[0] >= 0 && bottomwest[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
   }

   // Connecting the bottom-east neighbor
   if( bottomeast[0] < px && bottomeast[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +east[0]*dx ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
   }

   // Connecting the bottom-south neighbor
   if( bottomsouth[1] >= 0 && bottomsouth[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
   }

   // Connecting the bottom-north neighbor
   if( bottomnorth[1] < py && bottomnorth[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +north[1]*dy ),
         HalfSpace( Vec3(0,0,-1), -center[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
   }

   // Connecting the bottom-south-west neighbor
   if( bottomsouthwest[0] >= 0 && bottomsouthwest[1] >= 0 && bottomsouthwest[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
   }

   // Connecting the bottom-south-east neighbor
   if( bottomsoutheast[0] < px && bottomsoutheast[1] >= 0 && bottomsoutheast[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
   }

   // Connecting the bottom-north-west neighbor
   if( bottomnorthwest[0] >= 0 && bottomnorthwest[1] < py && bottomnorthwest[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
   }

   // Connecting the bottom-north-east neighbor
   if( bottomnortheast[0] < px && bottomnortheast[1] < py && bottomnortheast[2] >= 0 ) {
      MPI_Cart_rank( cartcomm, bottomnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,-1), -center[2]*dz ) ) );
   }

   // Connecting the top-west neighbor
   if( topwest[0] >= 0 && topwest[2] < pz ) {
      MPI_Cart_rank( cartcomm, topwest, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
   }

   // Connecting the top-east neighbor
   if( topeast[0] < px && topeast[2] < pz ) {
      MPI_Cart_rank( cartcomm, topeast, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(1,0,0), east[0]*dx ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(0,+1,0), +center[1]*dy ),
         HalfSpace( Vec3(0,-1,0), -north[1]*dy ) ) );
   }

   // Connecting the top-south neighbor
   if( topsouth[1] >= 0 && topsouth[2] < pz ) {
      MPI_Cart_rank( cartcomm, topsouth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
   }

   // Connecting the top-north neighbor
   if( topnorth[1] < py && topnorth[2] < pz ) {
      MPI_Cart_rank( cartcomm, topnorth, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,1,0), north[1]*dy ),
         HalfSpace( Vec3(0,0,1), top[2]*dz ),
         HalfSpace( Vec3(+1,0,0), +center[0]*dx ),
         HalfSpace( Vec3(-1,0,0), -east[0]*dx ) ) );
   }

   // Connecting the top-south-west neighbor
   if( topsouthwest[0] >= 0 && topsouthwest[1] >= 0 && topsouthwest[2] < pz ) {
      MPI_Cart_rank( cartcomm, topsouthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
   }

   // Connecting the top-south-east neighbor
   if( topsoutheast[0] < px && topsoutheast[1] >= 0 && topsoutheast[2] < pz ) {
      MPI_Cart_rank( cartcomm, topsoutheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,-1,0), -center[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
   }

   // Connecting the top-north-west neighbor
   if( topnorthwest[0] >= 0 && topnorthwest[1] < py && topnorthwest[2] < pz ) {
      MPI_Cart_rank( cartcomm, topnorthwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
   }

   // Connecting the top-north-east neighbor
   if( topnortheast[0] < px && topnortheast[1] < py && topnortheast[2] < pz ) {
      MPI_Cart_rank( cartcomm, topnortheast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx ),
                                HalfSpace( Vec3(0,1,0), north[1]*dy ),
                                HalfSpace( Vec3(0,0,1), top[2]*dz ) ) );
   }

#ifndef NDEBUG
   // Checking the process setup
   mpisystem->checkProcesses();
#endif


   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;

   if( povray )
   {
      const Vec3 location( real(0.5)*lx, -real(0.5)*lx/0.65, real(0.5)*lz );
      const Vec3 focus   ( real(0.5)*lx,  real(0.5)*ly     , real(0.5)*lz );

      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->setFilename( "video/pic%.pov" );
      pov->setBackground( Color( 0, 0, 0 ) );
      pov->include( "settings.inc" );

      pov->addLightSource( PointLight( location, Color( 1, 1, 1 ) ) );

      CameraID camera = theCamera();
      camera->setLocation( location );
      camera->setFocus   ( focus    );

      std::ostringstream texture;
      texture << "Texture" << theMPISystem()->getRank()%13;
      pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
   }


   /////////////////////////////////////////////////////
   // Setup of the simulation domain

   pe_GLOBAL_SECTION
   {
      createPlane( 0,  1.0,  0.0,  0.0, 0.0, elastic );
      createPlane( 0, -1.0,  0.0,  0.0, -lx, elastic );
      createPlane( 0,  0.0,  1.0,  0.0, 0.0, elastic, false );
      createPlane( 0,  0.0, -1.0,  0.0, -ly, elastic );
      createPlane( 0,  0.0,  0.0,  1.0, 0.0, elastic );
      createPlane( 0,  0.0,  0.0, -1.0, -lz, elastic );
   }


   /////////////////////////////////////////////////////
   // Deterministic setup of the particles

   Vec3 gpos, vel;
   size_t id( 0 );

   const int nxpp( nx / px );
   const int nypp( ny / py );
   const int nzpp( nz / pz );

   // Starting the time measurement for the setup
   WcTimer setupTime;
   setupTime.start();

   // Strong scaling initialization
   if( strong )
   {
      for( int z=0; z<nz; ++z ) {
         for( int y=0; y<ny; ++y ) {
            for( int x=0; x<nx; ++x )
            {
               ++id;
               gpos.set( ( x+real(0.5) ) * spacing,
                         ( y+real(0.5) ) * spacing,
                         ( z+real(0.5) ) * spacing );
               vel.set( rand<real>( -velocity, velocity ),
                        rand<real>( -velocity, velocity ),
                        rand<real>( -velocity, velocity ) );

               if( world->ownsPoint( gpos ) ) {
                  BodyID particle;
                  if( spheres ) particle = createSphere( id, gpos, radius, elastic );
                  else particle = createGranularParticle( id, gpos, radius, elastic );
                  particle->setLinearVel( vel );
               }
            }
         }
      }
   }

   // Weak scaling initialization
   else
   {
      for( int z=0; z<nzpp; ++z ) {
         for( int y=0; y<nypp; ++y ) {
            for( int x=0; x<nxpp; ++x )
            {
               ++id;
               gpos.set( ( center[0]*nxpp+x+real(0.5) )*spacing,
                         ( center[1]*nypp+y+real(0.5) )*spacing,
                         ( center[2]*nzpp+z+real(0.5) )*spacing );
               vel.set( rand<real>( -velocity, velocity ),
                        rand<real>( -velocity, velocity ),
                        rand<real>( -velocity, velocity ) );

               if( !world->ownsPoint( gpos ) ) {
                  std::cerr << " Invalid particle position on process " << mpisystem->getRank() << "\n"
                            << "   Coordinates     = (" << center[0] << "," << center[1] << "," << center[2] << ")\n"
                            << "   Global position = " << gpos << "\n"
                            << std::endl;
                  pe::exit( EXIT_FAILURE );
               }

               BodyID particle;
               if( spheres ) particle = createSphere( id, gpos, radius, elastic );
               else particle = createGranularParticle( id, gpos, radius, elastic );
               particle->setLinearVel( vel );
            }
         }
      }
   }

   // Synchronization of the MPI processes
   world->synchronize();

   // Ending the time measurement for the setup
   setupTime.end();


   /////////////////////////////////////////////////////
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "------------------------------------------------------------\n"
                << " Size of the domain                      = (" << lx << "," << ly << "," << lz << ")\n"
                << " Number of MPI processes                 = (" << px << "," << py << "," << pz << ")\n"
                << " Total number of particles               = " << nx*ny*nz << "\n"
                << " Number of particles on each process     = " << nxpp*nypp*nzpp << "\n"
                << " Radius of a single particle             = " << radius << "\n"
                << " Initial spacing inbetween two particles = " << spacing << "\n"
                << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << std::endl;
      std::cout << "------------------------------------------------------------------------------" << std::endl;
   }


   /////////////////////////////////////////////////////
   // Setup timing results

   double localTime( setupTime.total() );
   double globalMin( 0.0 );
   double globalMax( 0.0 );
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, cartcomm );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SETUP RESULTS" << pe_OLDCOLOR << "---------------------------------------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "------------------------------------------------------------------------------" << std::endl;
   }


   /////////////////////////////////////////////////////
   // Simulation loop

   WcTimer simTime;
   simTime.start();
   world->run( timesteps, stepsize );
   simTime.end();


   /////////////////////////////////////////////////////
   // Simulation timing results

   localTime  = simTime.total();
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, cartcomm );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, cartcomm );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION RESULTS" << pe_OLDCOLOR << "----------------------------------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // MPI Finalization

   MPI_Finalize();
}
//*************************************************************************************************
