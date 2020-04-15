//=================================================================================================
/*!
 *  \file MPILSS.cpp
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
#include <iostream>
#include <sstream>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;




//=================================================================================================
//
//  POVRAY CAMERA ANIMATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Animation for the POV-Ray camera.
 *
 * This class animates the POV-Ray camera for the MPILSS example. The camera performs a circular
 * motion from behind the LSS structure to its front.
 */
class CircularMotion : public CameraAnimation
{
 public:
   //**Constructor*********************************************************************************
   explicit CircularMotion( real angle )
      : q_( Vec3(0,0,1), angle )
   {}
   //**********************************************************************************************

   //**Update functions****************************************************************************
   virtual void updateLocation( Vec3& location )
   {
      location = q_.rotate( location );
   }
   //**********************************************************************************************

 private:
   //**Member variables****************************************************************************
   Quat q_;
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY TEXTURE POLICY
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Texture policy for the mpistair example.
 *
 * This class represents the POV-Ray texture policy for the mpistair example.
 */
class ParticleTexturePolicy : public TexturePolicy
{
 public:
   explicit ParticleTexturePolicy()
   {}

   virtual ~ParticleTexturePolicy()
   {}

   virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
   {
      switch( body->getID() )
      {
         case 0 : return CustomTexture( "Texture0"  );
         case 1 : return CustomTexture( "Texture1"  );
         case 2 : return CustomTexture( "Texture2"  );
         case 3 : return CustomTexture( "Texture3"  );
         case 4 : return CustomTexture( "Texture4"  );
         case 5 : return CustomTexture( "Texture5"  );
         case 6 : return CustomTexture( "Texture6"  );
         case 7 : return CustomTexture( "Texture7"  );
         case 8 : return CustomTexture( "Texture8"  );
         case 9 : return CustomTexture( "Texture9"  );
         case 10: return CustomTexture( "Texture10" );
         case 11: return CustomTexture( "Texture11" );
         case 12: return CustomTexture( "Texture12" );
         default: return Texture();
      }
   }

   using TexturePolicy::getTexture;
};
//*************************************************************************************************




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mpilss example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Simulation parameters for the random simulation

   // Timing variables
   const size_t timesteps ( 30000 );  // Total number of time steps
   const real stepsize    ( 0.001 );  // Size of a single time step

   // Sphere parameters
   const size_t nx         ( 80   );  // Number of particles in x-direction
   const size_t ny         ( 40   );  // Number of particles in y-direction
   const size_t nz         ( 10   );  // Number of particles in z-direction
   const real   dv         ( 0.1  );  // Initial variation of the particle velocities
   const real   prob       ( 0.6  );  // Sphere generation probability
   const real   density    ( 1.5  );  // Density of the spherical particles
   const real   restitution( 0.04 );  // Coefficient of restitution of the particles
   const real   friction   ( 0.25 );  // Coefficient of friction of the particles
   const real   poisson    ( 0.3  );  // Poisson's ration of the particle material
   const real   young      ( 300  );  // Young's modulus of the particle material
   const real   stiffness  ( 1e6  );  // The stiffness in normal direction of the contact region
   const real   dampingN   ( 1e5  );  // The damping coefficient in normal direction of the contact region
   const real   dampingT   ( 2e5  );  // The damping coefficient in tangential direction of the contact region

   // Process parameters
   const int px( 2 );  // Number of processes in x-direction
   const int py( 2 );  // Number of processes in y-direction

   // Verbose mode
   const bool verbose( true );  // Switches the output of the simulation on and off

   // Visualization systems
   bool povray( true );  // Switches the POV-Ray visualization on and off

   // Visualization parameters
   const bool   colorProcesses( true );  // Switches the processes visualization on and off
   const bool   animation     ( true );  // Switches the animation of the POV-Ray camera on and off
   const size_t visspacing    (  150 );  // Number of time steps inbetween two POV-Ray files


   /////////////////////////////////////////////////////
   // Initial setups

   const real diameter ( max( 40.0/nx,  25.0/ny ) );  // Maximum diameter of a spherical particle
   const real radius   ( diameter/2 );                // Maximum radius of a spherical particle
   const real lx       ( nx*diameter );               // Size of the drop zone in x-direction
   const real ly       ( nx*diameter );               // Size of the drop zone in y-direction

   MPISystemID mpisystem = theMPISystem();
   const int processes( mpisystem->getSize() );  // The total number of active MPI processes
   const int rank     ( mpisystem->getRank() );  // The rank of the MPI process

   // Checking the particle generation probability
   if( prob < real(0) || prob > real(1) ) {
      std::cerr << pe_RED
                << "\n Invalid particle generation probability!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the total number of MPI processes
   if( px <= 0 || py <= 0 || px*py != processes ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Setup of the random number generation
   setSeed( 12345 );

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;

   // Creating the material for the particles
   MaterialID granular = createMaterial( "granular", density, restitution,
                                         friction, friction, poisson, young,
                                         stiffness, dampingN, dampingT );

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -6.0 );


   /////////////////////////////////////////////////////
   // PovRay visualization setup

   WriterID pov;

   const ParallelLight light1(
      Vec3( 0.0, 0.0, 20.0 ),
      Color( 0.9, 0.9, 0.9 ),
      PointAt( 0.0, 0.0, 0.0 ),
      Shadowless()
   );

   const PointLight light2(
      Vec3( 12.0, -15.0, 30.0 ),
      Color( 0.9, 0.9, 0.9 )
   );

   const CustomTexture blackTexture( "BlackTexture" );

   if( povray )
   {
      // Configuration of the POV-Ray writer
      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->setFilename( "video/pic%.pov" );
      pov->include( "settings.inc" );
      pov->setBackground( Color( 0, 0, 0 ) );

      // Configuration of the scene lighting
      pov->addLightSource( light1 );
      pov->addLightSource( light2 );

      // Configuration of the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( -8.0, -22.0, 26.0 );
      camera->setFocus   ( -3.0,   2.0, -4.0 );

      // Animating the POV-Ray camera
      if( animation )
         camera->animate( CircularMotion( M_PI/(4*timesteps) ) );

      // Configuration of the texture policy
      if( colorProcesses ) {
         std::ostringstream texture;
         texture << "Texture" << theMPISystem()->getRank()%13;
         pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
      }
      else {
         pov->setTexturePolicy( ParticleTexturePolicy() );
      }
   }


   /////////////////////////////////////////////////////
   // Setup of the MPI processes: 2D Regular Domain Decomposition

   const real hlx( real(0.5)*lx );  // Half length of the drop zone in x-direction
   const real hly( real(0.5)*ly );  // Half length of the drop zone in y-direction
   const real dx ( lx / px );       // X-size of a MPI process
   const real dy ( ly / py );       // Y-size of a MPI process

   int dims   [] = { px   , py    };  // Dimensions of the 2D Cartesian process topology
   int periods[] = { false, false };  // Periodicity of the 2D Cartesian process topology

   int neighbor;       // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, rank, 2, center );

   int west     [] = { center[0]-1, center[1]   };
   int east     [] = { center[0]+1, center[1]   };
   int south    [] = { center[0]  , center[1]-1 };
   int north    [] = { center[0]  , center[1]+1 };
   int southwest[] = { center[0]-1, center[1]-1 };
   int southeast[] = { center[0]+1, center[1]-1 };
   int northwest[] = { center[0]-1, center[1]+1 };
   int northeast[] = { center[0]+1, center[1]+1 };

   // Specify local subdomain
   defineLocalDomain( intersect(
      HalfSpace( Vec3(+1,0,0), +( center[0]*dx - hlx ) ),
      HalfSpace( Vec3(-1,0,0), -( east[0]*dx - hlx ) ),
      HalfSpace( Vec3(0,+1,0), +( center[1]*dy - hly ) ),
      HalfSpace( Vec3(0,-1,0), -( north[1]*dy - hly ) ) ) );

   // Connecting the west neighbor
   if( west[0] >= 0 ) {
      MPI_Cart_rank( cartcomm, west, &neighbor );
      connect( neighbor, intersect(
         HalfSpace( Vec3(-1,0,0), -( center[0]*dx - hlx ) ),
         HalfSpace( Vec3(0,+1,0), +( center[1]*dy - hly ) ),
         HalfSpace( Vec3(0,-1,0), -( north[1]*dy - hly ) ) ) );
   }

   // Connecting the east neighbor
   if( east[0] < px ) {
      MPI_Cart_rank( cartcomm, east, &neighbor );
      connect( neighbor, intersect(
         HalfSpace( Vec3(+1,0,0), +( east[0]*dx - hlx ) ),
         HalfSpace( Vec3(0,+1,0), +( center[1]*dy - hly ) ),
         HalfSpace( Vec3(0,-1,0), -( north[1]*dy - hly ) ) ) );
   }

   // Connecting the south neighbor
   if( south[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, south, &neighbor );
      connect( neighbor, intersect(
         HalfSpace( Vec3(0,-1,0), -( center[1]*dy - hly ) ),
         HalfSpace( Vec3(+1,0,0), +( center[0]*dx - hlx ) ),
         HalfSpace( Vec3(-1,0,0), -( east[0]*dx - hlx ) ) ) );
   }

   // Connecting the north neighbor
   if( north[1] < py ) {
      MPI_Cart_rank( cartcomm, north, &neighbor );
      connect( neighbor, intersect(
         HalfSpace( Vec3(0,+1,0), +( north[1]*dy - hly ) ),
         HalfSpace( Vec3(+1,0,0), +( center[0]*dx - hlx ) ),
         HalfSpace( Vec3(-1,0,0), -( east[0]*dx - hlx ) ) ) );
   }

   // Connecting the south-west neighbor
   if( southwest[0] >= 0 && southwest[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southwest, &neighbor );
      connect( neighbor, intersect( HalfSpace( Vec3(-1,0,0), hlx-center[0]*dx ),
                                    HalfSpace( Vec3(0,-1,0), hly-center[1]*dy ) ) );
   }

   // Connecting the south-east neighbor
   if( southeast[0] < px && southeast[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southeast, &neighbor );
      connect( neighbor, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx-hlx ),
                                    HalfSpace( Vec3(0,-1,0), hly-center[1]*dy ) ) );
   }

   // Connecting the north-west neighbor
   if( northwest[0] >= 0 && northwest[1] < py ) {
      MPI_Cart_rank( cartcomm, northwest, &neighbor );
      connect( neighbor, intersect( HalfSpace( Vec3(-1,0,0), hlx-center[0]*dx ),
                                    HalfSpace( Vec3(0,1,0), north[1]*dy-hly ) ) );
   }

   // Connecting the north-east neighbor
   if( northeast[0] < px && northeast[1] < py ) {
      MPI_Cart_rank( cartcomm, northeast, &neighbor );
      connect( neighbor, intersect( HalfSpace( Vec3(1,0,0), east[0]*dx-hlx ),
                                    HalfSpace( Vec3(0,1,0), north[1]*dy-hly ) ) );
   }

#ifndef NDEBUG
   // Checking the process setup
   mpisystem->checkProcesses();
#endif


   /////////////////////////////////////////////////////
   // Rigid body setup

   // Setup of the LSS structure
   pe_GLOBAL_SECTION
   {
      // Creating the LSS structure
      pe_CREATE_UNION( lss, 0 )
      {
         // Setup of the ground box
         createBox( 0, 0.0, 0.0, -1.0, 38.0, 24.0, 2.0, granular );

         // Setup of the left side box
         createBox( 0, -18.0, 0.0, 2.0, 2.0, 24.0, 4.0, granular );

         // Setup of the right side box
         createBox( 0,  18.0, 0.0, 2.0, 2.0, 24.0, 4.0, granular );

         // Setup of the top box
         createBox( 0, 0.0,  11.0, 2.0, 34.0, 2.0, 4.0, granular );

         // Setup of the bottom box
         createBox( 0, 0.0, -11.0, 2.0, 34.0, 2.0, 4.0, granular );

         // Setup of the interior
         createBox( 0, -9.0,  2.0, 2.0, 8.0, 16.0, 4.0, granular );
         createBox( 0, -6.0, -8.0, 2.0, 2.0,  4.0, 4.0, granular );
         createBox( 0, -2.0, -4.0, 2.0, 6.0,  4.0, 4.0, granular );
         createBox( 0,  2.0,  4.0, 2.0, 6.0,  4.0, 4.0, granular );
         createBox( 0,  6.0,  0.0, 2.0, 2.0, 20.0, 4.0, granular );
         createBox( 0, 10.0, -4.0, 2.0, 6.0,  4.0, 4.0, granular );
         createBox( 0, 14.0,  4.0, 2.0, 6.0,  4.0, 4.0, granular );

         lss->rotate( M_PI/12.0, 0.0, 0.0 );
         if( povray ) pov->setTexture( lss, blackTexture );

         lss->setFixed( true );
      }

      // Add planes in order to prevent ouflow of the domain (30.3.2012: This has not been implemented yet in the pe)
      createPlane( 0, +1.0,  0.0,  0.0, -hlx, granular, false );
      createPlane( 0, -1.0,  0.0,  0.0, -hlx, granular, false );
      createPlane( 0,  0.0, +1.0,  0.0, -hly, granular, false );
      createPlane( 0,  0.0, -1.0,  0.0, -hly, granular, false );
   }

   // Setup of the falling particles
   const real rmax( 0.9*radius );
   const real rmin( 0.5*radius );
   const real minx( -( (nx+1)*diameter )/2 );
   const real miny( -( (ny+1)*diameter )/2 );
   const real minz( 25.0 );

   unsigned int spheres( 0 );
   real posx, posy, posz( minz );

   for( size_t i=0; i<nz; ++i )
   {
      posy = miny;

      for( size_t j=0; j<ny; ++j )
      {
         posx = minx;

         for( size_t k=0; k<nx; ++k )
         {
            if( world->ownsPoint( posx, posy, posz ) && rand<real>() < prob ) {
               SphereID sphere = createSphere( rand<size_t>(0,12), posx, posy, posz, rand<real>(rmin,rmax), granular );
               sphere->setLinearVel( rand<real>(-dv,dv), rand<real>(-dv,dv), -0.1 );
               ++spheres;
            }

            posx += diameter;
         }

         posy += diameter;
      }

      posz += diameter;
   }

   // Synchronization of the MPI processes
   world->synchronize();


   /////////////////////////////////////////////////////
   // Output of the simulation settings

   unsigned int total;
   MPI_Reduce( &spheres, &total, 1, MPI_UNSIGNED, MPI_SUM, 0, cartcomm );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "--------------------------------------------------------------\n"
                << " MPI processes in x-direction            = " << px << "\n"
                << " MPI processes in y-direction            = " << py << "\n"
                << " Total number of spherical particles     = " << total << "\n"
                << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << std::endl;
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // Simulation loop

   WcTimer time;
   time.start();

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
                << "---------------------------------------------------------" << std::endl;
   }

   for( size_t timestep=0; timestep<timesteps; ++timestep )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating time step step " <<  timestep+1 << "   " << std::flush;
         }
      }

      world->simulationStep( stepsize );
   }

   time.end();

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << " Total WC-time : " << time.total() << "\n"
                << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // MPI Finalization

   MPI_Finalize();
}
//*************************************************************************************************
