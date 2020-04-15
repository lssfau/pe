//=================================================================================================
/*!
 *  \file MPICube.cpp
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
 * This class animates the POV-Ray camera for the MPICube example. The camera is initially
 * located close to the particles to allow a close look at them before slowly shifting backwards
 * to give a complete view of the simulation scenario. During the simulation the camera moves
 * upward for an overview of the particle movements.
 */
class LinearShift : public CameraAnimation
{
 public:
   explicit LinearShift( size_t start1, size_t end1, size_t start2, size_t end2,
                         const Vec3& location1, const Vec3& location2, const Vec3& location3 )
      : start1_   ( start1    )  // First time step of the camera backwards shift
      , end1_     ( end1      )  // Last time step of the camera backwards shift
      , start2_   ( start2    )  // First time step of the camera upwards shift
      , end2_     ( end2      )  // Last time step of the camera upwards shift
      , location1_( location1 )  // Initial location of the camera
      , location2_( location2 )  // Intermediate location of the camera
      , location3_( location3 )  // Final location of the camera
   {}

   virtual ~LinearShift()
   {}

   virtual void updateLocation( Vec3& location )
   {
      if( TimeStep::step() > start1_ && TimeStep::step() <= end1_ ) {
         const Vec3 disp( ( location1_ - location2_ ) / ( end1_ - start1_ ) );
         location -= disp;
      }
      else if( TimeStep::step() > start2_ && TimeStep::step() <= end2_ ) {
         const Vec3 disp( ( location2_ - location3_ ) / ( end2_ - start2_ ) );
         location -= disp;
      }
   }

 private:
   size_t start1_;   // First time step of the camera backwards shift
   size_t end1_;     // Last time step of the camera backwards shift
   size_t start2_;   // First time step of the camera upwards shift
   size_t end2_;     // Last time step of the camera upwards shift
   Vec3 location1_;  // Initial location of the camera
   Vec3 location2_;  // Intermediate location of the camera
   Vec3 location3_;  // Final location of the camera
};
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY TEXTURE POLICY
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Texture policy for the mpicube example.
 *
 * This class represents the POV-Ray texture policy for the mpicube example.
 */
class GranularTexturePolicy : public TexturePolicy
{
 public:
   explicit GranularTexturePolicy()
   {}

   virtual ~GranularTexturePolicy()
   {}

   virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
   {
      switch( body->getID() )
      {
         case 0:  return CustomTexture( "GranularTexture1" );  // Granular media texture 1
         case 1:  return CustomTexture( "GranularTexture2" );  // Granular media texture 2
         case 2:  return CustomTexture( "GroundTexture"    );  // Texture for the ground plane
         case 3:  return CustomTexture( "GlassTexture"     );  // Texture for the cube
         default: return CustomTexture( "BlackTexture"     );  // Texture for the platform
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
/*!\brief Main function for the mpicube example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The mpicube example is a box with a box-shaped pillar in the middle. The box gets filled with
 * granular material and then the walls of the box swing open and the particles spill out in all
 * directions. The domain is decomposed into cake slices originating in the middle of the
 * container. The x-y-plane is sliced.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Simulation parameters

   // Particle parameters
   const bool   spheres (  true );  // Switch between spheres and granular particles
   const real   radius  (  0.7  );  // The radius of spheres of the granular media
   const real   spacing (  0.1  );  // Initial spacing inbetween two spheres
   const real   velocity(  0.01 );  // Initial maximum velocity of the spheres

   // Environment parameters
   const real lx   ( 30.0 );  // Size of the cube in x-direction
   const real ly   ( 30.0 );  // Size of the cube in y-direction
   const real lz   ( 30.0 );  // Size of the cube in z-direction
   const real hz   (  5.0 );  // Height of the cube platform
   const real width(  0.7 );  // Width of the cube walls

   // Time parameters
   const size_t initsteps     ( 15000 );  // Initialization steps with closed outlet door
   const size_t focussteps    (   100 );  // Number of initial close-up time steps
   const size_t animationsteps(   200 );  // Number of time steps for the camera animation
   const size_t opensteps     ( 18000 );  // Number of time steps for the door opening
   const size_t timesteps     ( 27000 );  // Number of time steps for the flowing granular media
   const real   stepsize      ( 0.001 );  // Size of a single time step

   // Random number generator parameters
   const size_t seed( 12345 );

   // Verbose mode
   const bool verbose( true );  // Switches the output of the simulation on and off

   // Visualization
   bool povray( true );  // Switches the POV-Ray visualization on and off

   // Visualization parameters
   const bool   colorProcesses(  true );  // Switches the processes visualization on and off
   const bool   animation     (  true );  // Switches the animation of the POV-Ray camera on and off
   const size_t visspacing    (   150 );  // Number of time steps inbetween two POV-Ray files
   const size_t colorwidth    (    10 );  // Number of spheres in z-dimension with a specific color


   /////////////////////////////////////////////////////
   // Initial setups

   const int processes( theMPISystem()->getSize() );  // The total number of active MPI processes
   const int rank     ( theMPISystem()->getRank() );  // The rank of the MPI process

   // Space initially required by a single particle
   const real space( real(2)*radius+spacing );

   // Rotation angle of the side walls per time step
   const real angle( ( M_PI/2+std::asin( (hz-3*radius)/lz) )/opensteps );

   // Checking the total number of MPI processes
   if( processes < 3 ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Setup of the random number generation
   setSeed( seed );

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;

   // Creating the material for the particles
   MaterialID granular = createMaterial( "granular", 1.0, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );
   world->setDamping( 0.9 );


   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;

   CustomTexture groundTexture ( "GroundTexture" );
   CustomTexture glassTexture  ( "GlassTexture"  );
   CustomTexture blackTexture  ( "BlackTexture"  );

   if( povray )
   {
      // Calculation of the camera location and focus point
      // Aspect ratio of the POV-Ray camera: 4/3 => Up-angle = 26.565 deg (0.4636 rad)
      const Vec3 location1( 2.5*lx, 1.6*ly, hz+0.5*lz );
      const Vec3 location2( 3.2*lx, 0.5*ly, hz+0.3*lz );
      const Vec3 location3( 3.2*lx, 1.0*ly, 4.5*lz    );
      const Vec3 focus    ( 2.5*lx, 2.5*ly, hz+0.5*lz );

      // Configuration of the POV-Ray writer
      pov = activateWriter();
      pov->setStart( initsteps );
      pov->setSpacing( 1 );
      pov->setFilename( "video/pic%.pov" );
      pov->include( "settings.inc" );
      pov->setBackground( Color( 0, 0, 0 ) );

      // Configuration of the scene lighting
      pov->addLightSource( PointLight   ( location2        , Color( 0.5, 0.5, 0.5 ) ) );
      pov->addLightSource( ParallelLight( Vec3( 0, -50, 0 ), Color( 1.0, 1.0, 1.0 ), Shadowless() ) );
      pov->addLightSource( ParallelLight( Vec3( 0, 0, 100 ), Color( 1.0, 1.0, 1.0 ), Shadowless() ) );

      // Calculating the parameters for the camera animation
      const size_t start1( initsteps+focussteps  );
      const size_t end1  ( start1+animationsteps );
      const size_t start2( end1+opensteps );
      const size_t end2  ( start2+timesteps );

      // Configuration of the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( ( animation )?( location1 ):( location2 ) );
      camera->setFocus   ( focus );

      // Animating the POV-Ray camera
      if( animation )
         camera->animate( LinearShift( start1, end1, start2, end2, location1, location2, location3 ) );

      // Configuration of the texture policy
      if( colorProcesses ) {
         std::ostringstream texture;
         texture << "Texture" << theMPISystem()->getRank()%13;
         pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
      }
      else {
         pov->setTexturePolicy( GranularTexturePolicy() );
      }
   }


   /////////////////////////////////////////////////////
   // Setup of the MPI processes

   // Decompose x-y-plane into cake slices
   const real da    ( real(2)*M_PI/processes );
   const Vec3 center( 2.5*lx, 2.5*ly, 0.0 );

   // Normals of neighboring subdomains in x-y-plane pointing towards previous cake slice
   Vec3 n[4];
   for( int i = rank - 1; i <= rank + 2; ++i )
      n[i - (rank - 1)] = Quat( Vec3(0, 0, -1), ( ( i + processes ) % processes ) * da ).rotate( Vec3(0, -1, 0) );

   // Connect remote subdomains of both nearest neighbors
   connect( ( rank - 1 + processes ) % processes, intersect(
      HalfSpace( -n[0], center ),
      HalfSpace(  n[1], center ) ) );
   defineLocalDomain( intersect(
      HalfSpace( -n[1], center ),
      HalfSpace(  n[2], center ) ) );
   connect( ( rank + 1 ) % processes, intersect(
      HalfSpace( -n[2], center ),
      HalfSpace(  n[3], center ) ) );

#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
#endif


   /////////////////////////////////////////////////////
   // Setup of the global objects

   const Vec3 leftPivotPoint ( 2*lx-width, 2.5*ly, hz );  // Pivot point for the left cube wall
   const Vec3 rightPivotPoint( 3*lx+width, 2.5*ly, hz );  // Pivot point for the right cube wall
   const Vec3 frontPivotPoint( 2.5*lx, 2*ly-width, hz );  // Pivot point for the front cube wall
   const Vec3 backPivotPoint ( 2.5*lx, 3*ly+width, hz );  // Pivot point for the back cube wall

   BoxID left   ( 0 );  // Handle for the left cube wall
   BoxID right  ( 0 );  // Handle for the right cube wall
   BoxID front  ( 0 );  // Handle for the front cube wall
   BoxID back   ( 0 );  // Handle for the back cube wall
   BoxID lpillar( 0 );  // Handle for the lower center pillar
   BoxID upillar( 0 );  // Handle for the upper center pillar

   pe_GLOBAL_SECTION
   {
      // Creating the ground plane
      PlaneID groundPlane = createPlane( 2, 0.0, 0.0, 1.0, 0.0, granular );
      if( povray && colorProcesses ) pov->setTexture( groundPlane, groundTexture );

      // Creating the left boundary plane
      createPlane( 2, 1.0, 0.0, 0.0, 0.0, granular, false );

      // Creating the right boundary plane
      createPlane( 2, -1.0, 0.0, 0.0, -real(5)*lx, granular, false );

      // Creating the front boundary plane
      createPlane( 2, 0.0, 1.0, 0.0, 0.0, granular, false );

      // Creating the back boundary plane
      createPlane( 2, 0.0, -1.0, 0.0, -real(5)*lx, granular, false );

      // Creating the cube
      pe_CREATE_UNION( outlet, 3 )
      {
         // Creating the platform
         BoxID platform = createBox( 4, 2.5*lx, 2.5*ly, 0.5*hz,
                                        lx+2*width, ly+2*width, hz, granular );
         if( povray && colorProcesses ) pov->setTexture( platform, blackTexture );

         // Creating the lower-left cube wall
         left = createBox( 3, 2*lx-0.5*width, 2.5*ly, hz+0.5*lz,
                              width, ly, lz, granular );
         if( povray && colorProcesses ) pov->setTexture( left, glassTexture );

         // Creating the upper-left cube wall
         createBox( 3, 2*lx-0.5*width, 2.5*ly, hz+1.5*lz,
                       width, ly, lz, granular, false );

         // Creating the lower-right cube wall
         right = createBox( 3, 3*lx+0.5*width, 2.5*ly, hz+0.5*lz,
                               width, ly, lz, granular );
         if( povray && colorProcesses ) pov->setTexture( right, glassTexture );

         // Creating the upper-right cube wall
         createBox( 3, 3*lx+0.5*width, 2.5*ly, hz+1.5*lz,
                       width, ly, lz, granular, false );

         // Creating the lower-front cube wall
         front = createBox( 3, 2.5*lx, 2*ly-0.5*width, hz+0.5*lz,
                               lx, width, lz, granular );
         if( povray && colorProcesses ) pov->setTexture( front, glassTexture );

         // Creating the upper-front cube wall
         createBox( 3, 2.5*lx, 2*ly-0.5*width, hz+1.5*lz,
                       lx, width, lz, granular, false );

         // Creating the lower-back cube wall
         back = createBox( 3, 2.5*lx, 3*ly+0.5*width, hz+0.5*lz,
                              lx, width, lz, granular );
         if( povray && colorProcesses ) pov->setTexture( back, glassTexture );

         // Creating the upper-back cube wall
         createBox( 3, 2.5*lx, 3*ly+0.5*width, hz+1.5*lz,
                       lx, width, lz, granular, false );

         // Creating the lower center pillar
         lpillar = createBox( 3, 2.5*lx, 2.5*ly, hz+0.5*lz, 0.3*lx, 0.3*ly, lz, granular );
         if( povray && colorProcesses ) pov->setTexture( lpillar, blackTexture );

         // Creating the upper center pillar
         upillar = createBox( 3, 2.5*lx, 2.5*ly, hz+1.5*lz, 0.3*lx, 0.3*ly, lz, granular, false );

         outlet->setFixed( true );
      }
   }


   /////////////////////////////////////////////////////
   // Setup of the granular medium

   const size_t nx( static_cast<size_t>(     lx / space ) );
   const size_t ny( static_cast<size_t>(     ly / space ) );
   const size_t nz( static_cast<size_t>( 1.4*lz / space ) );
   const real   dx(     lx / nx );
   const real   dy(     ly / ny );
   const real   dz( 1.5*lz / nz );

   BodyID particle;
   unsigned long local( 0 );

   for( size_t i=0; i<nz; ++i ) {
      for( size_t j=0; j<ny; ++j ) {
         for( size_t k=0; k<nx; ++k )
         {
            const Vec3 gpos( 2.0*lx+0.5*dx+k*dx, 2.0*ly+0.5*dy+j*dy, hz+0.5*dz+i*dz );
            const Vec3 vel ( rand<real>( -velocity, velocity ),
                             rand<real>( -velocity, velocity ),
                             rand<real>( -velocity, 0.0 ) );

            if( world->ownsPoint( gpos ) )
            {
               const size_t uid( (i/colorwidth)%2 );

               if( spheres ) particle = createSphere( uid, gpos, radius, granular );
               else particle = createGranularParticle( uid, gpos, radius, granular );

               if( overlap( particle, lpillar ) || overlap( particle, upillar ) )
                  destroy( particle );
               else {
                  particle->setLinearVel( vel );
                  ++local;
               }
            }
         }
      }
   }


   /////////////////////////////////////////////////////
   // Output of the simulation settings

   unsigned long global( 0 );
   MPI_Reduce( &local, &global, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, MPI_COMM_WORLD );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "--------------------------------------------------------------\n"
                << " Total number of MPI processes           = " << processes << "\n"
                << " Total number of particles               = " << global << "\n"
                << " Number of initialization steps          = " << initsteps << "\n"
                << " Number of camera focus steps            = " << focussteps << "\n"
                << " Number of camera animation steps        = " << animationsteps << "\n"
                << " Number of opening steps                 = " << opensteps << "\n"
                << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << std::endl;
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }

   if( povray )
      pov->writeFile( "video/init.pov" );


   /////////////////////////////////////////////////////
   // Simulation loop

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
                << "---------------------------------------------------------" << std::endl;
   }

   timing::WcTimer totalTime;
   timing::WcTimer simTime;

   totalTime.start();

   // Initialization phase
   if( !verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << initsteps << " initialization steps..." << std::endl;
      }
   }

   for( size_t timestep=0; timestep<initsteps; ++timestep )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating initialization step " <<  timestep+1 << "   " << std::flush;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   // Camera focus phase
   if( !verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << focussteps << " camera focus steps..." << std::endl;
      }
   }

   for( size_t timestep=0; timestep<focussteps; ++timestep )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating camera focus step " <<  timestep+1 << "   " << std::flush;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   // Camera animation phase
   if( !verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << animationsteps << " camera animation steps..." << std::endl;
      }
   }

   for( size_t timestep=0; timestep<animationsteps; ++timestep )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating camera animation step " <<  timestep+1 << "   " << std::flush;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   // Adjusting the POV-Ray spacing
   if( povray ) {
      pov->setSpacing( visspacing );
   }

   // Opening the door of the granular media outlet
   if( !verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << opensteps << " opening steps..." << std::endl;
      }
   }

   for( size_t timestep=0; timestep<opensteps; ++timestep )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating opening step " <<  timestep+1 << "   " << std::flush;
         }
      }

      left->rotateAroundPoint ( leftPivotPoint , Vec3( 0, -angle, 0 ) );
      right->rotateAroundPoint( rightPivotPoint, Vec3( 0,  angle, 0 ) );
      front->rotateAroundPoint( frontPivotPoint, Vec3(  angle, 0, 0 ) );
      back->rotateAroundPoint ( backPivotPoint , Vec3( -angle, 0, 0 ) );
      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   // Running the granular media simulation
   if( !verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << timesteps << " time steps..." << std::endl;
      }
   }

   for( size_t timestep=0; timestep<timesteps; ++timestep )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating time step " <<  timestep+1 << "   " << std::flush;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   totalTime.end();

   pe_EXCLUSIVE_SECTION( 0 ) {
      if( verbose ) std::cout << "\n";
      std::cout << "\n"
                << " WC-Time:  Pure simulation : " << simTime.total() << "\n"
                << "           Total time      : " << totalTime.total() << "\n";
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // MPI Finalization

   MPI_Finalize();
}
//*************************************************************************************************
