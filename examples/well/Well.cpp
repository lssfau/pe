//=================================================================================================
/*!
 *  \file Well.cpp
 *  \brief Example file for the pe physics engine
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

#include <cmath>
#include <cstddef>
#include <iostream>
#include <pe/engine.h>
#include <pe/support.h>
using namespace pe;
using namespace pe::povray;
using namespace pe::timing;


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <pe/irrlicht.h>
#if HAVE_IRRLICHT
using namespace pe::irrlicht;
#endif




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
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////
   // Simulation parameter definition

   // Timing parameters
   const std::size_t timesteps ( 15000 );  // Total number of time steps
   const std::size_t inputEnd  ( 12000 );  // End of random object generation
   const std::size_t visspacing(    10 );  // Spacing between two visualizations (POV-Ray & Irrlicht)
   const std::size_t frequency (     1 );  // Number of time steps between two new objects
   const real timestepsize     ( 0.005 );  // Size of a single time step

   // Well parameters
   const real        R ( 30.0 );  // Radius of the well
   const std::size_t N ( 40   );  // Number of bricks per level
   const std::size_t H (  5   );  // Number of levels

   // Randomness
   const real probSphere ( 0.0 );  // Probability for a random sphere
   const real probBox    ( 0.0 );  // Probability for a random box
   const real probCapsule( 0.0 );  // Probability for a random capsule
   const real probTristar( 1.0 );  // Probability for a random tristar union

   // General object parameters
   const real density    ( 1.0  );  // Density of the random objects
   const real restitution( 0.04 );  // Coefficient of restitution for the random objects
   const real friction   ( 0.25 );  // Coefficient of static and dynamic friction
   const real poisson    ( 0.3  );  // Poisson's ratio of the object material
   const real young      ( 300  );  // Young's modulus of the object material
   const real stiffness  ( 1e6  );  // The stiffness in normal direction of the contact region
   const real dampingN   ( 1e5  );  // The damping coefficient in normal direction of the contact region
   const real dampingT   ( 2e5  );  // The damping coefficient in tangential direction of the contact region

   // Sphere parameters
   const real sphereRadiusMin( 0.7 );  // Minimal radius of a random sphere
   const real sphereRadiusMax( 2.0 );  // Maximal radius of a random sphere

   // Box parameters
   const real boxLengthMin( 2.1 );  // Minimal side length of a random box
   const real boxLengthMax( 4.1 );  // Maximal side length of a random box

   // Capsule settings
   const real capsuleLengthMin( 1.1 );  // Minimal length of a random capsule
   const real capsuleLengthMax( 3.1 );  // Maximal length of a random capsule
   const real capsuleRadiusMin( 0.5 );  // Minimal radius of a random capsule
   const real capsuleRadiusMax( 1.1 );  // Maximal radius of a random capsule

   // Visualization
   bool povray  ( true );  // Switches the POV-Ray visualization on and off
   bool irrlicht( true );  // Switches the Irrlicht visualization on and off


   ////////////////////
   // Initial setups

   // Checking the probabilities
   if( probSphere + probBox + probCapsule + probTristar != 1.0 ) {
      std::cerr << pe_RED
                << "\n Invalid probabilities! Please check the probability settings!\n\n"
                << pe_OLDCOLOR;
      pe::exit( EXIT_FAILURE );
   }

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;
   if( vm.count( "no-irrlicht" ) > 0 )
      irrlicht = false;

   // Creating the material for the random rigid bodies
   MaterialID bodyMaterial = createMaterial( "body", density, restitution,
                                             friction, friction, poisson, young,
                                             stiffness, dampingN, dampingT );


   /////////////////////////////////
   // POV-Ray visualization setup

   // Creating colors
   const Color white( 1.0, 1.0, 1.0 );

   // Finish creation
   const Finish finish(
      Ambient( 0.1 ),
      Diffuse( 0.6 ),
      Phong( 1.0 ),
      PhongSize( 50.0 ),
      Reflection( 0.05 )
   );

   const Finish grassFinish(
      Ambient( 0.2 )
   );

   // Normal creation
   const GraniteNormal normal(
      0.6,
      Scale( 2.0 )
   );

   // Texture creation
   const PlainTexture brickTexture(
      ImagePigment( gif, "brick.gif", planar, true ),
      MarbleNormal( 0.7, Scale( 2.0 ) ),
      Scale( 3.5 ),
      Rotation( 0.0, M_PI/2.0, 0.0 )
   );
   const PlainTexture grassTexture(
      ImagePigment( gif, "grass.gif", planar, true ),
      Bumps( 0.8, Scale( 2.9 ) ),
      grassFinish,
      Scale( 20.0 ),
      Rotation( M_PI/2.0, 0.0, 0.0 )
   );

   // Light source setup
   const AreaLight areaLight(
      Vec3( 0.0, -100.0, 80.0 ),      // Global position of the area light
      white,                          // Color of the area light
      Vec3( 1.0, 0.0, 0.0 ),          // First axis
      Vec3( 0.0, 1.81525, 2.38849 ),  // Second axis
      5, 5                            // Number of point light sources
   );

   const ParallelLight parallelLight(
      Vec3( 0.0, 0.0, 75.0 ),    // Global position of the parallel light
      white,                     // Color of the parallel light
      PointAt( 0.0, 0.0, 0.0 ),  // Focus point of the light source
      Shadowless()               // Creates a shadowless light source
   );

   // POV-Ray writer and camera setup
   WriterID pov;

   if( povray )
   {
      // Configuring the POV-Ray writer
      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->include( "colors.inc" );
      pov->include( "stones.inc" );
      pov->setFilename( "video/pic%.pov" );
      pov->setBackground ( white         );
      pov->addLightSource( areaLight     );
      pov->addLightSource( parallelLight );

      // Configuring the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( -6.0, -50.0, 40.0 );
      camera->setFocus   (  0.0,   0.0,  3.0 );
   }

   //////////////////////////////////
   // Irrlicht visualization setup

#if HAVE_IRRLICHT
   if( irrlicht ) {
      ViewerID viewer = activateViewer( opengl, 800, 600 );
      viewer->setSpacing( visspacing );
      viewer->addPointLightSource( -6.0F, -35.0F, 40.0F, 0.6F, 0.6F,  0.6F );
      viewer->addFPSCamera  ( -6.0F, -35.0F, 40.0F, 0.0F, 0.0F, -3.0F );
   }
#else
   UNUSED( irrlicht );
#endif


   ////////////////////////////
   // Simulation world setup

   // Creating and initializing the pe simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );
   world->setDamping( 0.95 );

   // Setup of the ground plane
   PlaneID ground = createPlane( 100, 0.0, 0.0, 1.0, 0.0, granite );
   if( povray ) pov->setTexture( ground, grassTexture );

   // Setup of the well
   const real angle( real(2)*M_PI / N );
   const real sizeX( std::sqrt( real(2)*sq(R)*( real(1) - std::cos(angle) ) ) );
   const real sizeY( 0.5*sizeX );
   const real sizeZ( 0.4*sizeX );

   for( std::size_t h=0; h<H; ++h ) {
      const real z( ( h+real(0.5) ) * sizeZ );
      const real offset( (h%2)?(0.5*angle):(0) );
      for( std::size_t i=0; i<N; ++i ) {
         BoxID brick = createBox( 0, 0.0, R+real(0.5)*sizeY, z, sizeX, sizeY, sizeZ, granite );
         brick->rotateAroundOrigin( 0.0, 0.0, i*angle+offset );
         brick->setFixed( true );
         if( povray ) pov->setTexture( brick, brickTexture );
      }
   }

   std::cout << "\n"
             << "--SIMULATION SETUP--------------------------------------------------------------\n"
             << " Number of bricks contained in the well = " << H*N << "\n"
             << " Seed of the random number generator    = " << getSeed() << "\n"
             << "--------------------------------------------------------------------------------\n"
             << std::endl;


   /////////////////////
   // Simulation loop

   std::size_t counter( frequency );
   std::size_t create ( frequency );
   std::size_t numSpheres ( 0 );
   std::size_t numBoxes   ( 0 );
   std::size_t numCapsules( 0 );
   std::size_t id( 0 );

   WcTimer totalTime;
   WcTimer simTime;

   std::cout << "\n--RIGID BODY SIMULATION---------------------------------------------------------"
             << std::endl;

   totalTime.start();

   for( unsigned int timestep=0; timestep<timesteps; ++timestep, ++counter )
   {
      // Generating a new object
      if( counter == create && timestep < inputEnd )
      {
         const real object  ( rand<real>() );
         const real rotation( rand<real>( 0, real(2.0)*M_PI ) );
         const real arm     ( rand<real>( 0, real(0.8)*R    ) );
         const real posz    ( 50.0 );

         BodyID body( NULL );

         // Creating a new random sphere
         if( object < probSphere ) {
            const real radius( rand<real>( sphereRadiusMin, sphereRadiusMax ) );
            SphereID sphere = createSphere( ++id, 0.0, arm, posz, radius, bodyMaterial );
            sphere->rotateAroundOrigin( 0.0, 0.0, rotation );
            sphere->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
            if( povray ) pov->setTexture( sphere, PlainTexture( ColorPigment( RandomColor() ), finish, normal ) );
            ++numSpheres;
            body = sphere;
         }

         // Creating a new random box
         else if( object < probSphere+probBox ) {
            const real lenx( rand<real>( boxLengthMin, boxLengthMax ) );
            const real leny( rand<real>( boxLengthMin, boxLengthMax ) );
            const real lenz( rand<real>( boxLengthMin, boxLengthMax ) );
            BoxID box = createBox( ++id, 0.0, arm, posz, lenx, leny, lenz, bodyMaterial );
            box->rotateAroundOrigin( 0.0, 0.0, rotation );
            box->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
            if( povray ) pov->setTexture( box, PlainTexture( ColorPigment( RandomColor() ), finish, normal ) );
            ++numBoxes;
            body = box;
         }

         // Creating a new random capsule
         else if( object < probSphere+probBox+probCapsule ) {
            const real radius( rand<real>( capsuleRadiusMin, capsuleRadiusMax ) );
            const real length( rand<real>( capsuleLengthMin, capsuleLengthMax ) );
            CapsuleID capsule = createCapsule( ++id, 0.0, arm, posz, radius, length, bodyMaterial );
            capsule->rotateAroundOrigin( 0.0, 0.0, rotation );
            capsule->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
            if( povray ) pov->setTexture( capsule, PlainTexture( ColorPigment( RandomColor() ), finish, normal ) );
            ++numCapsules;
            body = capsule;
         }

         // Creating a new random tristar union
         else {
            const real radius( rand<real>( capsuleRadiusMin, capsuleRadiusMax ) );
            const real length( rand<real>( capsuleLengthMin, capsuleLengthMax ) );
            UnionID tristar = createTristar( ++id, 0.0, arm, posz, radius, length, bodyMaterial );
            tristar->rotateAroundOrigin( 0.0, 0.0, rotation );
            tristar->rotate( rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI), rand<real>(-M_PI,M_PI) );
            if( povray ) pov->setTexture( tristar, PlainTexture( ColorPigment( RandomColor() ), finish, normal ) );
            numCapsules += 3;
            body = tristar;
         }

         // Checking for overlaps
         if( body != NULL ) {
            for( World::Iterator b=world->begin(); b!=world->end(); ++b ) {
               if( *b != body && overlap( *b, body ) ) {
                  destroy( body );
                  break;
               }
            }
         }

         create  = rand<std::size_t>()%frequency + 1;
         counter = 0;
      }

      std::cout << "\r Simulating time step " << timestep+1
                << " , spheres: "  << numSpheres
                << " , boxes: "    << numBoxes
                << " , capsules: " << numCapsules
                << std::flush;

      simTime.start();
      world->simulationStep( timestepsize );
      simTime.end();
   }

   totalTime.end();

   std::cout << "\n\n"
             << " Final number of spheres   : " << numSpheres << "\n"
             << " Final number of boxes     : " << numBoxes << "\n"
             << " Final number of capsules  : " << numCapsules << "\n"
             << " WC-Time:  Pure simulation : " << simTime.total() << "\n"
             << "           Total time      : " << totalTime.total() << "\n";
   std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
}
//*************************************************************************************************
