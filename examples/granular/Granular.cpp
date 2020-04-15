//=================================================================================================
/*!
 *  \file Granular.cpp
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

#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <iostream>
#include <sstream>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;


//*************************************************************************************************
// Irrlicht includes
//*************************************************************************************************

#include <pe/irrlicht.h>
#if HAVE_IRRLICHT
using namespace pe::irrlicht;
#endif




//=================================================================================================
//
//  POVRAY CAMERA ANIMATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Animation for the POV-Ray camera.
 *
 * This class animates the POV-Ray camera for the Granular example. The camera initially focuses
 * on the granular media outlet before slowly shifting the focus to the well at the bottom of the
 * simulation.
 */
class LinearFocusShift : public CameraAnimation
{
 public:
   explicit LinearFocusShift( size_t first, size_t last, const Vec3& begin, const Vec3& end )
      : first_   ( first )  // First time step of the camera animation
      , last_    ( last  )  // Last time step of the camera animation
      , begin_   ( begin )  // Initial focus point of the camera
      , end_     ( end   )  // Final focus point of the camera
   {}

   virtual ~LinearFocusShift()
   {}

   virtual void updateFocus( Vec3& focus )
   {
      if( TimeStep::step() > first_ && TimeStep::step() < last_ ) {
         const Vec3& disp( ( begin_ - end_ ) / ( last_ - first_ ) );
         focus -= disp;
      }
   }

 private:
   size_t first_;  // First time step of the camera animation
   size_t last_;   // Last time step of the camera animation
   Vec3 begin_;    // Initial focus point of the camera
   Vec3 end_;      // Final focus point of the camera
};
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY TEXTURE POLICY
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Texture policy for the granular medium example.
 *
 * This class represents the POV-Ray texture policy for the granular medium example.
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
         case 3:  return CustomTexture( "GlassTexture"     );  // Texture for the outlet
         default: return CustomTexture( "BrickTexture"     );  // Texture for well bricks
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
/*!\brief Main function for the granular example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // Simulation parameters

   // Particle parameters
   const bool   spheres ( false );  // Switch between spheres and granular particles
   const real   radius  (  0.45 );  // The radius of spheres of the granular media
   const real   spacing (  0.05 );  // Initial spacing inbetween two spheres
   const real   velocity(  0.01 );  // Initial maximum velocity of the spheres

   // Environment parameters
   const real   lx     (   30.0 );  // Size of the outlet in x-dimension
   const real   ly     (    5.0 );  // Size of the outlet in y-dimension
   const real   lz     (   60.0 );  // Size of the outlet in z-dimension
   const real   width  (    0.7 );  // Width of the outlet walls
   const real   opening(    5.0 );  // Size of the outlet door
   const real   angle  ( M_PI/8 );  // Angle of the lower outlet walls
   const real   disp   (  -18.0 );  // Displacement of the ground plane

   // Well parameters
   const size_t B (  30  );  // Number of bricks per level
   const size_t H (   3  );  // Number of levels
   const real   R ( 16.0 );  // Radius of the well

   // Time parameters
   const size_t initsteps(  20000 );  // Initialization steps with closed outlet door
   const size_t opensteps(   2000 );  // Number of time steps for the door opening
   const size_t timesteps( 180000 );  // Number of time steps for the flowing granular media
   const real   stepsize (  0.001 );  // Size of a single time step

   // Visualization
   bool povray  ( true );  // Switches the POV-Ray visualization on and off
   bool irrlicht( true );  // Switches the Irrlicht visualization on and off

   // Visualization parameters
   const size_t visspacing(   150 );  // Number of time steps inbetween two POV-Ray files
   const size_t startanim ( 72000 );  // Initial time step for the camera animation
   const size_t colorwidth(     8 );  // Number of spheres in z-dimension with a specific color


   /////////////////////////////////////////////////////
   // Initial setups

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;
   if( vm.count( "no-irrlicht" ) > 0 )
      irrlicht = false;

   MaterialID granular = createMaterial( "granular", 1.0, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );
   world->setDamping( 0.9 );


   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;

   if( povray )
   {
      // Calculation of the camera location and initial and final focus point
      const Vec3 location( 0.3*lx, -0.65*lz, 0.2*lz  );
      const Vec3 focus1  ( 0.0   ,  0.0    , 0.15*lz );
      const Vec3 focus2  ( 0.0   ,  0.0    , 0.0     );

      // Configuration of the POV-Ray writer
      pov = activateWriter();
      pov->setStart( initsteps/2 );
      pov->setSpacing( visspacing );
      pov->setFilename( "video/pic%.pov" );
      pov->include( "settings.inc" );
      pov->setBackground( Color( 0, 0, 0 ) );
      pov->setTexturePolicy( GranularTexturePolicy() );

      // Configuration of the scene lighting
      pov->addLightSource( PointLight   ( location         , Color( 0.5, 0.5, 0.5 ) ) );
      pov->addLightSource( ParallelLight( Vec3( 0, -50, 0 ), Color( 1.0, 1.0, 1.0 ) ) );
      pov->addLightSource( ParallelLight( Vec3( 0, 0, 100 ), Color( 1.0, 1.0, 1.0 ), Shadowless() ) );

      // Configuration of the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( location );
      camera->setFocus   ( focus1   );
      camera->animate    ( LinearFocusShift( startanim, initsteps+timesteps, focus1, focus2 ) );
   }


   //////////////////////////////////
   // Irrlicht visualization setup

#if HAVE_IRRLICHT
   if( irrlicht ) {
      ViewerID viewer = activateViewer( opengl, 800, 600 );
      viewer->setSpacing( visspacing );
      viewer->addPointLightSource( 0.3*lx, -0.65*lz, 0.2*lz, 0.6F, 0.6F, 0.6F  );
      viewer->addFPSCamera  ( 0.3*lx, -0.65*lz, 0.2*lz, 0.0, 0.0, 0.15*lz );
   }
#else
   UNUSED( irrlicht );
#endif


   /////////////////////////////////////////////////////
   // Setup of the simulation domain

   // Creating the ground plane
   createPlane( 2, 0.0, 0.0, 1.0, disp, granular );

   // Creating the outlet union
   BoxID lid( 0 );   // Handle for the outlet lid
   Vec3 pivotPoint;  // Pivot point for the outlet lid

   pe_CREATE_UNION( outlet, 3 )
   {
      const real sizex( 0.5*lx - 0.5*opening + width );
      const real sizez( sizex*std::tan(angle) );
      const real boxx ( sizex/std::cos(angle) );

      // Creating the front wall
      createBox( 3, Vec3( 0.0, -0.5*(ly+width), 0.5*(lz-sizez) ),
                    Vec3( lx+2.0*width, width, lz+sizez ), granular, false );

      // Creating the back wall
      createBox( 3, Vec3( 0.0, 0.5*(ly+width), 0.5*(lz-sizez) ),
                    Vec3( lx+2.0*width, width, lz+sizez ), granular );

      // Creating the upper left wall
      createBox( 3, Vec3( -0.5*(lx+width), 0.0, 0.5*lz ),
                    Vec3( width, ly+width, lz ), granular );

      // Creating the lower left wall
      BoxID left = createBox( 3, 0, 0, 0, boxx, ly+width, width, granular );
      left->rotate( 0, angle, 0 );
      const Vec3 offset1( left->pointFromBFtoWF( -0.5*boxx, 0, -0.5*width ) );
      left->setPosition( Vec3( -0.5*lx-width, 0, 0 ) - offset1 );

      // Creating the upper right wall
      createBox( 3, Vec3( 0.5*(lx+width), 0.0, 0.5*lz ),
                    Vec3( width, ly+width, lz ), granular );

      // Creating the lower right wall
      BoxID right = createBox( 3, 0, 0, 0, boxx, ly+width, width, granular );
      right->rotate( 0, -angle, 0 );
      const Vec3 offset2( right->pointFromBFtoWF( 0.5*boxx, 0, -0.5*width ) );
      right->setPosition( Vec3( 0.5*lx+width, 0, 0 ) - offset2 );

      // Creating the lid
      const Vec3 offset3 ( left->pointFromBFtoWF (  0.5*boxx, 0, -0.5*width ) );
      const Vec3 offset4 ( right->pointFromBFtoWF( -0.5*boxx, 0, -0.5*width ) );
      lid = createBox( 3, Vec3( 0, 0, offset3[2]+0.5*width ),
                          Vec3( offset4[0] - offset3[0], ly+width, width ), granular );

      // Fixing the global position of the outlet
      outlet->setFixed( true );

      // Calculating the pivot point of the lid
      pivotPoint = left->pointFromBFtoWF ( 0.5*boxx, 0, 0.5*width );
      pivotPoint[0] = -0.5*opening;
   }

   // Setup of the round well
   const real brickangle( real(2)*M_PI / B );
   const real brickX( std::sqrt( real(2)*sq(R)*( real(1) - std::cos(brickangle) ) ) );
   const real brickY( 0.5*brickX );
   const real brickZ( 0.4*brickX );

   for( std::size_t h=0; h<H; ++h ) {
      const real z( ( h+real(0.5) ) * brickZ + disp );
      const real offset( (h%2)?(0.5*brickangle):(0) );
      for( std::size_t i=0; i<B; ++i ) {
         BoxID brick = createBox( 4, 0.0, R+real(0.5)*brickY, z, brickX, brickY, brickZ, granite );
         brick->rotateAroundOrigin( 0.0, 0.0, i*brickangle+offset );
         brick->setFixed( true );
      }
   }

   // Setup of the spheres
   const real space( 2.0*radius+spacing );
   const size_t nx( lx / space );
   const size_t ny( ly / space );
   const size_t nz( lz / space );
   const real   dx( lx / nx );
   const real   dy( ly / ny );
   const real   dz( lz / nz );

   BodyID particle;

   for( size_t i=0; i<nz; ++i ) {
      for( size_t j=0; j<ny; ++j ) {
         for( size_t k=0; k<nx; ++k )
         {
            const size_t uid( (i/colorwidth)%2 );
            const Vec3 gpos( -0.5*lx+0.5*dx+k*dx, -0.5*ly+0.5*dy+j*dy, width+0.5*dz+i*dz );
            const Vec3 vel ( rand<real>( -velocity, velocity ),
                             rand<real>( -velocity, velocity ),
                             rand<real>( -velocity, 0.0 ) );

            if( spheres ) particle = createSphere( uid, gpos, radius, granular );
            else particle = createGranularParticle( uid, gpos, radius, granular );
            particle->setLinearVel( vel );
         }
      }
   }


   /////////////////////////////////////////////////////
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "--------------------------------------------------------------\n"
                << " Total number of particles               = " << nx*ny*nz << "\n"
                << " Total number of bricks in the well      = " << H*B << "\n"
                << " Number of initialization steps          = " << initsteps << "\n"
                << " Number of opening steps                 = " << opensteps << "\n"
                << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << std::endl;
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


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
   for( size_t timestep=0; timestep<initsteps; ++timestep )
   {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\r Simulating initialization step " <<  timestep+1 << "   " << std::flush;
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n";
   }

   // Opening the door of the granular media outlet
   for( size_t timestep=0; timestep<opensteps; ++timestep )
   {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\r Simulating opening step " <<  timestep+1 << "   " << std::flush;
      }

      lid->rotateAroundPoint( pivotPoint, Vec3( 0, M_PI/(2*opensteps), 0 ) );
      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n";
   }

   // Running the granular media simulation
   for( size_t timestep=0; timestep<timesteps; ++timestep )
   {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\r Simulating time step " <<  timestep+1 << "   " << std::flush;
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   totalTime.end();

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n\n"
                << " WC-Time:  Pure simulation : " << simTime.total() << "\n"
                << "           Total time      : " << totalTime.total() << "\n";
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   return 0;
}
//*************************************************************************************************
