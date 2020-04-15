//=================================================================================================
/*!
 *  \file Chain.cpp
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
#include <cstring>
#include <iostream>
#include <pe/engine.h>
#include <pe/support.h>
using namespace pe;
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
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the chain example.
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
   const size_t timesteps   ( 10000 );  // Total number of time steps
   const size_t visspacing  (    50 );  // Spacing between two visualizations (POV-Ray & Irrlicht)
   const real   timestepsize( 0.002 );  // Size of a single time step

   // Chain parameters
   const size_t N ( 3   );  // Total number of chain links
   const real   L ( 8.0 );  // Length of a single chain link
   const real   W ( 4.0 );  // Width of a single chain link
   const real   R ( 0.5 );  // Thickness of the chain link
   const real   Z ( 5.0 );  // Initial height of the chain

   // Visualization
   bool povray  ( true );  // Switches the POV-Ray visualization on and off
   bool irrlicht( true );  // Switches the Irrlicht visualization on and off

   // POV-Ray options
   const Vec3 location( -3.0, -15.0, 20.0 );  // Fixed global location of the POV-Ray camera
   const Vec3 focus   (  0.0,   0.0,  2.0 );  // Focus point of the POV-Ray camera


   ////////////////////
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


   /////////////////////////////////
   // POV-Ray visualization setup

   // Color setup
   const Color white( 1.0, 1.0, 1.0 );

   // Light source setup
   const AreaLight areaLight(
      Vec3( -3.0, -15.0, 100.0 ),     // Global position of the area light
      white,                          // Color of the area light
      Vec3( 1.0, 0.0, 0.0 ),          // First axis
      Vec3( 0.0, 1.81525, 2.38849 ),  // Second axis
      5, 5                            // Number of point light sources
   );

   const SpotLight spotLight(
      Vec3( -3.0, -3.0, 40.0 ),  // Global position of the spotlight
      white,                     // Color of the spotlight
      PointAt( 0.0, 0.0, 0.0 ),  // Focus point
      Radius ( 0.2618  ),        // Radius of the spotlight light cone
      Falloff( 0.43633 )         // Falloff radius of the spotlight light cone
   );

   // Texture setup
   const PlainTexture planksTexture(
      ImagePigment( jpeg, "planks.jpg", planar, true ),
      Scale( 100.0 ),
      Rotation( M_PI/2.0, 0.0, 0.0 ),
      Translation( 40.0, 80.0, 0.0 )
   );

   const CustomTexture chainTexture(
      "T_Chrome_1A normal { granite 0.4 scale 0.15 } finish { ambient 0.2 reflection 0.2 }"
   );

   // POV-Ray writer and camera setup
   WriterID pov;

   if( povray )
   {
      // Configuring the POV-Ray writer
      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->include( "colors.inc" );
      pov->include( "metals.inc" );
      pov->include( "textures.inc" );
      pov->setFilename( "./video/pic%.pov" );
      pov->setBackground ( white     );
      pov->addLightSource( areaLight );
      pov->addLightSource( spotLight );

      // Configuring the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( location );
      camera->setFocus   ( focus    );
   }


   //////////////////////////////////
   // Irrlicht visualization setup

#if HAVE_IRRLICHT
   if( irrlicht ) {
      ViewerID viewer = activateViewer( opengl, 800, 600 );
      viewer->setSpacing( visspacing );
      viewer->addPointLightSource( -3.0F, -15.0F, 20.0F, 0.3F, 0.3F,  0.3F );
      viewer->addFPSCamera  ( -3.0F, -15.0F, 20.0F, 0.0F, 0.0F, -3.0F );
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
   if( povray ) pov->setTexture( ground, planksTexture );

   // Setup of the chain
   const real disp( L - real(3)*R );
   const real totalLength( L + (N-1)*disp );

   Vec3 gpos( ( L - totalLength ) / real(2), 0.0, Z );
   real rot( M_PI/4.0 );

   for( size_t i=1; i<=N; ++i )
   {
      // Creating a new chain link
      UnionID link = createChainLink( i, gpos, R, L, W, iron );
      link->rotate( rot, 0.0, 0.0 );
      link->setLinearVel ( rand<real>(-0.3,0.3), rand<real>(-0.3,0.3), rand<real>( 0.0,0.3) );
      link->setAngularVel( rand<real>(-0.2,0.2), rand<real>(-0.2,0.2), rand<real>(-0.2,0.2) );
      if( povray ) pov->setTexture( link, chainTexture );

      // Adapting the global position and rotation
      gpos[0] += disp;      // Incrementing the global position
      rot     += M_PI/2.0;  // Incrementing the rotation
   }


   /////////////////////
   // Simulation loop

   std::cout << "\n--RIGID BODY SIMULATION---------------------------------------------------------\n"
             << " Simulation seed = " << getSeed() << "\n";

   for( size_t timestep=0; timestep<timesteps; ++timestep ) {
      std::cout << "\r Simulating time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
      world->simulationStep( timestepsize );
   }

   std::cout << "\n--------------------------------------------------------------------------------\n" << std::endl;
}
//*************************************************************************************************
