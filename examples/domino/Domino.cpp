//=================================================================================================
/*!
 *  \file Domino.cpp
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
#include <sstream>
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
/*!\brief Main function for the domino example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char* argv[] )
{
   /////////////////////////////////////
   // Simulation parameter definition

   // Timing parameters
   const size_t timesteps ( 10000 );  // Total number of time steps
   const size_t visspacing(   20 );  // Spacing between two visualizations (POV-Ray & Irrlicht)

   // Domino parameters
   const size_t dominos   (   50 );  // Total number of dominos

   // Visualization
   bool povray  ( true );  // Switches the POV-Ray visualization on and off
   bool irrlicht( true );  // Switches the Irrlicht visualization on and off


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


   ////////////////////////////
   // Simulation world setup

   // Creating and initializing the pe simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -9.81 );
   world->setDamping( 0.9 );

   // Setup of the ground plane
   PlaneID plane = createPlane( 0, 0.0, 0.0, 1.0, 0.0, granite );

   // Dimensions of a single domino
   const real sizeX( 2.0 );
   const real sizeY( 1.0 );
   const real sizeZ( 4.0 );

   // Initial parameters for the spiral
   const real minDistance( 0.75*sizeZ );
   const real phiStep( 0.00008 );

   size_t id ( 0 );
   real   phi( 0.5 );
   real   a  ( 2.0*sizeZ );
   real   r  ( 0.0 );

   Vec3 oldPosition( 0.0, 0.0, 0.5*sizeZ );
   Vec3 newPosition( 0.0, 0.0, 0.5*sizeZ  );

   // Creating the domino spiral
   while( id < dominos )
   {
      oldPosition = newPosition;

      // Calculating the new position
      while( ( newPosition - oldPosition ).length() < minDistance )
      {
         phi += phiStep;
         r = a * phi;
         newPosition[0] = r * std::cos( 2.0*M_PI*phi );
         newPosition[1] = r * std::sin( 2.0*M_PI*phi );
      }

      // Placing the domino at the new position
      BoxID domino = createBox( ++id, newPosition, sizeX, sizeY, sizeZ, oak );
      domino->rotate( Vec3( 0.0, 0.0, 1.0 ), 2.0*M_PI*phi );
   }

   // Initial parameter for the pendulum
   real height( 3.0*sizeZ );

   // Creating the balks
   UnionID gibbet = createUnion( ++id );
   BoxID balk;

   balk = createBox( ++id, 2.0, 0.0, 0.5*height, sizeY, sizeY, height, oak );
   gibbet->add( balk );
   balk = createBox( ++id, -1.5*sizeY, 0.0, height - 0.5*sizeY, 0.5*height, sizeY, sizeY, oak );
   gibbet->add( balk );
   gibbet->setFixed( true );

   SphereID ball = createSphere( ++id, -4.0, 7.5, 11.0, sizeY, oak );

   attachSpring( balk, Vec3( -2.25, 0.0, -0.5 ), ball, Vec3( 0.0, 0.0, 0.0 ), 1000.0, 100.0 );


   /////////////////////////////////
   // POV-Ray visualization setup

   if( povray )
   {
      // Light source setup
      SpotLight spotlight(
         Vec3   ( r/2.0, r/4.0, 2.5*r ),  // Position of the spotlight
         Color  ( 0.9  , 0.9  , 0.9   ),  // Color of the spotlight
         PointAt( 0.0  , 0.0  , 0.0   )   // Focus point of the spotlight
      );

      // Configuring the POV-Ray writer
      WriterID pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->include( "domino.inc" );
      pov->setFilename( "./video/pic%.pov" );
      pov->setBackground( 1.0, 1.0, 1.0 );
      pov->addLightSource( spotlight );

      // Configuring the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( 1.2*r, 1.2*r, 1.2*r );
      camera->setFocus   ( 0.0, 0.0, 0.0 );

      // Setting the ground plane texture
      pov->setTexture( plane, CustomTexture( "FloorTexture" ) );

      // Setting the gibbet texture
      pov->setTexture( gibbet, CustomTexture( "GibbetTexture" ) );

      // Setting the ball texture
      pov->setTexture( ball, CustomTexture( "BallTexture" ) );

      // Setting the textures of the dominos
      std::ostringstream oss;

      for( World::Bodies::CastIterator<Box> b=world->begin<Box>(); b!=world->end<Box>(); ++b )
      {
         // Setting the domino background
         CustomTexture background( "DominoBackground" );

         // Selecting the first number
         oss.str( "" );
         oss << "Texture" << rand<int>( 0, 6 ) << " translate <-" << sizeY << ",0,-0.2>";
         CustomTexture up( oss.str() );

         // Adding the separator
         oss.str( "" );
         oss << "Separator translate <-" << sizeY << ",0,0>";
         CustomTexture sep( oss.str() );

         // Selecting the Second number
         oss.str( "" );
         oss << "Texture" << rand<int>( 0, 6 ) << " translate <-" << sizeY << ",0,1.5>";
         CustomTexture down( oss.str() );

         // Setting the texture
         pov->setTexture( *b, LayeredTexture( background, up, sep, down ) );
      }
   }


   //////////////////////////////////
   // Irrlicht visualization setup

#if HAVE_IRRLICHT
   if( irrlicht ) {
      ViewerID viewer = activateViewer( opengl, 800, 600 );
      viewer->addPointLightSource( r/2.0, r/4.0, 2.5*r, 0.6F, 0.6F, 0.6F );
      viewer->addFPSCamera  ( 1.2*r, 1.2*r, 1.2*r, 0.0F, 0.0F, 0.0F );
   }
#else
   UNUSED( irrlicht );
#endif


   /////////////////////
   // Simulation loop

   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "------------------------------------------------------" << std::endl;

   WcTimer simTime;

   for( size_t timestep=0; timestep<timesteps; ++timestep ) {
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;
      simTime.start();
      world->simulationStep( 0.005 );
      simTime.end();
   }

   std::cout << "\n"
             << " Total WC-Time        : " << simTime.total() << "\n"
             << " Average per time step: " << simTime.average() << "\n";
   std::cout << "\n--------------------------------------------------------------------------------\n"
             << std::endl;

   return 0;
}
//*************************************************************************************************
