//=================================================================================================
/*!
 *  \file Shaker.cpp
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
#include <pe/core.h>
#include <pe/support.h>
#include <pe/povray.h>
#include <pe/util.h>
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

#if !MOBILE_INFINITE
#  error "The engine is not compiled with mobile infinite bodies. Add -DMOBILE_INFINITE to the compiler flags and recompile pe."
#endif




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the shaker example.
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
   const size_t timesteps ( 6000 );  // Total number of time steps
   const size_t visspacing(    2 );  // Spacing between two visualizations (POV-Ray & Irrlicht)

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

   /////////////////////////////////
   // POV-Ray visualization setup
   WriterID pov;
   if( povray )
      pov = activateWriter();

   ////////////////////////////
   // Simulation world setup

   // Creating and initializing the pe simulation world
   size_t uid( 0 );
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -22.0 );
   //world->setDamping( 0.9 );

   // Setup of the ground plane
   PlaneID plane = createPlane( uid++, 0.0, 0.0, 1.0, 0.0, granite );

   MaterialID materialWall    = createMaterial( "wall",   1.0, 0.0, 0.2, 0.2, 0.5, 200, 1e6, 1e5, 2e5 );
   MaterialID materialObjects = createMaterial( "object", 1.0, 0.0, 0.2, 0.2, 0.5, 200, 1e6, 1e5, 2e5 );

   // Dimensions of the shaker
   const real sizeX( 2.0 );
   const real sizeY( 1.0 );
   const real sizeZ( 1.0 );

   // Thickness of the shaker walls
   const real sizeWall( 0.1 );

   // Number of objects in each dimension
   const size_t nX( 20 );
   const size_t nY( 10 );
   const size_t nZ( 10 );

   // Object aa-boundingbox sizes
   const real sizeObjectX( ( sizeX - 2 * sizeWall ) / nX );
   const real sizeObjectY( ( sizeY - 2 * sizeWall ) / nY );
   const real sizeObjectZ( ( sizeObjectX + sizeObjectY ) * 0.5 );

   // Generate shaker walls
   /*
   BoxID wallNorth  = createBox( uid++, 0.5 * sizeX, sizeY - 0.5 * sizeWall, 0.5 * sizeZ, sizeX, sizeWall, sizeZ, materialWall, true );
   BoxID wallEast   = createBox( uid++, sizeX - 0.5 * sizeWall, 0.5 * sizeY, 0.5 * sizeZ, sizeWall, sizeY, sizeZ, materialWall, true );
   BoxID wallSouth  = createBox( uid++, 0.5 * sizeX, 0.5 * sizeWall, 0.5 * sizeZ,         sizeX, sizeWall, sizeZ, materialWall, true );
   BoxID wallWest   = createBox( uid++, 0.5 * sizeWall, 0.5 * sizeY, 0.5 * sizeZ,         sizeWall, sizeY, sizeZ, materialWall, true );
   BoxID wallBottom = createBox( uid++, 0.5 * sizeX, 0.5 * sizeY, 0.5 * sizeWall,         sizeX, sizeY, sizeWall, materialWall, true );
   if( povray ) {
      pov->setTexture( wallNorth,  CustomTexture( "TextureContainerN" ) );
      pov->setTexture( wallEast,   CustomTexture( "TextureContainerE" ) );
      pov->setTexture( wallSouth,  CustomTexture( "TextureContainerS" ) );
      pov->setTexture( wallWest,   CustomTexture( "TextureContainerW" ) );
      pov->setTexture( wallBottom, CustomTexture( "TextureContainerB" ) );
   }
   */
   PlaneID wallNorth  = createPlane( uid++, 0, -1, 0, -(sizeY - sizeWall), materialWall, false );
   PlaneID wallEast   = createPlane( uid++, -1, 0, 0, -(sizeX - sizeWall), materialWall, false );
   PlaneID wallSouth  = createPlane( uid++, 0,  1, 0, sizeWall,         materialWall, false );
   PlaneID wallWest   = createPlane( uid++,  1, 0, 0, sizeWall,         materialWall, false );
   PlaneID wallBottom = createPlane( uid++, 0,  0, 1, sizeWall,         materialWall, false );
   if( povray ) {
      pov->setTexture( wallNorth,  CustomTexture( "TextureContainerN" ) );
      pov->setTexture( wallEast,   CustomTexture( "TextureContainerE" ) );
      pov->setTexture( wallSouth,  CustomTexture( "TextureContainerS" ) );
      pov->setTexture( wallWest,   CustomTexture( "TextureContainerW" ) );
      pov->setTexture( wallBottom, CustomTexture( "TextureContainerB" ) );
   }
   wallNorth ->setFixed( true );
   wallEast  ->setFixed( true );
   wallSouth ->setFixed( true );
   wallWest  ->setFixed( true );
   wallBottom->setFixed( true );

   // Generate objects
   real minrad = sizeX;
   for( size_t iZ = 0; iZ < nZ; ++iZ ) {
      for( size_t iY = 0; iY < nY; ++iY ) {
         for( size_t iX = 0; iX < nX; ++iX ) {
            const real radius( min( sizeObjectX, min( sizeObjectY, sizeObjectZ ) ) * 0.5 );
            const real shrink( 0.9 ), perturbShrink( rand( -0.1, 0.1 ) );
            //const real margin( 0 );
            //const real shrink( 0.2 ), perturbShrink( 0 );
            const real margin( 1 - ( shrink + perturbShrink ) );

            const real perturbX( rand( -radius * margin, radius * margin ) );
            const real perturbY( rand( -radius * margin, radius * margin ) );
            const real perturbZ( rand( -radius * margin, radius * margin ) );
            
            const real boundingradius( ( shrink + perturbShrink ) * radius );

            SphereID object = createSphere( uid++, sizeWall + sizeObjectX * ( 0.5 + iX ) + perturbX, sizeWall + sizeObjectY * ( 0.5 + iY ) + perturbY, sizeZ + sizeObjectZ * ( 0.5 + iZ ) + perturbZ, boundingradius, materialObjects, true );
            minrad = min( boundingradius, minrad );

            /*
            const real cylradius( boundingradius * std::sin( M_PI * 25.0 / 180.0 ) );
            const real cyllength( ( boundingradius - cylradius ) * 2 );
            minrad = min( cylradius, minrad );

            CapsuleID object = createCapsule( uid++, sizeWall + sizeObjectX * ( 0.5 + iX ) + perturbX, sizeWall + sizeObjectY * ( 0.5 + iY ) + perturbY, sizeZ + sizeObjectZ * ( 0.5 + iZ ) + perturbZ, cylradius, cyllength, materialObjects, true );

            // roll (x), pitch (y), yaw (z)
            object->rotate( rand( -M_PI, M_PI ), 0.5 * rand( -M_PI, M_PI ), rand( -M_PI, M_PI ) );
            */

            /*
            const real boxlen( boundingradius * ( 2 / sqrt(3) ) );
            minrad = min( boundingradius * 0.5, minrad );
            BoxID object = createBox( uid++, sizeWall + sizeObjectX * ( 0.5 + iX ) + perturbX, sizeWall + sizeObjectY * ( 0.5 + iY ) + perturbY, sizeZ + sizeObjectZ * ( 0.5 + iZ ) + perturbZ, boxlen, boxlen, boxlen, materialObjects, true );
            object->rotate( rand( -M_PI, M_PI ), 0.5 * rand( -M_PI, M_PI ), rand( -M_PI, M_PI ) );
            */

            if( povray ) {
               std::ostringstream textureName;
               textureName << "Texture" << rand( 0, 18 );
               pov->setTexture( object, CustomTexture( textureName.str() ) );
            }
         }
      }
   }

   std::cout << "minrad = " << minrad << std::endl;

   /////////////////////////////////
   // POV-Ray visualization setup

   if( povray )
   {
      // Light source setup
      SpotLight spotlight(
         Vec3   ( sizeX * 1.2, sizeY * 1.5, ( sizeZ + sizeObjectZ * nZ ) * 2.0 ),  // Position of the spotlight
         Color  ( 0.9, 0.9, 0.9   ),  // Color of the spotlight
         PointAt( sizeX * 0.5, sizeY * 0.5, sizeZ * 0.5 )   // Focus point of the spotlight
      );
      
      ParallelLight sun(
         Vec3( 2, 1, 2 ),
         Color( 1, 1, 1 ),
         PointAt( 1, 0, 0.5 )
      );

      // Configuring the POV-Ray writer
      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->include( "shaker.inc" );
      pov->setFilename( "video/pic%.pov" );
      pov->setBackground( 1.0, 1.0, 1.0 );
      pov->addLightSource( spotlight );
      pov->addLightSource( sun );

      // Configuring the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( sizeX, sizeY * 2.0, ( sizeZ + sizeObjectZ * nZ ) * 0.5 );
      camera->setFocus   ( sizeX * 0.5, sizeY * 0.5, sizeZ * 0.5 );

      // Setting the ground plane texture
      pov->setTexture( plane, CustomTexture( "FloorTexture" ) );
   }


   //////////////////////////////////
   // Irrlicht visualization setup

#if HAVE_IRRLICHT
   if( irrlicht ) {
      ViewerID viewer = activateViewer( opengl, 800, 600 );
      viewer->addPointLightSource( sizeX * 1.2, sizeY * 1.5, ( sizeZ + sizeObjectZ * nZ ) * 2.0, 0.6F, 0.6F, 0.6F );
      viewer->addFPSCamera( sizeX * 1.5, sizeY * 0.7, ( sizeZ + sizeObjectZ * nZ ) * 0.75, 0.0F, 0.0F, 0.0F, 0.05F );
   }
#else
   UNUSED( irrlicht );
#endif


   /////////////////////
   // Simulation loop

   std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
             << "------------------------------------------------------" << std::endl;

   WcTimer simTime;
   const real dt( 0.001 );
   const real f( 10.0 );
   const real hub( sizeWall );
   const real t0( 0.6 );
   //const real t0( 1.2 );
   real t, pos( 0 );

   for( size_t timestep=0; timestep<timesteps; ++timestep ) {
      t = timestep * dt;
      std::cout << "\r Time step " << timestep+1 << " of " << timesteps << "   " << std::flush;

      if( t >= t0 ) {
         // start shaking the container
         const real posNext( hub * 0.5 * ( std::sin( ( t - t0 ) * 2.0 * M_PI * f + 1.5 * M_PI ) + 1.0 ) );
         const real v( ( posNext - pos ) / dt );
         wallNorth ->setLinearVel( 0, 0, v );
         wallEast  ->setLinearVel( 0, 0, v );
         wallSouth ->setLinearVel( 0, 0, v );
         wallWest  ->setLinearVel( 0, 0, v );
         wallBottom->setLinearVel( 0, 0, v );
         pos = posNext;
      }

      /*
      real maxvel = 0;
      for (pe::World::Iterator i = world->begin(); i != world->end(); ++i) {
         maxvel = max( i->getLinearVel().length(), maxvel );
      }
      std::cout << "maxvel: " << maxvel << std::endl;
      std::cout << "maxdt: " << minrad * 0.5 / maxvel << std::endl;
      */

      if( povray ) {
         pov->undeclare( "container_position" );
         pov->declare( "container_position", wallBottom->getPosition() );
      }
      simTime.start();
      world->simulationStep( dt );
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
