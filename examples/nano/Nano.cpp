//=================================================================================================
/*!
 *  \file Nano.cpp
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
#include <cmath>
#include <cstddef>
#include <iostream>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the nano example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // Simulation parameters

   const real   radius    ( 0.5  );    // The radius of spherical particles
   const real   spacing   ( 2.0  );    // Initial spacing inbetween two spherical particles
   const real   velocity  ( 0.02 );    // Initial maximum velocity of the spherical particles

   const size_t timesteps ( 100000 );   // Total number of time steps
   const real   stepsize  (   0.01 );   // Size of a single time step

   const size_t seed      ( 12345 );  // Seed for the random number generation

   const bool   povray    ( false );  // Switches the POV-Ray visualization on and off
   const size_t visspacing(    10 );  // Number of time steps inbetween two POV-Ray files


   /////////////////////////////////////////////////////
   // Initial setups

   // Checking the ratio of the particle radius and the spacing
   if( real(2.1)*radius >= spacing ) {
      std::cerr << pe_RED << "\n Invalid particle/spacing ratio!\n\n" << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the number of command line arguments
   if( argc != 4 ) {
      std::cerr << pe_RED << " Invalid number of command line arguments!\n\n" << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   const int nx( atoi( argv[1] ) );
   const int ny( atoi( argv[2] ) );
   const int nz( atoi( argv[3] ) );

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

   const real lx( nx * spacing );  // Length of the simulation domain in x-dimension
   const real ly( ny * spacing );  // Length of the simulation domain in y-dimension
   const real lz( nz * spacing );  // Length of the simulation domain in z-dimension

   setSeed( seed );  // Setup of the random number generation

   MaterialID elastic = createMaterial( "elastic", 1.0, 1.0, 0.05, 0.05, 0.5, 0.1, 1e6, 1e5, 2e5 );

   WorldID world = theWorld();


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
      pov->setTexturePolicy( DefaultTexture( CustomTexture( "ObjectTexture" ) ) );
   }


   /////////////////////////////////////////////////////
   // Setup of the simulation domain

   createPlane( 0,  1.0,  0.0,  0.0, 0.0, elastic );
   createPlane( 0, -1.0,  0.0,  0.0, -lx, elastic );
   createPlane( 0,  0.0,  1.0,  0.0, 0.0, elastic, false );
   createPlane( 0,  0.0, -1.0,  0.0, -ly, elastic );
   createPlane( 0,  0.0,  0.0,  1.0, 0.0, elastic );
   createPlane( 0,  0.0,  0.0, -1.0, -lz, elastic );


   /////////////////////////////////////////////////////
   // Deterministic setup of the particles

   Vec3 gpos, vel;
   size_t id( 0 );

   // Starting the time measurement for the setup
   WcTimer setupTime;
   setupTime.start();

   // Setup of the particles
   for( int z=0; z<nz; ++z ) {
      for( int y=0; y<ny; ++y ) {
         for( int x=0; x<nx; ++x )
         {
            gpos.set( ( x+real(0.5) ) * spacing,
                      ( y+real(0.5) ) * spacing,
                      ( z+real(0.5) ) * spacing );
            vel.set( rand<real>( -velocity, velocity ),
                     rand<real>( -velocity, velocity ),
                     rand<real>( -velocity, velocity ) );

            SphereID particle = createSphere( id, gpos, radius, elastic );
            particle->setLinearVel( vel );
         }
      }
   }

   // Ending the time measurement for the setup
   setupTime.end();


   /////////////////////////////////////////////////////
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "------------------------------------------------------------\n"
                << " Size of the domain                      = (" << lx << "," << ly << "," << lz << ")\n"
                << " Total number of particles               = " << nx*ny*nz << "\n"
                << " Radius of a single particle             = " << radius << "\n"
                << " Initial spacing inbetween two particles = " << spacing << "\n"
                << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << std::endl;
      std::cout << "------------------------------------------------------------------------------" << std::endl;
   }


   /////////////////////////////////////////////////////
   // Setup timing results

   std::cout << "\n--" << pe_BROWN << "SETUP RESULTS" << pe_OLDCOLOR << "---------------------------------------------------------------\n"
             << " Total setup WC-Time = " << setupTime.total() << "\n"
             << "------------------------------------------------------------------------------" << std::endl;


   /////////////////////////////////////////////////////
   // Simulation loop

   WcTimer simTime;
   simTime.start();
   world->run( timesteps, stepsize );
   simTime.end();


   /////////////////////////////////////////////////////
   // Simulation timing results

   std::cout << "\n--" << pe_BROWN << "SIMULATION RESULTS" << pe_OLDCOLOR << "----------------------------------------------------------\n"
             << " Total simulation WC-Time = " << simTime.total() << "\n"
             << "------------------------------------------------------------------------------\n" << std::endl;
}
//*************************************************************************************************
