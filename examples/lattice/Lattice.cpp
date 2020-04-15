//=================================================================================================
/*!
 *  \file Lattice.cpp
 *  \brief Example file for the pe physics engine
 *
 *  Copyright (C) 2014 Tobias Preclik
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

#include <boost/filesystem.hpp>
#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;
using boost::filesystem::path;




// Assert statically that only a hard contact solver is used.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config TargetConfig1;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig1);




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the lattice example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * Particles in a non-periodic box with zero initial velocities.
 */
int main( int argc, char** argv )
{
   WcTimer setupTime, simTime;
   setupTime.start();

   // Conversion factors
   const real   second                   ( 1.0e-3 );                  // Conversion factor from unitless time to seconds.
   const real   kilogram                 ( 1.0e-3 );                  // Conversion factor from unitless weight to kilograms.
   const real   meter                    ( 1.0e-3 );                  // Conversion factor from unitless length to meters.
   const real   g_cm3                    ( 1.0e+3 );                  // g/cm^3 = 1000 kg/m^3 = 1000*kilogram/meter^3
   const real   m_s2                     ( 1.0e+3 );                  // meter/second^2
   const real   m_s                      ( 1.0e+0 );                  // meter/second

   UNUSED( m_s );
   UNUSED( kilogram );


   //----------------------------------------------------------------------------------------------
   // Hard-coded simulation parameters

         size_t seed                     ( 12345 );                   // Seed for the random number generation
   const real   gravity                  ( 9.81 / m_s2 );             // Acceleration along negative z direction in mm/ms^2=10^3*m/s^2. The gravity of earth is 9.81m/s^2.

   const real   duration                 ( 0.1 / second );            // The duration of the shaking phase
   const real   dt_max                   ( 1.0e-5 / second );         // The maximum size of the time steps limiting time discretization errors.

         bool   povray                   ( true );                   // Switches the POV-Ray visualization on and off.
   const size_t povray_fps               ( 1000 );                     // Target frames per second for the POV-Ray visualization.
   //const size_t povray_fps               ( 30 );                      // Target frames per second for the POV-Ray visualization.
   const path   povray_path              ( "video/" );                // The path where to store the visualization data.

   // Properties of the granular particles:
   const real   r                        ( 1.00e-3 / meter );         // The radius of particles.
   const real   granular_density         ( 2.65 / g_cm3 );            // Density of the granular particles is 2.65 g/cm^3, that is similar to quartz.

   const real   cof                      ( 0.85 / 2 );                // Coefficient of friction. Note: pe doubles the input coefficient of friction for material-material contacts.
   const real   angle                    ( 0.0 / 180.0 * M_PI );     // Angle of the ramp.


   //----------------------------------------------------------------------------------------------
   // Evaluation of command line arguments

   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ("particles", value< std::vector<int> >()->multitoken()->required(), "number of particles in x-, y- and z-dimension")
   ;
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();

   // Override hard-coded povray settings by command line options
   if( vm.count( "no-povray" ) > 0 )
      povray = false;

   // Supply initial seed for the random number generation (prefer seed given on command line)
   if( vm.count( "seed" ) > 0 )
      seed = vm[ "seed" ].as<uint32_t>();
   else
      setSeed( seed );

   const int nx( vm[ "particles" ].as< std::vector<int> >()[0] );
   const int ny( vm[ "particles" ].as< std::vector<int> >()[1] );
   const int nz( vm[ "particles" ].as< std::vector<int> >()[2] );

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


   //----------------------------------------------------------------------------------------------
   // Derived quantities

   // Calculate domain \Omega = [0; lx] x [0; ly] x (0; lz):
   const real   lx                       ( nx * 2 * r );                                       // Length of the simulation domain in x-dimension.
   const real   ly                       ( (ny - 1) * std::sqrt( real( 3 ) ) * r + 2*r );                  // Length of the simulation domain in y-dimension.
   const real   lz                       ( (nz - 1) * std::sqrt( real( 8 ) / real( 3 ) ) * r + 2*r  );                                       // Length of the simulation domain in z-dimension.

   const size_t povray_spacing           ( (std::size_t) std::ceil( 1 / ( second * dt_max * povray_fps ) ) );  // The number of simulation steps between two frames.
   const size_t timesteps                ( (std::size_t)( std::ceil( ( duration / dt_max ) / povray_spacing ) ) * povray_spacing );

   //----------------------------------------------------------------------------------------------
   // Aliases

   WorldID      world                    ( theWorld() );


   //----------------------------------------------------------------------------------------------
   // Parameter assertions

   pe_INTERNAL_ASSERT( ( real( 1 ) / ( dt_max * povray_spacing ) / second - povray_fps ) / povray_fps < 0.01, "Effective visualization frame rate deviates from prescribed frame rate by more than 1%." );


   //----------------------------------------------------------------------------------------------
   // Setup of the POV-Ray visualization

   WriterID pov;

   if( povray ) {
      pov = activateWriter();
      pov->setSpacing( povray_spacing );
      pov->setFilename( ( povray_path / "pic%.pov" ).string().c_str() );
      pov->include( "settings.inc" );
      pov->setDecorations( false );
   }


   //----------------------------------------------------------------------------------------------
   // Setup of the collision system

   CollisionSystemID cs( theCollisionSystem() );
   cs->setRelaxationParameter( 1.0 );
   cs->setMaxIterations( 100 );


   //----------------------------------------------------------------------------------------------
   // Setup of the simulation domain

   world->setGravity( std::sin( angle ) * gravity, 0, -std::cos( angle ) * gravity );  // Take over gravity.
   world->setDamping( 1 );               // Deactivate damping.

   MaterialID granular_material = createMaterial( "granular", granular_density, 0, cof, cof, real( 0.5 ), 1, 1, 0, 0 );
   MaterialID boundary_material = createMaterial( "boundary", 1,                0, cof, cof, real( 0.5 ), 1, 1, 0, 0 );

   // Setup of confining walls
   createPlane( 0,  Vec3( 0,  0, +1), 0,   boundary_material, true );
   createPlane( 1,  Vec3( 0,  0, -1), -lz, boundary_material, true );
   createPlane( 2,  Vec3( 0, +1,  0), 0,   boundary_material, true );
   createPlane( 3,  Vec3( 0, -1,  0), -ly, boundary_material, true );
   createPlane( 4,  Vec3(+1,  0,  0), 0,   boundary_material, true );
   createPlane( 5,  Vec3(-1,  0,  0), -lx, boundary_material, true );

   // Deterministic setup of the particles (iterate over all points in the grid which are strictly inside our subdomain or on _any_ boundary)
   for( int i_z = 0; i_z < nz; ++i_z ) {
      for( int i_y = 0; i_y < ny - i_z % 2; ++i_y ) {
         for( int i_x = 0; i_x < nx - (i_z % 2 == 0 ? i_y % 2 : 1 - i_y % 2 ); ++i_x ) {
            int offset;
            if( i_z % 2 == 1 && i_y % 2 == 1 )
               offset = 2;
            else
               offset = 0;

            Vec3 position(
                  ( 2 * i_x + i_y % 2 + i_z % 2 - offset + real( 1 ) ) * r,
                  ( std::sqrt( real( 3 ) ) * ( i_y + ( real( 1 ) / real( 3 ) ) * ( i_z % 2 ) ) + real( 1 ) ) * r,
                  ( std::sqrt( real( 8 ) / real( 3 ) ) * i_z  + real( 1 ) ) * r );

            std::cout << "(" << i_x << ", " << i_y << ", " << i_z << ") = (" << position[0]/r << ", " << position[1]/r << ", " << position[2]/r << ")" << std::endl;

            createSphere( 6, position, r, granular_material );
         }
      }
   }

   if( povray )
      pov->writeFile( ( povray_path / "init.pov" ).string().c_str() );


   //----------------------------------------------------------------------------------------------
   // Parameter estimations and assertions

   pe_INTERNAL_ASSERT( real(1) / ( povray_fps * second ) > dt_max, "Visualization rate too high." );


   // Ending the time measurement for the setup
   setupTime.end();


   //----------------------------------------------------------------------------------------------
   // Output of the simulation settings

   std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
             << "------------------------------------------------------------\n"
             << " Size of the domain                  = [0mm; " << lx << "mm] x [0mm; " << ly << "mm] x [0mm; " << lz << "mm]\n"
             << " Visualization framerate             = "  << povray_fps << "fps\n"
             << " Total number of particles           = "  << nx*ny*nz << "\n"
             << " Range of radii of particles         = "  << r << "mm\n"
             << " Seed of the random number generator = "  << getSeed() << "\n"
             << " Duration                            = "  << duration << "ms (" << timesteps << " time steps)\n"
             << " Maximum time step                   = "  << dt_max << "ms\n"
             << "------------------------------------------------------------------------------" << std::endl;


   //----------------------------------------------------------------------------------------------
   // Setup timing results

   setupTime.end();

   std::cout << "\n--" << pe_BROWN << "SETUP RESULTS" << pe_OLDCOLOR << "---------------------------------------------------------------\n"
             << " Total WC-Time = " << setupTime.total() << "\n"
             << "------------------------------------------------------------------------------" << std::endl;


   //----------------------------------------------------------------------------------------------
   // Simulation loop

   simTime.start();
   //world->run( timesteps, dt_max );
   simTime.end();


   //----------------------------------------------------------------------------------------------
   // Simulation timing results

   std::cout << "\n--" << pe_BROWN << "SIMULATION RESULTS" << pe_OLDCOLOR << "----------------------------------------------------------\n"
             << " Total WC-Time = " << simTime.total() << "\n"
             << "------------------------------------------------------------------------------\n" << std::endl;
}
//*************************************************************************************************
