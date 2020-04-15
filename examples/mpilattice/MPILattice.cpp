//=================================================================================================
/*!
 *  \file MPILattice.cpp
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




//*************************************************************************************************
template<class C>
class CollisionSystemConfigurator
{
   // Disallow any other collision system.
   CollisionSystemConfigurator();
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystemConfigurator< C<CD,FD,BG,response::FFDSolver> >
{
public:
   CollisionSystemConfigurator() {
      // Nothing to configure here.
   }
};
//*************************************************************************************************


//*************************************************************************************************
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystemConfigurator< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >
{
public:
   CollisionSystemConfigurator() {
      typedef C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> Config;
      boost::shared_ptr< CollisionSystem<Config> > cs( CollisionSystem<Config>::instance() );
      cs->setRelaxationParameter( 0.75 );
      cs->setMaxIterations( 10 );
   }
};
//*************************************************************************************************




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mpilattice example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * Particles in a non-periodic box with non-zero initial velocities.
 */
int main( int argc, char** argv )
{
   WcTimer setupTime, simTime;
   setupTime.start();

   // MPI Initialization
   MPI_Init( &argc, &argv );

   // Conversion factors
   const real   second                   ( 1.0e-3 );                  // Conversion factor from unitless time to seconds.
   const real   kilogram                 ( 1.0e-3 );                  // Conversion factor from unitless weight to kilograms.
   const real   meter                    ( 1.0e-3 );                  // Conversion factor from unitless length to meters.
   const real   g_cm3                    ( 1.0e+3 );                  // g/cm^3 = 1000 kg/m^3 = 1000*kilogram/meter^3
   const real   m_s2                     ( 1.0e+3 );                  // meter/second^2
   const real   m_s                      ( 1.0e+0 );                  // meter/second

   UNUSED( kilogram );


   //----------------------------------------------------------------------------------------------
   // Hard-coded simulation parameters

         size_t seed                     ( 12345 );                   // Seed for the random number generation
   const real   gravity                  ( 9.81 / m_s2 );             // Acceleration along negative z direction in mm/ms^2=10^3*m/s^2. The gravity of earth is 9.81m/s^2.

   const real   duration                 ( 0.1 / second );              // The duration of the shaking phase
   const real   dt_max                   ( 1.0e-5 / second );         // The maximum size of the time steps limiting time discretization errors.

         bool   povray                   ( false );                    // Switches the POV-Ray visualization on and off.
   const size_t povray_fps               ( 1000 );                     // Target frames per second for the POV-Ray visualization.
   //const size_t povray_fps               ( 30 );                      // Target frames per second for the POV-Ray visualization.
   const path   povray_path              ( "video/" );                // The path where to store the visualization data.

   // Properties of the granular particles:
   const real   r                        ( 1.00e-3 / meter );         // The radius of particles.
   const real   granular_density         ( 2.65 / g_cm3 );            // Density of the granular particles is 2.65 g/cm^3, that is similar to quartz.

   const real   v                        ( 10.00e-2 / m_s );

   const real   static_cof               ( 0.85 / 2 );                // Coefficient of static friction. Roughly 0.85 with high variation depending on surface roughness for low stresses. Note: pe doubles the input coefficient of friction for material-material contacts.
   const real   dynamic_cof              ( static_cof );              // Coefficient of dynamic friction. Similar to static friction for low speed friction.

   const real   angle                    ( 30.0 / 180.0 * M_PI );     // Angle of the ramp.


   //----------------------------------------------------------------------------------------------
   // Evaluation of command line arguments

   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ("particles", value< std::vector<int> >()->multitoken()->required(), "number of particles in x-, y- and z-dimension")
      ("processes", value< std::vector<int> >()->multitoken()->required(), "number of processes in x-, y- and z-dimension")
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
      setSeed( seed + MPISettings::rank() );

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
   if( ny % 2 != 0 ) {
      std::cerr << " The number of particles in y-dimension must be even!\n";
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
      pe::exit( EXIT_FAILURE );
   }


   //----------------------------------------------------------------------------------------------
   // Derived quantities

   // Calculate domain \Omega = [0; lx] x [0; ly] x (0; lz):
   const real   lx                       ( nx * 2 * r );                                       // Length of the simulation domain in x-dimension.
   const real   ly                       ( ny * std::sqrt( real( 3 ) ) * r );                  // Length of the simulation domain in y-dimension.
   const real   lz                       ( (nz - 1) * std::sqrt( real( 8 ) / real( 3 ) ) * r + 2*r  );                                       // Length of the simulation domain in z-dimension.
   const int    nxpp                     ( nx / px );
   const int    nypp                     ( ny / py );
   const int    nzpp                     ( nz / pz );

   const real   l_dd                     ( std::min( lx/px, std::min( ly/py, lz/pz ) ) );  // Minimum distance to non-nearest neighbor subdomain.
   //const real   v_max                    ( (real(1) / real(250)) * r / dt_max );
   const size_t povray_spacing           ( (std::size_t) std::ceil( 1 / ( second * dt_max * povray_fps ) ) );  // The number of simulation steps between two frames.
   const size_t timesteps                ( (std::size_t)( std::ceil( ( duration / dt_max ) / povray_spacing ) ) * povray_spacing );

   //----------------------------------------------------------------------------------------------
   // Aliases

   WorldID      world                    ( theWorld() );


   //----------------------------------------------------------------------------------------------
   // Parameter assertions

   UNUSED( l_dd );
   pe_INTERNAL_ASSERT( r < l_dd, "Granular matter too large for nearest neighbor communication." );
   pe_INTERNAL_ASSERT( ( real( 1 ) / ( dt_max * povray_spacing ) / second - povray_fps ) / povray_fps < 0.01, "Effective visualization frame rate deviates from prescribed frame rate by more than 1%." );


   //----------------------------------------------------------------------------------------------
   // Setup of Domain Decomposition

   RectilinearGrid grid;
   grid.connect( Vec3(0, 0, 0), Vec3(lx, ly, lz), Vector3<size_t>(px, py, pz), Vector3<BoundaryCondition>( boundaryConditionPeriodic, boundaryConditionPeriodic, boundaryConditionOpen ), Vector3<BoundaryCondition>( boundaryConditionPeriodic, boundaryConditionPeriodic, boundaryConditionOpen ) );
   Vector3<size_t> coords( grid.getCoords() );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "The domain decomposition grid comprises the domain [0; " << lx << "] x [0; " << ly << "] x (0; " << lz << "), where periodic boundary conditions are applied in x- and y-directions and open boundary conditions in z-direction.\n";
   }

   //----------------------------------------------------------------------------------------------
   // Setup of the POV-Ray visualization

   WriterID pov;

   if( povray ) {
      pov = activateWriter();
      pov->setSpacing( povray_spacing );
      pov->setFilename( ( povray_path / "pic%.pov" ).string().c_str() );
      pov->include( "settings.inc" );
      pov->setDecorations( false );

      std::ostringstream texture;
      texture << "TProcess" << (grid.getCoords()[0] + grid.getCoords()[1] + grid.getCoords()[2]) % 2;
      pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
   }


   //----------------------------------------------------------------------------------------------
   // Setup of the collision system

   CollisionSystemConfigurator<Config>();


   //----------------------------------------------------------------------------------------------
   // Setup of the simulation domain

   world->setGravity( std::sin( angle ) * gravity, 0, -std::cos( angle ) * gravity );  // Take over gravity.
   world->setDamping( 1 );               // Deactivate damping.

   MaterialID granular_material = createMaterial( "granular", granular_density, 0, static_cof, dynamic_cof, real( 0.5 ), 1, 1, 0, 0 );
   MaterialID boundary_material = createMaterial( "boundary", 1,                0, static_cof, dynamic_cof, real( 0.5 ), 1, 1, 0, 0 );

   // Setup of confining walls
   pe_GLOBAL_SECTION
   {
      createPlane( 0,  Vec3(0, 0, +1), 0,   boundary_material, false );
      createPlane( 1,  Vec3(0, 0, -1), -lz, boundary_material, false );
   }

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "The domain decomposition grid comprises the domain [0; " << lx << "] x [0; " << ly << "] x (0; " << lz << "), where periodic boundary conditions are applied in x- and y-directions and open boundary conditions in z-direction.\n";
   }

   // Deterministic setup of the particles (iterate over all points in the grid which are strictly inside our subdomain or on _any_ boundary)
   for( size_t i_x = coords[0] * nxpp; i_x < ((coords[0]+1) * nxpp); ++i_x ) {
      for( size_t i_y = coords[1] * nypp; i_y < ((coords[1]+1) * nypp); ++i_y ) {
         for( size_t i_z = coords[2] * nzpp; i_z < ((coords[2]+1) * nzpp); ++i_z ) {
            int offset;
            if( i_z % 2 == 1 && i_y % 2 == 1 )
               offset = 2;
            else
               offset = 0;

            Vec3 position(
                  ( 2 * i_x + i_y % 2 + i_z % 2 - offset + real( 0.5 ) ) * r,
                  ( std::sqrt( real( 3 ) ) * ( i_y + ( real( 1 ) / real( 3 ) ) * ( i_z % 2 ) ) + real( 0.5 ) ) * r,
                  ( std::sqrt( real( 8 ) / real( 3 ) ) * i_z  + real( 1 ) ) * r );

            BodyID body = createSphere( 2, position, r, granular_material );
            body->setLinearVel( v, 0, 0 );
         }
      }
   }

   if( povray )
      pov->writeFile( ( povray_path / "init.pov" ).string().c_str() );

   // Synchronization of the MPI processes
   world->synchronize();


   //----------------------------------------------------------------------------------------------
   // Parameter estimations and assertions

   pe_INTERNAL_ASSERT( real(1) / ( povray_fps * second ) > dt_max, "Visualization rate too high." );


   // Ending the time measurement for the setup
   setupTime.end();


   //----------------------------------------------------------------------------------------------
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "------------------------------------------------------------\n"
                << " Size of the domain                  = [0mm; " << lx << "mm] x [0mm; " << ly << "mm] x [0mm; " << lz << "mm]\n"
                << " Number of MPI processes             = (" << px << ", " << py << ", " << pz << ")\n"
                << " Visualization framerate             = "  << povray_fps << "fps\n"
                << " Total number of particles           = "  << nx*ny*nz << "\n"
                << " Particles per process               = "  << nxpp << "x" << nypp << "x" << nzpp << " = " << nx * ny * nz/(px*py*pz) << "\n"
                << " Range of radii of particles         = "  << r << "mm\n"
                << " Seed of the random number generator = "  << getSeed() << "\n"
                << " Duration                            = "  << duration << "ms (" << timesteps << " time steps)\n"
                << " Maximum time step                   = "  << dt_max << "ms\n"
                << "------------------------------------------------------------------------------" << std::endl;
   }


   //----------------------------------------------------------------------------------------------
   // Setup timing results

   setupTime.end();
   double localTime( setupTime.total() );
   double globalMin( 0.0 );
   double globalMax( 0.0 );
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, MPISettings::comm() );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, MPISettings::comm() );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SETUP RESULTS" << pe_OLDCOLOR << "---------------------------------------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "------------------------------------------------------------------------------" << std::endl;
   }


   //----------------------------------------------------------------------------------------------
   // Simulation loop

   MPI_Barrier( MPISettings::comm() );
   simTime.start();
   world->run( timesteps, dt_max );
   simTime.end();


   //----------------------------------------------------------------------------------------------
   // Simulation timing results

   localTime  = simTime.total();
   MPI_Reduce( &localTime, &globalMin, 1, MPI_DOUBLE, MPI_MIN, 0, MPISettings::comm() );
   MPI_Reduce( &localTime, &globalMax, 1, MPI_DOUBLE, MPI_MAX, 0, MPISettings::comm() );

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION RESULTS" << pe_OLDCOLOR << "----------------------------------------------------------\n"
                << " Minimum total WC-Time = " << globalMin << "\n"
                << " Maximum total WC-Time = " << globalMax << "\n"
                << "------------------------------------------------------------------------------\n" << std::endl;
   }


   //----------------------------------------------------------------------------------------------
   // Cleanup

   MPI_Finalize();
}
//*************************************************************************************************
