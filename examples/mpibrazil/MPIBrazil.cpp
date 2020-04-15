//=================================================================================================
/*!
 *  \file MPIBrazil.cpp
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

#include <pe/core/MPISystem.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <cstddef>
#include <iostream>

using namespace pe;
using namespace pe::timing;
using namespace pe::povray;
using boost::filesystem::path;




// Assert statically that only DEM solvers or a hard contact solver is used since parameters are tuned for it.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config TargetConfig1;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig1);




//*************************************************************************************************
class Checkpointer {
public:

   Checkpointer( path checkpointsPath = path( "checkpoints/" ) ) : checkpointsPath_( checkpointsPath ) {
   }

   void setPath( path checkpointsPath = path( "checkpoints/" ) ) {
      checkpointsPath_ = checkpointsPath;
   }

   void write( std::string name, bool povray ) {
      boost::filesystem::create_directories( checkpointsPath_ );
      bbwriter_.writeFileAsync( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );
      pe_PROFILING_SECTION {
         timing::WcTimer timeWait;
         timeWait.start();
         bbwriter_.wait();
         timeWait.end();
         MPI_Barrier( MPI_COMM_WORLD );

         pe_LOG_DEBUG_SECTION( log ) {
            log << "BodyBinaryWriter::wait() took " << timeWait.total() << "s on rank " << MPISettings::rank() << ".\n";
         }
      }

      std::ofstream fout( ( checkpointsPath_ / ( name + ".txt" ) ).string().c_str() );
      fout << std::setprecision( 17 );
      fout << "world_timesteps " << TimeStep::step() << "\n";
      if( povray ) {
         WriterID pov( activateWriter() );
         fout << "povray_steps "   << pov->getSteps()       << "\n"
              << "povray_counter " << pov->getFileCounter() << "\n";
      }
      fout << std::flush;
   }

   void read( std::string name, bool povray ) {
      bbreader_.readFile( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );

      std::ifstream fin( ( checkpointsPath_ / ( name + ".txt" ) ).string().c_str() );
      std::string key, value;
      std::map<std::string, std::string> paramMap;
      while( fin ) {
         fin >> key >> value;
         if( !fin )
            break;
         paramMap[key] = value;
      }

      TimeStep::step( boost::lexical_cast<unsigned int>( paramMap["world_timesteps"] ) );
      if( povray ) {
         WriterID pov( activateWriter() );
         pov->setSteps      ( boost::lexical_cast<size_t>( paramMap["povray_steps"]   ) );
         pov->setFileCounter( boost::lexical_cast<size_t>( paramMap["povray_counter"] ) );
      }
   }

   void flush() {
      bbwriter_.wait();
   }

private:

   BodyBinaryWriter bbwriter_;
   BodyBinaryReader bbreader_;
   path             checkpointsPath_;
};
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY TEXTURE POLICY
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Process Texture policy.
 *
 * This class assigns textures to the particles according to the process they are owned.
 */
class ProcessTexturePolicy : public TexturePolicy
{
public:
   explicit ProcessTexturePolicy( const std::string& texproc ) : texproc_(texproc) {}
   virtual ~ProcessTexturePolicy() {}

   virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
   {
      switch( body->getID() )
      {
         case 0:   return CustomTexture( "TBoundaryXMin"  );
         case 1:   return CustomTexture( "TBoundaryXMax"  );
         case 2:   return CustomTexture( "TBoundaryYMin"  );
         case 3:   return CustomTexture( "TBoundaryYMax"  );
         case 4:   return CustomTexture( "TBoundaryZMin"  );
         case 5:   return CustomTexture( "TBoundaryZMax"  );
         case 6:   return CustomTexture( texproc_         );
         case 7:   return CustomTexture( texproc_         );
         case 8:   return CustomTexture( texproc_         );
         case 9:   return CustomTexture( texproc_         );
         case 10:  return CustomTexture( texproc_         );
         case 11:  return CustomTexture( texproc_         );
         case 12:  return CustomTexture( texproc_         );
         case 13:  return CustomTexture( texproc_         );
         case 14:  return CustomTexture( texproc_         );
         case 15:  return CustomTexture( texproc_         );
         default:  return CustomTexture( "TUnknown" );
      }
   }

   using TexturePolicy::getTexture;

private:
   std::string texproc_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Custom Texture policy.
 *
 * This class assigns textures to the particles according to their size they have.
 */
class GrainsizeTexturePolicy : public TexturePolicy
{
public:
   explicit GrainsizeTexturePolicy() {}
   virtual ~GrainsizeTexturePolicy() {}

   virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
   {
      switch( body->getID() )
      {
         case 0:   return CustomTexture( "TBoundaryXMin"  );
         case 1:   return CustomTexture( "TBoundaryXMax"  );
         case 2:   return CustomTexture( "TBoundaryYMin"  );
         case 3:   return CustomTexture( "TBoundaryYMax"  );
         case 4:   return CustomTexture( "TBoundaryZMin"  );
         case 5:   return CustomTexture( "TBoundaryZMax"  );
         case 6:   return CustomTexture( "TGranular0"     );
         case 7:   return CustomTexture( "TGranular1"     );
         case 8:   return CustomTexture( "TGranular2"     );
         case 9:   return CustomTexture( "TGranular3"     );
         case 10:  return CustomTexture( "TGranular4"     );
         case 11:  return CustomTexture( "TGranular5"     );
         case 12:  return CustomTexture( "TGranular6"     );
         case 13:  return CustomTexture( "TGranular7"     );
         case 14:  return CustomTexture( "TGranular8"     );
         case 15:  return CustomTexture( "TGranular9"     );
         default:  return CustomTexture( "TUnknown" );
      }
   }

   using TexturePolicy::getTexture;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Prints height-size histogram data on standard output in Matlab format.
 */
void printHistogram() {
   WorldID world( theWorld() );

   // Determine maximum height
   real h_max( 0 );
   for( World::Iterator it = world->begin(); it != world->end(); ++it )
      if( (*it)->getID() >= 6 && (*it)->getID() < 16 && !(*it)->isRemote() )
         h_max = std::max( h_max, (*it)->getAABB()[5] );

   MPI_Allreduce( MPI_IN_PLACE, &h_max, 1, MPITrait<real>::getType(), MPI_MAX, MPISettings::comm() );

   real dh( h_max / 10 );

   size_t histogram[10][10];
   for( int i = 0; i < 10; ++i )
      for( int j = 0; j < 10; ++j )
         histogram[i][j] = 0;

   // Generate local histogram
   for( World::Iterator it = world->begin(); it != world->end(); ++it )
      if( (*it)->getID() >= 6 && (*it)->getID() < 16 && !(*it)->isRemote() )
         ++histogram[(*it)->getID()-6][std::min((size_t)( std::floor( (*it)->getPosition()[2] / dh ) ), (size_t)9)];

   MPI_Allreduce( MPI_IN_PLACE, histogram, 100, MPITrait<size_t>::getType(), MPI_SUM, MPISettings::comm() );

   // Print histogram as Matlab matrix
   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "h_settled = " << h_max << ";\n";
      std::cout << "A_settled = [";
      for( int i = 0; i < 10; ++i ) {
         for( int j = 0; j < 10; ++j ) {
            std::cout << " " << histogram[i][j];
         }
         std::cout << " ;\n";
      }
      std::cout << " ];" << std::endl;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mpibrazil example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The mpibrazil simulation is a simulation of a container filled with particles that is shaken.
 * The shaking is done by fictitious forces. The simulation is split into three phases. In the
 * first phase particles freely fall downward. The particles have an initial downward velocity
 * and the falling velocity is limited. Afterwards the shaking phase begins. Finally in the
 * third phase the particles settle again.
 */
int main( int argc, char** argv )
{
   WcTimer setupTime, simTime, checkpointerTime;
   setupTime.start();

   // MPI Initialization
   MPI_Init( &argc, &argv );

   // Conversion factors
   const real   second                   ( 1.0e-3 );                  // Conversion factor from unitless time to seconds.
   const real   kilogram                 ( 1.0e-3 );                  // Conversion factor from unitless weight to kilograms.
   const real   meter                    ( 1.0e-3 );                  // Conversion factor from unitless length to meters.
   const real   g_cm3                    ( 1.0e+3 );                  // g/cm^3 = 1000 kg/m^3 = 1000*kilogram/meter^3
   const real   m_s2                     ( 1.0e+3 );                  // meter/second^2

   UNUSED( kilogram );


   //----------------------------------------------------------------------------------------------
   // Hard-coded simulation parameters

         size_t seed                     ( 12345 );                   // Seed for the random number generation
   const real   gravity                  ( 9.81 / m_s2 );             // Acceleration along negative z direction in mm/ms^2=10^3*m/s^2. The gravity of earth is 9.81m/s^2.

   const real   duration_shaking         ( 2 / second );              // The duration of the shaking phase
   const real   dt_max                   ( 1.0e-6 / second );         // The maximum size of the time steps limiting time discretization errors.

   const int    px                       ( 3 );                       // Number of processes in x direction
   const int    py                       ( 3 );                       // Number of processes in y direction
   const int    pz                       ( 1 );                       // Number of processes in z direction

         bool   povray                   ( true );                    // Switches the POV-Ray visualization on and off.
   const size_t povray_fps               ( 1000 );                      // Target frames per second for the POV-Ray visualization.
   //const size_t povray_fps               ( 30 );                      // Target frames per second for the POV-Ray visualization.
   const path   povray_path              ( "video/" );                // The path where to store the visualization data.

   // Properties of the granular particles:
   const size_t granular_nx              ( 3*px );                    // Number of granular particles in x direction
   const size_t granular_ny              ( 3*py );                    // Number of granular particles in y direction
   const size_t granular_nz              ( 75*pz );                   // Number of granular particles in z direction
   const real   granular_r_min           ( 0.25e-3 / meter );         // The radius of particles in mm. Sand grains range from 0.063mm to 2mm in size.
   const real   granular_r_max           ( 2.00e-3 / meter );         // The radius of particles in mm. Sand grains range from 0.063mm to 2mm in size.
   const real   granular_spacing         ( 0.1 * 0.5 * ( granular_r_min + granular_r_max ) );    // Initial spacing in-between two spherical particles in mm.
   const real   granular_dist            ( 2.0 * granular_r_max + granular_spacing );
   const real   granular_density         ( 2.65 / g_cm3 );            // Density of the granular particles is 2.65 g/cm^3, that is similar to quartz.

   const real   static_cof               ( 0.85 / 2 );                // Coefficient of static friction. Roughly 0.85 with high variation depending on surface roughness for low stresses. Note: pe doubles the input coefficient of friction for material-material contacts.
   const real   dynamic_cof              ( static_cof );              // Coefficient of dynamic friction. Similar to static friction for low speed friction.

   // Calculate domain \Omega = [0; lx] x [0; ly] x (0; lz):
   const real   lx                       ( granular_nx * granular_dist );                                       // Length of the simulation domain in x-dimension.
   const real   ly                       ( granular_ny * granular_dist );                                       // Length of the simulation domain in y-dimension.
   const real   lz                       ( granular_nz * granular_dist );                                       // Length of the simulation domain in z-dimension.

   const real   shaking_period           ( 0.25 / second );
   const real   shaking_amplitude        ( 15 * granular_r_max );


   //----------------------------------------------------------------------------------------------
   // Evaluation of command line arguments

   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ( "resume", value<std::string>()->default_value( "" ), "the checkpoint to resume" );
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

   std::string resume( vm[ "resume" ].as<std::string>() );


   //----------------------------------------------------------------------------------------------
   // Derived quantities

   const real   l_dd                     ( std::min( lx/px, std::min( ly/py, lz/pz ) ) );  // Minimum distance to non-nearest neighbor subdomain.
   const real   v_max                    ( (real(1) / real(250)) * granular_r_min / dt_max );
   const real   granular_freefall        ( 1.296 * ( granular_nz * granular_dist ) / v_max );     // Time for the top-most particles to (freely) fall to the ground without gravitational acceleration.
   //const real   granular_freefall        ( 1.3 * std::sqrt( 2 * ( granular_nz * granular_dist ) / gravity ) );  // Time for the top-most particles to (freely) fall to the ground subject to gravitational acceleration.
   const size_t povray_spacing           ( (std::size_t) std::ceil( 1 / ( second * dt_max * povray_fps ) ) );  // The number of simulation steps between two frames.
   const size_t timesteps_settling       ( (std::size_t)( std::ceil( ( granular_freefall / dt_max ) / povray_spacing ) ) * povray_spacing );
   const size_t timesteps_shaking        ( (std::size_t)( std::ceil( ( duration_shaking / dt_max ) / povray_spacing ) ) * povray_spacing );
   const size_t timesteps_total          ( 2*timesteps_settling + timesteps_shaking );

   //----------------------------------------------------------------------------------------------
   // Aliases

   WorldID      world                    ( theWorld() );


   //----------------------------------------------------------------------------------------------
   // Global variables

   bool         timeout                  ( false );
   bool         statusupdate             ( false );
   double       last_statusupdate        ( 0 );
   size_t       t0                       ( 0 );


   //----------------------------------------------------------------------------------------------
   // Parameter assertions

   UNUSED( l_dd );
   pe_INTERNAL_ASSERT( granular_r_max < l_dd, "Granular matter too large for nearest neighbor communication." );
   pe_INTERNAL_ASSERT( ( real( 1 ) / ( dt_max * povray_spacing ) / second - povray_fps ) / povray_fps < 0.01, "Effective visualization frame rate deviates from prescribed frame rate by more than 1%." );
   pe_INTERNAL_ASSERT( granular_freefall > 0, "Freefall estimation must not be zero." );
   pe_INTERNAL_ASSERT( std::abs( std::floor( duration_shaking / shaking_period ) * shaking_period - duration_shaking ) < Limits<real>::fpuAccuracy(), "Shaking duration is not a multiple of the period." );


   //----------------------------------------------------------------------------------------------
   // Setup of Domain Decomposition

   RectilinearGrid grid;
   grid.connect( Vec3(0, 0, 0), Vec3(lx, ly, lz), Vector3<size_t>(px, py, pz), Vector3<BoundaryCondition>( boundaryConditionOpen, boundaryConditionOpen, boundaryConditionOpen ), Vector3<BoundaryCondition>( boundaryConditionOpen, boundaryConditionOpen, boundaryConditionOpen ) );
   //grid.connect( Vec3(0, 0, 0), Vec3(lx, ly, lz), Vector3<size_t>(px, py, pz), Vector3<BoundaryCondition>( boundaryConditionOutflow, boundaryConditionOutflow, boundaryConditionOutflow ), Vector3<BoundaryCondition>( boundaryConditionOutflow, boundaryConditionOutflow, boundaryConditionOutflow ) );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "The domain decomposition grid comprises the domain [0; " << lx << "] x [0; " << ly << "] x (0; " << lz << "), where open boundary conditions are applied in x-, y- and z-directions.\n";
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

      //std::ostringstream texture;
      //texture << "TProcess" << (grid.getCoords()[0] + grid.getCoords()[1] + grid.getCoords()[2]) % 2;
      //pov->setTexturePolicy( ProcessTexturePolicy( texture.str() ) );
      pov->setTexturePolicy( GrainsizeTexturePolicy() );
   }


   //----------------------------------------------------------------------------------------------
   // Setup of the collision system

   CollisionSystemID cs( theCollisionSystem() );
   cs->setRelaxationParameter( 0.75 );
   cs->setMaxIterations( 10 );


   //----------------------------------------------------------------------------------------------
   // Setup of the simulation domain

   world->setGravity( 0, 0, -gravity );  // Take over gravity.
   world->setDamping( 1 );               // Deactivate damping.

   MaterialID granular_material = createMaterial( "granular", granular_density, 0, static_cof, dynamic_cof, real( 0.5 ), 1, 1, 0, 0 );
   MaterialID boundary_material = createMaterial( "boundary", 1,                0, static_cof, dynamic_cof, real( 0.5 ), 1, 1, 0, 0 );

   Checkpointer         checkpointer;
   path                 checkpoint_path( "checkpoints/" );            // The path where to store the checkpointing data

   if( resume.empty() ) {
      // Setup of confining walls
      pe_GLOBAL_SECTION
      {
         createPlane( 0,  Vec3(+1, 0, 0), 0,   boundary_material, false );
         createPlane( 1,  Vec3(-1, 0, 0), -lx, boundary_material, false );
         createPlane( 2,  Vec3(0, +1, 0), 0,   boundary_material, false );
         createPlane( 3,  Vec3(0, -1, 0), -ly, boundary_material, false );
         createPlane( 4,  Vec3(0, 0, +1), 0,   boundary_material, false );
         createPlane( 5,  Vec3(0, 0, -1), -lz, boundary_material, false );
      }

      // Deterministic setup of the particles (iterate over all points in the grid which are strictly inside our subdomain or on _any_ boundary)
      size_t x_min = std::max( (int)ceil ( (grid.getCoords()[0]    ) * (lx / px) / granular_dist - real(1.0) ), 0                );
      size_t y_min = std::max( (int)ceil ( (grid.getCoords()[1]    ) * (ly / py) / granular_dist - real(1.0) ), 0                );
      size_t z_min = std::max( (int)ceil ( (grid.getCoords()[2]    ) * (lz / pz) / granular_dist - real(1.0) ), 0                );
      size_t x_max = std::min( (int)floor( (grid.getCoords()[0] + 1) * (lx / px) / granular_dist + real(0.5) ), (int)granular_nx );
      size_t y_max = std::min( (int)floor( (grid.getCoords()[1] + 1) * (ly / py) / granular_dist + real(0.5) ), (int)granular_ny );
      size_t z_max = std::min( (int)floor( (grid.getCoords()[2] + 1) * (lz / pz) / granular_dist + real(0.5) ), (int)granular_nz );

      for( size_t i_x = x_min; i_x < x_max; ++i_x ) {
         for( size_t i_y = y_min; i_y < y_max; ++i_y ) {
            for( size_t i_z = z_min; i_z < z_max; ++i_z ) {
               Vec3 position(
                     ( i_x + real(0.5) ) * granular_dist,
                     ( i_y + real(0.5) ) * granular_dist,
                     ( i_z + real(0.5) ) * granular_dist );

               // Explicitly test points whether they are located on the boundary
               if( !theWorld()->ownsPoint( position ) )
                  continue;

               real radius         = pe::rand( granular_r_min, granular_r_max );
               real vel_angle      = pe::rand( real(0), 2*M_PI );
               real vel            = pe::rand( real(0), granular_spacing / granular_freefall );
               int  radius_key     = std::max( 0, std::min( 9, static_cast<int>( ( radius - granular_r_min ) / ( granular_r_max - granular_r_min ) * 10.0 ) ) );
               real orient_angle_z = pe::rand( real(0), 2*M_PI );
               real orient_angle_y = pe::rand( -0.5*M_PI, 0.5*M_PI );
               real orient_angle_x = pe::rand( real(0), 2*M_PI );

               BodyID particle( createRock( 6+radius_key, position, radius, granular_material, true ) );
               particle->setLinearVel( vel * cos( vel_angle ), vel * sin( vel_angle ), -v_max );

               // Perform z, y', x'' Euler rotation
               particle->rotate( Vec3(0,                                       0,                                       1                  ), orient_angle_z );
               particle->rotate( Vec3(-sin(orient_angle_z),                    cos(orient_angle_z),                     0                  ), orient_angle_y );
               particle->rotate( Vec3(cos(orient_angle_y)*cos(orient_angle_z), cos(orient_angle_y)*sin(orient_angle_z), sin(orient_angle_y)), orient_angle_x );
            }
         }
      }

      if( povray )
         pov->writeFile( ( povray_path / "init.pov" ).string().c_str() );
   }
   else {
      // Resume from checkpoint
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Resuming checkpoint \"" << resume << "\"..." << std::endl;
      }

      checkpointer.setPath( checkpoint_path / resume );
      checkpointer.read( resume, povray );
      // PovRay textures are reassigned through texture policy

      if( povray )
         pov->writeFile( ( povray_path / "resume.pov" ).string().c_str() );

      t0 = TimeStep::step();
   }

   // Synchronization of the MPI processes
   world->synchronize();

   //----------------------------------------------------------------------------------------------
   // Parameter estimations and assertions

   //pe_LOG_INFO_SECTION( log ) {
   //   log << "Estimated height of random close packing of particles: " << lz << "\n";
   //}
   //pe_INTERNAL_ASSERT( lz < granular_nz * granular_dist, "Initial packing too dense to estimate duration of settling phase." );

   pe_INTERNAL_ASSERT( real(1) / ( povray_fps * second ) > dt_max, "Visualization rate too high." );


   //----------------------------------------------------------------------------------------------
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "------------------------------------------------------------\n"
                << " Size of the domain                  = [0mm; " << lx << "mm] x [0mm; " << ly << "mm] x [0mm; " << lz << "mm]\n"
                << " Number of MPI processes             = (" << px << ", " << py << ", " << pz << ")\n"
                << " Visualization framerate             = "  << povray_fps << "fps\n"
                << " Total number of particles           = "  << granular_nx*granular_ny*granular_nz << "\n"
                << " Particles per process               = "  << granular_nx/px << "x" << granular_ny/py << "x" << granular_nz/pz << " = " << granular_nx * granular_ny * granular_nz/(px*py*pz) << "\n"
                << " Range of radii of particles         = [" << granular_r_min << "mm; " << granular_r_max << "mm)\n"
                << " Initial spacing between particles   = "  << granular_spacing << "mm\n"
                << " Seed of the random number generator = "  << getSeed() << "\n"
                << " Duration of settling phase(s)       = "  << granular_freefall << "ms (" << timesteps_settling << " time steps)\n"
                << " Duration of shaking phase           = "  << duration_shaking << "ms (" << timesteps_shaking << " time steps)\n"
                << " Maximum time step                   = "  << dt_max << "ms\n"
                << "------------------------------------------------------------------------------" << std::endl;
   }

   //----------------------------------------------------------------------------------------------
   // Simulation loop

   std::vector<real> buffer( 4 );

   setupTime.end();
   MPI_Barrier( MPI_COMM_WORLD );
   simTime.start();

   while( TimeStep::step() < timesteps_total ) {
      if( TimeStep::step() < timesteps_settling ) {
         // Initial settling phase.

         // Limit maximum velocity in initial settling phase.
         for( World::Iterator it = world->begin(); it != world->end(); ++it ) {
            if( (*it)->getLinearVel()[2] < -v_max ) {
               Vec3 v( (*it)->getLinearVel() );
               v[2] = -v_max;
               (*it)->setLinearVel( v );
            }
         }
      }
      else if( TimeStep::step() < timesteps_settling + timesteps_shaking ) {
         // Shaking phase.

         // Checkpoint and print histogram when entering shaking phase.
         if( TimeStep::step() == timesteps_settling && TimeStep::step() != t0 ) {
            checkpointerTime.start();

            checkpointer.setPath( checkpoint_path / "settled" );
            checkpointer.write( "settled", povray );

            if( povray )
               pov->writeFile( ( povray_path / "settled.pov" ).string().c_str() );

            pe_EXCLUSIVE_SECTION( 0 ) {
               std::cout << "Ended initial settling phase at time " << TimeStep::step()*dt_max << " (" << TimeStep::step() << ").\n";
            }

            checkpointerTime.end();

            printHistogram();

            //break;
         }

         // The pe coordinate system (frame) is attached to the container that is shaken and is
         // thus a non-inertial frame. The container is shaken with period T and amplitude x0
         // in the inertial frame. The x-coordinate of the origin of the pe frame in the inertial
         // frame is thus x(t)=x0*sin((t-t0)*2pi/T) resulting in an acceleration of
         // \ddot{x}(t)=-x0*sin((t-t0)*2pi/T)*(2pi/T)^2.
         world->setGravity( shaking_amplitude * sin( (TimeStep::step() - timesteps_settling)*dt_max*2*M_PI/shaking_period ) * sq( 2*M_PI/shaking_period ), 0, -gravity );
      }
      else {
         // Final settling phase.

         // Checkpoint when entering final settling phase.
         if( TimeStep::step() == timesteps_settling + timesteps_shaking && TimeStep::step() != t0 ) {
            checkpointerTime.start();

            checkpointer.setPath( checkpoint_path / "shaken" );
            checkpointer.write( "shaken", povray );

            if( povray )
               pov->writeFile( ( povray_path / "shaken.pov" ).string().c_str() );

            pe_EXCLUSIVE_SECTION( 0 ) {
               std::cout << "Ended shaking phase at time " << TimeStep::step()*dt_max << " (" << TimeStep::step() << ").\n";
            }

            checkpointerTime.end();

            //break;
         }

         world->setGravity( 0, 0, -gravity );
      }

      // Checkpoint every 100000 time steps
      if( TimeStep::step() % 100000 == 0 && TimeStep::step() != t0 ) {
         checkpointerTime.start();

         std::stringstream sstr;
         sstr << "snapshot" << TimeStep::step() / 1000;
         checkpointer.setPath( checkpoint_path / sstr.str() );
         checkpointer.write( sstr.str(), povray );

         checkpointerTime.end();
      }

      // All-to-all communication
      if( TimeStep::step() % 100 == 0 ) {
         buffer[0] = timeout      ? 0 : 1;
         buffer[1] = statusupdate ? 0 : 1;

         MPI_Allreduce( MPI_IN_PLACE, &buffer[0], buffer.size(), MPITrait<real>::getType(), MPI_MIN, MPISettings::comm() );

         timeout      = buffer[0] == 0;
         statusupdate = buffer[1] == 0;

         if( timeout )
            break;

         if( statusupdate ) {
            statusupdate = false;

            MemoryMeter mm;
            mm.stop();

            pe_LOG_DEBUG_SECTION( log ) {
               log << "------------------------------------------------------------------------------\n"
                   << " Total memory allocated      = " << TimeStep::step() << " " << mm.lastAllocation() << "bytes\n"
                   << " Total memory in use         = " << TimeStep::step() << " " << mm.lastInUse()     << "bytes\n"
                   << "------------------------------------------------------------------------------\n";
            }

            theCollisionSystem()->logProfilingSummary();
         }
      }

      world->simulationStep( dt_max );

      pe_EXCLUSIVE_SECTION( 0 ) {
         simTime.lap();

         // Update status if last status output was at least 60 seconds ago
         if( simTime.total() - last_statusupdate >= 60 ) {
            last_statusupdate = simTime.total();
            statusupdate = true;

            size_t wcl        ( (size_t)( simTime.total() ) );
            size_t wcl_hours  ( ( wcl                                   ) / 3600 );
            size_t wcl_minutes( ( wcl - wcl_hours*3600                  ) / 60   );
            size_t wcl_seconds( ( wcl - wcl_hours*3600 - wcl_minutes*60 )        );

            size_t eta        ( (size_t)( simTime.average() * ( timesteps_total - TimeStep::step() ) )  );
            size_t eta_hours  ( ( eta                                   ) / 3600 );
            size_t eta_minutes( ( eta - eta_hours*3600                  ) / 60   );
            size_t eta_seconds( ( eta - eta_hours*3600 - eta_minutes*60 )        );

            std::cout << "------------------------------------------------------------------------------\n"
                      << " Simulation time      = " << TimeStep::step()*dt_max << "ms\n"
                      << " Number of time steps = " << TimeStep::step() << "/" << timesteps_total << " (" << std::floor( (real)(TimeStep::step()) / (real)(timesteps_total) * 10000 ) / 100 << "%)\n"
                      << " Timestep size        = " << dt_max << "ms\n"
                      << " Wall Clock Time      = " << wcl_hours << ":" << std::setfill('0') << std::setw(2) << wcl_minutes << ":" << std::setfill('0') << std::setw(2) << wcl_seconds << " (" << simTime.average() << "s per timestep)\n"
                      << " ETA                  = " << eta_hours << ":" << std::setfill('0') << std::setw(2) << eta_minutes << ":" << std::setfill('0') << std::setw(2) << eta_seconds << "\n"
                      << "------------------------------------------------------------------------------\n" << std::endl;
         }

         // Checkpoint and quit simulation if 23:55 hours have passed
         if( simTime.total() >= 24*60*60 - 5*60 )
            timeout = true;
      }
   }
   simTime.end();

   // Checkpoint and print histogram after final settling phase.
   if( TimeStep::step() == timesteps_total && TimeStep::step() != t0 ) {
      checkpointerTime.start();

      checkpointer.setPath( checkpoint_path / "end" );
      checkpointer.write( "end", povray );

      if( povray )
         pov->writeFile( ( povray_path / "end.pov" ).string().c_str() );

      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Ended final settling phase at time " << TimeStep::step()*dt_max << " (" << TimeStep::step() << ").\n";
      }

      checkpointerTime.end();

      printHistogram();
   }
   // Checkpoint on timeout.
   else if( timeout && TimeStep::step() != t0 ) {
      checkpointerTime.start();

      checkpointer.setPath( checkpoint_path / "timeout" );
      checkpointer.write( "timeout", povray );

      if( povray )
         pov->writeFile( ( povray_path / "timeout.pov" ).string().c_str() );

      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Wrote checkpoint before running out of time at time " << TimeStep::step()*dt_max << " (" << TimeStep::step() << ")." << std::endl;
      }

      checkpointerTime.end();
   }


   //----------------------------------------------------------------------------------------------
   // Simulation timing results

   pe_EXCLUSIVE_SECTION( 0 ) {
      size_t wcl        ( (size_t)( simTime.total() ) );
      size_t wcl_hours  ( ( wcl                                   ) / 3600 );
      size_t wcl_minutes( ( wcl - wcl_hours*3600                  ) / 60   );
      size_t wcl_seconds( ( wcl - wcl_hours*3600 - wcl_minutes*60 )        );

      std::cout << "------------------------------------------------------------------------------\n"
                << " Simulation time      = " << TimeStep::step()*dt_max << "ms\n"
                << " Number of time steps = " << TimeStep::step() << "\n"
                << " Timestep size        = " << dt_max << "ms\n"
                << " Wall Clock Time      = " << wcl_hours << ":" << std::setfill('0') << std::setw(2) << wcl_minutes << ":" << std::setfill('0') << std::setw(2) << wcl_seconds << " (" << simTime.average() << "s per timestep)\n"
                << "------------------------------------------------------------------------------\n\n"
                << "Timing results reduced over all time steps on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              " << "min time  "                                    << "   " << "max time  "                                    << "   " << "avg time  "                                        << "   " << "total time"                                      << "   " << "executions"                                           << "\n"
                << "--------------------   " << "----------"                                    << "   " << "----------"                                    << "   " << "----------"                                        << "   " << "----------"                                      << "   " << "----------"                                           << "\n"
                << "setup                  " << std::setw(10) << setupTime.min()                << "   " << std::setw(10) << setupTime.max()                << "   " << std::setw(10) << setupTime.average()                << "   " << std::setw(10) << setupTime.total()                << "   " << std::setw(10) << setupTime.getCounter()                << "\n"
                << "simulation step        " << std::setw(10) << simTime.min()                  << "   " << std::setw(10) << simTime.max()                  << "   " << std::setw(10) << simTime.average()                  << "   " << std::setw(10) << simTime.total()                  << "   " << std::setw(10) << simTime.getCounter()                  << "\n"
                << "  checkpointer         " << std::setw(10) << checkpointerTime.min()         << "   " << std::setw(10) << checkpointerTime.max()         << "   " << std::setw(10) << checkpointerTime.average()         << "   " << std::setw(10) << checkpointerTime.total()         << "   " << std::setw(10) << checkpointerTime.getCounter()         << "\n"
                << "--------------------   " << "----------"                                    << "   " << "----------"                                    << "   " << "----------"                                        << "   " << "----------"                                      << "   " << "----------"                                           << "\n";
   }

   theCollisionSystem()->logProfilingSummary();


   //----------------------------------------------------------------------------------------------
   // Cleanup

   checkpointer.flush();
   MPI_Finalize();
}
//*************************************************************************************************
