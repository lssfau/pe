//=================================================================================================
/*!
 *  \file MPIImpact.cpp
 *  \brief Example file for the pe physics engine
 *
 *  Copyright (C) 2011, 2012 Tobias Preclik
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




#define BODYREADER 0
#define ADAPTIVETIMESTEPPING 1




// Assert statically that only DEM solvers or a hard contact solver is used since parameters are tuned for it.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B, C, D) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value | pe::IsSame<A,C>::value | pe::IsSame<A,D>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::DEMSolverObsolete>::Config TargetConfig1;
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config         TargetConfig2;
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::DEMSolver>::Config         TargetConfig3;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig1, TargetConfig2, TargetConfig3);




//*************************************************************************************************
class Checkpointer {
public:

   struct Params {
      Params( real &t, real &povray_t_next, bool &impactor_present)
         : t_(t), povray_t_next_( povray_t_next ), impactor_present_( impactor_present ) {}

      real &t_, &povray_t_next_;
      bool &impactor_present_;
   };

   Checkpointer( path checkpointsPath = path( "checkpoints/" ) ) : checkpointsPath_( checkpointsPath ) {
#if BODYREADER
      bodyReader_.setPrecision( 17 );
#endif
   }

   void setPath( path checkpointsPath = path( "checkpoints/" ) ) {
      checkpointsPath_ = checkpointsPath;
   }

   void write( std::string name, const Params &params ) {
#if BODYREADER
      path p( checkpointsPath_ / boost::lexical_cast<std::string>( theMPISystem()->getRank() ) );
      boost::filesystem::create_directories( p / name );

      bodyReader_.writeFile( ( p / ( name + ".pe" ) ).string().c_str() );
#else
      boost::filesystem::create_directories( checkpointsPath_ );
      bbwriter_.writeFileAsync( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );
      pe_PROFILING_SECTION {
         timing::WcTimer timeWait;
         timeWait.start();
         bbwriter_.wait();
         timeWait.end();
         MPI_Barrier( MPI_COMM_WORLD );

         pe_LOG_INFO_SECTION( log ) {
            log << "BodyBinaryWriter::wait() took " << timeWait.total() << "s on rank " << MPISettings::rank() << ".\n";
         }
      }
#endif

      std::ofstream fout( ( checkpointsPath_ / ( name + ".txt" ) ).string().c_str() );
      fout << std::setprecision( 17 );
      fout << "t " << params.t_ << "\n"
           << "povray_t_next " << params.povray_t_next_ << "\n"
           << "impactor_present " << params.impactor_present_ << "\n";
      fout << std::flush;
   }

   void read( std::string name, bool povray, Params& params ) {
#if BODYREADER
      path p( checkpointsPath_ / boost::lexical_cast<std::string>( theMPISystem()->getRank() ) );
      bodyReader_.readFile( ( p / ( name + ".pe" ) ).string().c_str(), povray );
      if( bodyReader_.hasError() ) {
         pe_LOG_ERROR_SECTION( log ) {
            log << bodyReader_.getError() << "\n";
            log.commit();
         }

         pe::exit( EXIT_FAILURE );
      }
#else
      UNUSED( povray );
      bbreader_.readFile( ( checkpointsPath_ / ( name + ".peb" ) ).string().c_str() );
#endif

      std::ifstream fin( ( checkpointsPath_ / ( name + ".txt" ) ).string().c_str() );
      std::string key, value;
      std::map<std::string, std::string> paramMap;
      while( fin ) {
         fin >> key >> value;
         if( !fin )
            break;
         paramMap[key] = value;
      }

      params.t_                = boost::lexical_cast<real>    ( paramMap["t"] );
      params.povray_t_next_    = boost::lexical_cast<real>    ( paramMap["povray_t_next"] );
      params.impactor_present_ = boost::lexical_cast<bool>    ( paramMap["impactor_present"] );
   }

   void flush() {
#if !BODYREADER
      bbwriter_.wait();
#endif
   }

private:

#if BODYREADER
   BodyReader       bodyReader_;
#else
   BodyBinaryWriter bbwriter_;
   BodyBinaryReader bbreader_;
#endif
   path             checkpointsPath_;
};
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY TEXTURE POLICY
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Custom Texture policy.
 *
 * This class represents the POV-Ray texture policy for this example.
 */
class ImpactTexturePolicy : public TexturePolicy
{
public:
   explicit ImpactTexturePolicy( const std::string& texgranular ) : texgranular_(texgranular)
   {}

   virtual ~ImpactTexturePolicy()
   {}

   virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
   {
      switch( body->getID() )
      {
         case 0:  return CustomTexture( "TextureGround"   );
         case 1:  return CustomTexture( texgranular_      );
         case 2:  return CustomTexture( "TextureImpactor" );
         default: return CustomTexture( "TextureOther"    );
         //default: return CustomTexture( texgranular_      );
      }
   }

   using TexturePolicy::getTexture;

private:
   std::string texgranular_;
};
//*************************************************************************************************



//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mpiimpact example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The mpiimpact example is a sand bed where a big particle is impacting. The domain is periodic
 * in x- and y-direction.
 */
int main( int argc, char** argv )
{
   WcTimer setupTime, simTime, simStepTime, impactorSetupTime, adaptiveTimesteppingTime, allreduceTime;
   pe_PROFILING_SECTION {
      setupTime.start();
   }

   // MPI Initialization
   MPI_Init( &argc, &argv );

   pe_LOG_INFO_SECTION( log ) {
      char name[256];
      if( gethostname( name, sizeof(name) ) != 0 )
         log << "Cannot get hostname.\n";
      else
         log << "Running on node " << name << ".\n";
   }

   // Conversion factors
   const real   second                   ( 1.0e-3 );                  // Conversion factor from unitless time to seconds.
   const real   kilogram                 ( 1.0e-3 );                  // Conversion factor from unitless weight to kilograms.
   const real   meter                    ( 1.0e-3 );                  // Conversion factor from unitless length to meters.
   const real   newton                   ( kilogram * meter / sq( second ) );  // Conversion factor from unitless force unit to Newton.

   // Simulation parameters
   const size_t seed                     ( 12345 );                   // Seed for the random number generation
   const real   gravity                  ( 9.81 / meter * sq( second ) );  // Acceleration along negative z direction in mm/ms^2=10^3*m/s^2. The gravity of earth is 9.81m/s^2.
   //const real   fluid_density            ( 1.204e-6 );                // Mass density of fluid surrounding impactor in g/mm^3=10^6*kg/m^3. Air has a density of 1.204kg/m^3 at 20°C.

   real         t( 0 / second );                                      // The current simulation time. Initially the simulation time starts at 0.
   const real   t_end( 2 / second );                                  // The time when to end the simulation.
   const real   dt_max( 1.0e-6 / second );                            // The maximum size of the time steps limiting time discretization errors. DEM needs 50 timesteps during a typical contact duration.
   //const real   dt_max( 1.0e-5 / second );                            // The maximum size of the time steps limiting time discretization errors.  FFD
   //const real   dt_max( 1.0e-3 / second );                            // The maximum size of the time steps limiting time discretization errors.

   const int    px                       ( 3 );                       // Number of processes in x direction
   const int    py                       ( 3 );                       // Number of processes in y direction
   const int    pz                       ( 1 );                       // Number of processes in z direction

         bool   povray                   ( true );                    // Switches the POV-Ray visualization on and off.
   const size_t povray_fps               ( 1000 );                    // Target frames per second for the POV-Ray visualization.
   //const size_t povray_fps               ( 10000 );                   // Target frames per second for the POV-Ray visualization.
   const real   povray_fps_tolerance     ( 0.01 );                    // The effective fps may be off by this tolerance.
   real         povray_t_next            ( 0 );                       // The time when the next frame should be written.
   path         povray_path              ( "video/" );                // The path where to store the visualization data.

   // Properties of the granular particles:
   const size_t granular_nx              ( 8*px );                    // Number of granular particles in x direction
   const size_t granular_ny              ( 8*py );                    // Number of granular particles in y direction
   const size_t granular_nz              ( 24*pz );                   // Number of granular particles in z direction
   const real   granular_r               ( 0.25e-3 / meter );         // The radius of particles in mm. Sand grains range from 0.063mm to 2mm in size.
   const real   granular_spacing         ( 0.1 * granular_r );        // Initial spacing in-between two spherical particles in mm.
   const real   granular_dist            ( 2.0 * granular_r + granular_spacing );
   const real   granular_density         ( 2.65e-3 );                 // Density of the granular particles in g/mm^3=10^3*g/cm^3. 2.65 g/cm^3 is quartz.
   const bool   granular_spherical       ( false );                   // Granular particles are spherical.
   const real   granular_restitution     ( 0.85 );                    // Coefficient of restitution.
   const real   granular_static_cof      ( 0.85 / 2 );                // Coefficient of static friction. Roughly 0.85 with high variation depending on surface roughness for low stresses. Note: pe doubles the input coefficient of friction for material-material contacts.
   const real   granular_dynamic_cof     ( granular_static_cof );     // Coefficient of dynamic friction. Similar to static friction for low speed friction.
   const real   granular_poissonsratio   ( 0.17 );                    // Poisson's ratio for Quartz crystals is 0.17. [http://www.azom.com/article.aspx?ArticleID=1114]
   const real   granular_youngsmodulus   ( 8e4 );                     // Young's modulus of a Quartz crystal is roughly 80 GPa = 8*10^10 N/m^2 = 8*10^10 kg/(m*s^2) = 8*10^10*10^-6 g/(mm*ms^2) = 8*10^4 g/(mm*ms^2). [http://www.crystran.co.uk/quartz-crystal-sio2.htm]
   const real   granular_volume          ( (4.0 / 3.0) * granular_r * granular_r * granular_r * M_PI );
   const real   granular_mass            ( granular_volume * granular_density );
   const real   granular_stiffness       ( 4e3 * meter / newton );    // The stiffness k in normal direction of the contact region. The resulting composite stiffness is (1/k+1/k)^{-1} = k/2. In [1] the authors used k/2 = 10^5 N/m = 100 N/mm in order to achieve a contact duration of 2.033e-5s. But since we have a non-zero damping coefficient we also have a different stiffness in order to obtain a contact duration of around 2.07e-5s.
   const real   granular_dampingN        ( -4 * std::log( granular_restitution ) * std::sqrt( ( granular_mass / 2 ) * ( sq( std::log( granular_restitution ) ) + sq( M_PI ) ) * ( granular_stiffness / 2 ) ) / ( sq( std::log( granular_restitution ) ) + sq( M_PI ) ) );  // Calculate damping coefficient so that a linear normal force model produces the correct COR for a two particle collision. WARNING: The underlying formula presented for example in [2] is not valid if adhesive forces a truncated. Since the pe truncates these forces the effective COR will be higher than specified especially for low COR.
   const real   granular_dampingT        ( 1 );                       // The damping coefficient in tangential direction of the contact region - a technical parameter which must be "high enough".

   // Calculate domain \Omega = [0; lx] x [0; ly] x (0; lz):
   const real   lx                       ( granular_nx * granular_dist );                                       // Length of the simulation domain in x-dimension.
   const real   ly                       ( granular_ny * granular_dist );                                       // Length of the simulation domain in y-dimension.
   const real   lz                       ( granular_nx * granular_ny * granular_nz * granular_volume / ( lx * ly * ( granular_spherical ? 0.64 : 0.80 ) ) );  // Estimate of the height of the random closed packing. The density of such packings is known to be roughly 0.64. Experimentally the density of the packing of granular particles was evaluated to be 0.80 which is considerably higher.
   //const real   lz                       ( impactor_height + 2.0 * impactor_r + granular_nz * granular_dist );  // Length of the simulation domain in z-dimension.

   const real   granular_freefall        ( 1.3 * std::sqrt( 2 * ( granular_nz * granular_dist - lz ) / gravity ) );  // Time for the top-most particles to (freely) fall to the expected height of the dense packing plus some safety margin.
   const real   granular_contact_duration( M_PI / std::sqrt( ( granular_stiffness / 2 ) / ( granular_mass / 2 ) - sq( 0.5 * granular_dampingN / ( granular_mass / 2 ) ) ) );  // Time for a collision of two self-similar particles
   const real   granular_deletion_threshold( std::sqrt( 2 * gravity * (2*granular_r) ) ); // Estimate some velocity as of which particles are removed if they are about to leave the domain on one side. Here: Velocity after a free fall of a particle height.
   UNUSED( granular_deletion_threshold );

   // Properties of the impactor:
         real   impactor_r               ( 20 * granular_r );                // The radius of the impactor in mm.
   const real   impactor_density         ( granular_density );               // Density of the impactor in g/mm^3=10^3*g/cm^3.
   const bool   impactor_spherical       ( true );                           // Impactor is spherical.
   //const real   impactor_dragcoeff       ( 0.47 );                           // The drag coefficient for the impactor. Typically 0.47 for spheres.
   //const real   impactor_maxspeed        ( sqrt( impactor_r * gravity * 8.0 / 3.0 / impactor_dragcoeff * impactor_density / fluid_density ) );  // Maximum speed of the impactor computed from the drag equation.
   //const real   impactor_speed           ( 0.6 * impactor_maxspeed );        // Velocity of the impactor along the negative z direction in mm/ms=m/s.
   const real   impactor_speed           ( std::sqrt( 2 * 1 / meter * gravity ) );  // Velocity of the impactor along the negative z direction in mm/ms=m/s. Speed when falling from 1m height.
   const real   impactor_height          ( 0 );                              // The approximate initial height of the lower end of the impactor above the granular matter in mm.
         bool   impactor_present         ( false );

   WorldID      world                    ( theWorld() );

   //----------------------------------------------------------------------------------------------
   // Parsing the command line arguments

   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.getDescription().add_options()
      ( "resume", value<std::string>()->default_value( "" ), "the checkpoint to resume" );
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;
   std::string resume( vm[ "resume" ].as<std::string>() );

   //----------------------------------------------------------------------------------------------
   // Setup of Domain Decomposition

   RectilinearGrid grid;
   grid.connect( Vec3(0, 0, 0), Vec3(lx, ly, lz), Vector3<size_t>(px, py, pz), Vector3<BoundaryCondition>( boundaryConditionPeriodic, boundaryConditionPeriodic, boundaryConditionOpen ), Vector3<BoundaryCondition>( boundaryConditionPeriodic, boundaryConditionPeriodic, boundaryConditionOpen ) );
   RectilinearGrid::AABB aabb( grid.getCell() );
   pe_LOG_INFO_SECTION( log ) {
      log << "The domain decomposition grid comprises the domain [0; " << lx << "] x [0; " << ly << "] x (0; " << lz << "), where periodic boundary conditions are applied in x- and y- direction and open boundary conditions in both z-directions.\n";
   }

   MPI_Comm cartcomm    = MPISettings::comm();
   real     bounds_x[2] = {aabb[0], aabb[3]};
   real     bounds_y[2] = {aabb[1], aabb[4]};
   size_t   coords[3]   = {grid.getCoords()[0], grid.getCoords()[1], grid.getCoords()[2]};

   //----------------------------------------------------------------------------------------------
   // Setup of the POV-Ray visualization

   WriterID pov;
   size_t povray_frame( 0 );

   if( povray ) {
      //const Vec3 location( -0.725, 0, 0.2625 );
      const Vec3 location( real(0.5)*lx, -real(0.5)*ly/0.65, real(0.5)*( impactor_height + 2.0 * impactor_r + granular_nz * granular_dist ) );
      const Vec3 focus   ( real(0.5)*lx,  real(0.5)*ly,      real(0.5)*granular_nz*granular_dist );

      pov = activateWriter();
#if ADAPTIVETIMESTEPPING
      pov->setSpacing( std::numeric_limits<size_t>::max() );
#else
      pov->setSpacing( (std::size_t) std::ceil( 1 / ( second * dt_max * povray_fps ) ) );
#endif
      pov->setFilename( ( povray_path / "pic%.pov" ).string().c_str() );
      pov->setBackground( Color( 0, 0, 0 ) );
      pov->include( "settings.inc" );

      pov->addLightSource( PointLight( location, Color( 1, 1, 1 ) ) );

      CameraID camera = theCamera();
      camera->setLocation( location );
      camera->setFocus   ( focus    );

      std::ostringstream texture;
      texture << "Texture" << (coords[0] + coords[1] + coords[2])%2;
      pov->setTexturePolicy( ImpactTexturePolicy( texture.str() ) );
   }

   //----------------------------------------------------------------------------------------------
   // Setup of the simulation domain
   setSeed( seed );                      // Supply initial seed for the random number generation.
   world->setGravity( 0, 0, -gravity );  // Take over gravity.
   world->setDamping( 1 );               // Deactivate damping.

   MaterialID granular_material = createMaterial( "granular", granular_density, granular_restitution, granular_static_cof, granular_dynamic_cof, granular_poissonsratio, granular_youngsmodulus, granular_stiffness, granular_dampingN, granular_dampingT );
   MaterialID impactor_material = createMaterial( "impactor", impactor_density, granular_restitution, granular_static_cof, granular_dynamic_cof, granular_poissonsratio, granular_youngsmodulus, granular_stiffness, granular_dampingN, granular_dampingT );
   MaterialID boundary_material = createMaterial( "boundary", 1,                0,                    granular_static_cof, granular_dynamic_cof, granular_poissonsratio, granular_youngsmodulus, granular_stiffness, granular_dampingN, granular_dampingT );

   Checkpointer         checkpointer;
   Checkpointer::Params checkpointer_params( t, povray_t_next, impactor_present );
   bool                 checkpoint_next( false );                     // Write out checkpoint in next iteration
   path                 checkpoint_path( "checkpoints/" );            // The path where to store the checkpointing data

   if( resume.empty() ) {
      // Setup of ground plane
      pe_GLOBAL_SECTION
      {
         PlaneID plane = createPlane( 0,  0.0,  0.0,  1.0, 0.0, boundary_material, true );
         if( povray )
            pov->setTexture( plane, CustomTexture( "TextureGround" ) );

         //createPlane( 0,  1.0,  0.0,  0.0, 0.0, boundary_material, false );
         //createPlane( 0, -1.0,  0.0,  0.0, -lx, boundary_material, false );
         //createPlane( 0,  0.0,  1.0,  0.0, 0.0, boundary_material, false );
         //createPlane( 0,  0.0, -1.0,  0.0, -ly, boundary_material, false );
      }

      // Deterministic setup of the particles (iterate over all points in the grid which are strictly inside our subdomain or on _any_ boundary)
      size_t limit_x = (int)floor( ( bounds_x[1] + real(0.5) * granular_dist ) / granular_dist );
      size_t limit_y = (int)floor( ( bounds_y[1] + real(0.5) * granular_dist ) / granular_dist );
      for( size_t i_x = (int)ceil( ( bounds_x[0] + real(0.5) * granular_dist ) / granular_dist - real(1) ); i_x < limit_x; ++i_x ) {
         for( size_t i_y = (int)ceil( ( bounds_y[0] + real(0.5) * granular_dist ) / granular_dist - real(1) ); i_y < limit_y; ++i_y ) {
            for( size_t i_z = 0; i_z < granular_nz; ++i_z ) {
               Vec3 position(
                     ( i_x + real(0.5) ) * granular_dist,
                     ( i_y + real(0.5) ) * granular_dist,
                     ( i_z + real(0.5) ) * granular_dist );

               // Explicitly test points whether they are located on the boundary
               if( !theWorld()->ownsPoint( position ) )
                  continue;

               BodyID particle( NULL );
               if( granular_spherical )
                  particle = createSphere( 1, position, granular_r, granular_material );
                  //particle = createSphere( 3 + i_x + i_y * granular_nx + i_z * granular_nx * granular_ny , position, granular_r, granular_material );
               else
                  particle = createGranularParticle( 1, position, granular_r, granular_material );
                  //particle = createGranularParticle( 3 + i_x + i_y * granular_nx + i_z * granular_nx * granular_ny, position, granular_r, granular_material );

               real angle = pe::rand( real(0), 2*M_PI );
               real vel   = pe::rand( real(0), granular_spacing / granular_freefall );
               particle->setLinearVel( vel * cos( angle ), vel * sin( angle ), 0 );
            }
         }
      }

      if( povray )
         pov->writeFile( ( povray_path / "init.pov" ).string().c_str() );
      povray_t_next = real(1) / ( povray_fps * second );
   }
   else {
      // Resume from checkpoint
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Resuming checkpoint \"" << resume << "\"..." << std::endl;
      }

      checkpointer.setPath( checkpoint_path / resume );
      checkpointer.read( resume, povray, checkpointer_params );
      // PovRay textures are reassigned through texture policy

      if( povray )
         pov->writeFile( ( povray_path / "resume.pov" ).string().c_str() );
      povray_t_next = t + real(1) / ( povray_fps * second );
   }

   // Synchronization of the MPI processes
   world->synchronize();

   //----------------------------------------------------------------------------------------------
   // Parameter estimations and assertions

   pe_LOG_INFO_SECTION( log ) {
      log << "Contact duration of granular particles in a linear viscoelastic model: " << granular_contact_duration << "\n";
      log << "Lower limit for time steps per contact duration: " << std::floor( granular_contact_duration / dt_max ) << "\n";
      log << "Contact damping in a linear viscoelastic model for a COR of " << granular_restitution << ": " << granular_dampingN << "\n";
      log << "Estimated height of random close packing of particles: " << lz << "\n";
   }

   if( granular_contact_duration < 10 * dt_max ) {
      // According to [1] the time step must be 50 times smaller than the contact duration. According to [2] 100 times smaller.
      pe_LOG_WARNING_SECTION( log ) {
         log << "Time step is too large to resolve contact in a linear viscoelastic model.\n";
      }
   }

   pe_INTERNAL_ASSERT( lz < granular_nz * granular_dist, "Initial packing too dense to estimate duration of settling phase." );
   pe_INTERNAL_ASSERT( real(1) / ( povray_fps * second ) > dt_max, "Visualization rate too high." );

   if( impactor_r >= std::min( std::min( lx/px, ly/py ), lz/pz ) ) {
      impactor_r = 0.99 * std::min( std::min( lx/px, ly/py ), lz/pz );
      pe_LOG_WARNING_SECTION( log ) {
         log << "Impactor radius capped in order for nearest neighbor communication to suffice.\n";
      }
   }

   pe_INTERNAL_ASSERT( impactor_r < std::min( std::min( lx/px, ly/py ), lz/pz ), "Impactor too large for nearest neighbor communication." );
   pe_INTERNAL_ASSERT( granular_r < std::min( std::min( lx/px, ly/py ), lz/pz ), "Granular matter too large for nearest neighbor communication." );

   //----------------------------------------------------------------------------------------------
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "------------------------------------------------------------\n"
                << " Size of the domain                  = (" << lx << "mm, " << ly << "mm, " << lz << "mm)\n"
                << " Number of MPI processes             = (" << px << ", " << py << ", " << pz << ")\n"
                << " Visualization framerate             = "  << povray_fps << "fps\n"
                << " Total number of particles           = "  << granular_nx*granular_ny*granular_nz << "\n"
                << " Particles per process               = "  << granular_nx/px << "x" << granular_ny/py << "x" << granular_nz/pz << " = " << granular_nx * granular_ny * granular_nz/(px*py*pz) << "\n"
                << " Radius of a single particle         = "  << granular_r << "mm\n"
                << " Initial spacing between particles   = "  << granular_spacing << "mm\n"
                << " Radius of impactor                  = "  << impactor_r << "mm\n"
                << " Seed of the random number generator = "  << getSeed() << "\n"
                << " Speed of impactor                   = "  << impactor_speed << "m/s\n"
                << " Time to settle                      = "  << granular_freefall << "ms\n"
                << " Maximum time step                   = "  << dt_max << "ms\n"
                << "------------------------------------------------------------------------------" << std::endl;
   }

   //----------------------------------------------------------------------------------------------
   // Simulation loop

   size_t ts( 0 );
   size_t ts_max( std::numeric_limits<size_t>::max() );
   double last_output( 0 );
   real   dt_min( inf );
   bool   statusupdate_next( false );
   std::vector<real> buffer( 4 );

   pe_PROFILING_SECTION {
      setupTime.end();
      MPI_Barrier( MPI_COMM_WORLD );
   }
   simTime.start();

   while( t < t_end && ts < ts_max ) {
      // Add impactor
      if( !impactor_present && t >= granular_freefall ) {
         pe_PROFILING_SECTION {
            impactorSetupTime.start();
         }

         // Write out checkpoint
         checkpointer.setPath( checkpoint_path / "settled" );
         checkpointer.write( "settled", checkpointer_params );

         impactor_present = true;

         real local_granular_maxheight( 0 ), global_granular_maxheight( 0 );
         for( World::Iterator it = world->begin(); it != world->end(); ++it )
            if( it->getID() == 1 )
               local_granular_maxheight = std::max( local_granular_maxheight, it->getPosition()[2] + granular_r );
         MPI_Allreduce( &local_granular_maxheight, &global_granular_maxheight, 1, MPITrait<real>::getType(), MPI_MAX, cartcomm );

         Vec3 position( 0.5 * lx, 0.5 * ly, global_granular_maxheight + granular_r + impactor_height + impactor_r );
         //Vec3 position( 0.5 * lx, 0.5 * ly, granular_nz * granular_dist + impactor_height + impactor_r );

         if( theWorld()->ownsPoint( position ) ) {
            pe_LOG_INFO_SECTION( log ) {
               log << "Adding impactor on rank " << MPISettings::rank() << ".\n";
            }
            BodyID impactor;
            if( impactor_spherical ) {
               SphereID obj = createSphere( 2, position, impactor_r, impactor_material );
               if( povray )
                  pov->setTexture( obj, CustomTexture( "TextureImpactor" ) );
               impactor = obj;
            }
            else {
               UnionID obj = createGranularParticle( 2, position, impactor_r, impactor_material );
               if( povray )
                  pov->setTexture( obj, CustomTexture( "TextureImpactor" ) );
               impactor = obj;
            }
            impactor->setLinearVel( 0, 0, -impactor_speed );
         }

         pe_LOG_INFO_SECTION( log ) {
            log << "Adding impactor at t " << t << " (" << ts << ") with speed " << impactor_speed << ".\n"
                << "Granular matter height is " << global_granular_maxheight + granular_r << ".\n";
         }

         world->synchronize();

         pe_PROFILING_SECTION {
            impactorSetupTime.end();
            MPI_Barrier( MPI_COMM_WORLD );
         }
      }

#if ADAPTIVETIMESTEPPING
      pe_PROFILING_SECTION {
         adaptiveTimesteppingTime.start();
      }

      // Adaptive time-stepping
      real surface_speed1 = 0, surface_speed2 = 0;
      bool sync( false );
      for( World::Iterator it = world->begin(); it != world->end(); ) {
         if( impactor_present && it->getID() == 1 && !it->isRemote() && ( ( it->getPosition()[0] < granular_r*1.1 && it->getLinearVel()[0] < -granular_deletion_threshold ) || ( it->getPosition()[0] > lx - granular_r*1.1 && it->getLinearVel()[0] > granular_deletion_threshold ) || ( it->getPosition()[1] < granular_r*1.1 && it->getLinearVel()[1] < -granular_deletion_threshold ) || ( it->getPosition()[1] > ly - granular_r*1.1 && it->getLinearVel()[1] > granular_deletion_threshold ) ) ) {
            if( it->sizeProcesses() > 0 )
               sync = true;
            it = world->destroy( it );
            continue;
         }

         real surface_speed_limit;
         if( it->getID() == 2 )
            surface_speed_limit = it->getLinearVel().length() + it->getAngularVel().length() * impactor_r;
         else
            surface_speed_limit = it->getLinearVel().length() + it->getAngularVel().length() * granular_r;

         if( surface_speed_limit >= surface_speed1 ) {
            surface_speed2 = surface_speed1;
            surface_speed1 = surface_speed_limit;
         }

         ++it;
      }

      real dt_global, dt_local = std::min( 2 * granular_r, 2 * impactor_r ) * 0.01 / ( surface_speed1 + surface_speed2 );
      dt_local = std::min( dt_local, dt_max );

      buffer[0] = dt_local;
      buffer[1] = sync ? 0 : 1;
      buffer[2] = checkpoint_next ? 0 : 1;
      buffer[3] = statusupdate_next ? 0 : 1;

      pe_PROFILING_SECTION {
         MPI_Barrier( MPI_COMM_WORLD );
         allreduceTime.start();
      }
      MPI_Allreduce( MPI_IN_PLACE, &buffer[0], buffer.size(), MPITrait<real>::getType(), MPI_MIN, cartcomm );
      pe_PROFILING_SECTION {
         allreduceTime.end();
      }
      dt_global       = buffer[0];
      sync            = buffer[1] == 0;
      checkpoint_next = buffer[2] == 0;
      statusupdate_next = buffer[3] == 0;

      if( checkpoint_next )
         break;

      if( sync )
         world->synchronize();

      if( statusupdate_next ) {
         statusupdate_next = false;

         MemoryMeter mm;
         mm.stop();

         pe_LOG_INFO_SECTION( log ) {
            log << "------------------------------------------------------------------------------\n"
                << " Total memory allocated      = " << t << " " << mm.lastAllocation() << "bytes\n"
                << " Total memory in use         = " << t << " " << mm.lastInUse()     << "bytes\n"
                << "------------------------------------------------------------------------------\n";
         }

         theCollisionSystem()->logProfilingSummary();
      }

      // Visualization time-stepping modification
      if( povray ) {
         if( std::fabs( t + dt_global - povray_t_next ) * povray_fps * second < povray_fps_tolerance ) {
            pov->setSpacing( 1 );
            pe_EXCLUSIVE_SECTION( 0 ) {
               std::cout << "Writing out frame " << povray_frame << " at " << t + dt_global << " instead of " << povray_t_next << " since we are just off by " << std::fabs( t + dt_global - povray_t_next ) * povray_fps * second << std::endl;
            }
            povray_frame++;
            povray_t_next += real(1) / ( povray_fps * second );
         }
         else if( t + dt_global > povray_t_next ) {
            pov->setSpacing( 1 );
            dt_global = povray_t_next + povray_fps_tolerance / ( povray_fps * second ) - t;
            pe_EXCLUSIVE_SECTION( 0 ) {
               std::cout << "Forcing write out frame " << povray_frame << " at " << t + dt_global << " instead of " << povray_t_next << " as initially planed. This is just off by " << std::fabs( t + dt_global - povray_t_next ) * povray_fps * second << std::endl;
            }
            povray_frame++;
            povray_t_next += real(1) / ( povray_fps * second );
            pe_INTERNAL_ASSERT( dt_global > 0, "Invalid time step size." );
         }
      }

      pe_PROFILING_SECTION {
         adaptiveTimesteppingTime.end();
         MPI_Barrier( MPI_COMM_WORLD );
      }
#else
      real dt_global( dt_max );

      UNUSED( statusupdate_next );
      UNUSED( povray_frame );
      UNUSED( povray_fps_tolerance );
#endif

      dt_min = std::min( dt_global, dt_min );

      pe_PROFILING_SECTION {
         simStepTime.start();
      }

      world->simulationStep( dt_global );

      pe_PROFILING_SECTION {
         simStepTime.end();
         MPI_Barrier( MPI_COMM_WORLD );
      }

      ++ts;
      t += dt_global;

      pe_EXCLUSIVE_SECTION( 0 ) {
         simTime.end();
         if( simTime.total() - last_output >= 60 ) {
            last_output = simTime.total();

            statusupdate_next = true;

            std::cout << "------------------------------------------------------------------------------\n"
                      << " Simulation time             = " << t << "ms\n"
                      << " Number of timesteps         = " << ts << "\n"
                      << " Total Simulation Time       = " << simTime.total() << "s (" << simTime.average() << "s per timestep)\n"
                      << " Total Simulation Time pe    = " << simStepTime.total() << "s (" << simStepTime.average() << "s per timestep)\n"
                      << " Timestep size               = " << dt_global << "ms\n"
                      << "------------------------------------------------------------------------------\n" << std::endl;
         }

         // Trigger checkpointing if we run out of time (Disclaimer: This only works with adaptive timestepping)
         if( simTime.total() >= 24*60*60 - 5*60 )
            checkpoint_next = true;

         simTime.start();
      }

#if ADAPTIVETIMESTEPPING
      if( povray )
         pov->setSpacing( std::numeric_limits<size_t>::max() );
#endif
   }
   pe_PROFILING_SECTION {
      simTime.end();
   }

   if( checkpoint_next ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "Note: Writing out checkpoint before running out of time." << std::endl;
      }
      checkpointer.setPath( checkpoint_path / "intermediate" );
      checkpointer.write( "intermediate", checkpointer_params );

      if( povray )
         pov->writeFile( ( povray_path / "intermediate.pov" ).string().c_str() );
   }
   else {
      checkpointer.setPath( checkpoint_path / "end" );
      checkpointer.write( "end", checkpointer_params );

      if( povray )
         pov->writeFile( ( povray_path / "end.pov" ).string().c_str() );
   }

   //----------------------------------------------------------------------------------------------
   // Simulation timing results

   pe_EXCLUSIVE_SECTION( 0 ) {
      pe_LOG_INFO_SECTION( log ) {
         log << "------------------------------------------------------------------------------\n"
             << " Simulation time             = " << t << "ms\n"
             << " Number of timesteps         = " << ts << "\n"
             << " Minimum timestep size       = " << dt_min << "ms\n"
             << "------------------------------------------------------------------------------\n\n"
             << "Timing results reduced over all time steps on current process:\n" << std::fixed << std::setprecision(4)
             << "code part              min time     max time     avg time     total time   executions\n"
             << "--------------------   ----------   ----------   ----------   ----------   ----------\n"
             << "setup                  " << std::setw(10) << setupTime.min() << "   " << std::setw(10) << setupTime.max() << "   " << std::setw(10) << setupTime.average() << "   " << std::setw(10) << setupTime.total() << "   " << std::setw(10) << setupTime.getCounter() << "\n"
             << "simulation step        " << std::setw(10) << simTime.min() << "   " << std::setw(10) << simTime.max() << "   " << std::setw(10) << simTime.average() << "   " << std::setw(10) << simTime.total() << "   " << std::setw(10) << simTime.getCounter() << "\n"
             << "  simulation step pe   " << std::setw(10) << simStepTime.min() << "   " << std::setw(10) << simStepTime.max() << "   " << std::setw(10) << simStepTime.average() << "   " << std::setw(10) << simStepTime.total() << "   " << std::setw(10) << simStepTime.getCounter() << "\n"
             << "  impactor setup       " << std::setw(10) << impactorSetupTime.min() << "   " << std::setw(10) << impactorSetupTime.max() << "   " << std::setw(10) << impactorSetupTime.average() << "   " << std::setw(10) << impactorSetupTime.total() << "   " << std::setw(10) << impactorSetupTime.getCounter() << "\n"
             << "  adap. timestepping   " << std::setw(10) << adaptiveTimesteppingTime.min() << "   " << std::setw(10) << adaptiveTimesteppingTime.max() << "   " << std::setw(10) << adaptiveTimesteppingTime.average() << "   " << std::setw(10) << adaptiveTimesteppingTime.total() << "   " << std::setw(10) << adaptiveTimesteppingTime.getCounter() << "\n"
             << "    all reduce         " << std::setw(10) << allreduceTime.min() << "   " << std::setw(10) << allreduceTime.max() << "   " << std::setw(10) << allreduceTime.average() << "   " << std::setw(10) << allreduceTime.total() << "   " << std::setw(10) << allreduceTime.getCounter() << "\n"
             << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
      }
   }

   theCollisionSystem()->logProfilingSummary();

   //----------------------------------------------------------------------------------------------
   // MPI Finalization

   checkpointer.flush();
   MPI_Finalize();
}
//*************************************************************************************************

// [1] O. Mouraille, W.A. Mulder and S. Luding, Mechanic Waves in Sand, 3d Simulations
// [2] J. Schäfer, S. Dippel and D.E. Wolf, Force Schemes in Simulations of Granular Materials, J. Phys. I France 6 (1996) 5-20
