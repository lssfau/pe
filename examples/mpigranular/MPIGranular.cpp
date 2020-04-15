//=================================================================================================
/*!
 *  \file MPIGranular.cpp
 *  \brief Example file for the pe physics engine
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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
#include <boost/lexical_cast.hpp>
#include <pe/engine.h>
#include <pe/support.h>
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace pe;
using namespace pe::timing;
using namespace pe::povray;
using boost::filesystem::path;




// Assert statically that only the FFD solver or a hard contact solver is used since parameters are tuned for them.
#define pe_CONSTRAINT_MUST_BE_EITHER_TYPE(A, B, C) typedef \
   pe::CONSTRAINT_TEST< \
      pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< \
         pe::IsSame<A,B>::value | pe::IsSame<A,C>::value \
      >::value > \
   pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ );

typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::FFDSolver>::Config TargetConfig1;
typedef Configuration< pe_COARSE_COLLISION_DETECTOR, pe_FINE_COLLISION_DETECTOR, pe_BATCH_GENERATOR, response::HardContactSemiImplicitTimesteppingSolvers>::Config TargetConfig2;
pe_CONSTRAINT_MUST_BE_EITHER_TYPE(Config, TargetConfig1, TargetConfig2);




//*************************************************************************************************
class Checkpointer {
public:

   struct Params {
      Params( size_t &t, Vec3& pivotPoint )
         : t_( t ), pivotPoint_( pivotPoint ) {}

      size_t &t_;
      Vec3 &pivotPoint_;
   };

   Checkpointer( path checkpointsPath = path( "checkpoints/" ) ) : checkpointsPath_( checkpointsPath ) {
   }

   void setPath( path checkpointsPath = path( "checkpoints/" ) ) {
      checkpointsPath_ = checkpointsPath;
   }

   void write( std::string name, const Params &params ) {
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

      std::ofstream fout( ( checkpointsPath_ / ( name + ".txt" ) ).string().c_str() );
      fout << "t " << params.t_ << "\n";
      fout << "pivotPointX " << params.pivotPoint_[0] << "\n";
      fout << "pivotPointY " << params.pivotPoint_[1] << "\n";
      fout << "pivotPointZ " << params.pivotPoint_[2] << "\n";
      fout << std::flush;
   }

   void read( std::string name, bool povray, Params& params ) {
      UNUSED( povray );
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

      params.t_                = boost::lexical_cast<size_t>    ( paramMap["t"] );
      params.pivotPoint_[0]    = boost::lexical_cast<real>      ( paramMap["pivotPointX"] );
      params.pivotPoint_[1]    = boost::lexical_cast<real>      ( paramMap["pivotPointY"] );
      params.pivotPoint_[2]    = boost::lexical_cast<real>      ( paramMap["pivotPointZ"] );
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
//  POVRAY CAMERA ANIMATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Animation for the POV-Ray camera.
 *
 * This class animates the POV-Ray camera for the MPIGranular example. The camera is initially
 * located close to the particles to allow a close look at them before slowly shifting backwards
 * to give a complete view of the simulation scenario.
 */
class LinearShift : public CameraAnimation
{
 public:
   explicit LinearShift( size_t first, size_t last, const Vec3& begin, const Vec3& end )
      : first_   ( first )  // First time step of the camera animation
      , last_    ( last  )  // Last time step of the camera animation
      , begin_   ( begin )  // Initial location of the camera
      , end_     ( end   )  // Final location of the camera
   {}

   virtual ~LinearShift()
   {}

   virtual void updateLocation( Vec3& location )
   {
      if( TimeStep::step() > first_ && TimeStep::step() < last_ ) {
         const Vec3 disp( ( begin_ - end_ ) / ( last_ - first_ ) );
         location -= disp;
      }
   }

 private:
   size_t first_;  // First time step of the camera animation
   size_t last_;   // Last time step of the camera animation
   Vec3 begin_;    // Initial location of the camera
   Vec3 end_;      // Final location point of the camera
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
         default: return CustomTexture( "GlassTexture"     );  // Texture for the outlet
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
/*!\brief Main function for the mpigranular example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The mpigranular example is a pseudo-3D hopper flow.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Simulation parameters

   // Particle parameters
   const bool   spheres ( false );  // Switch between spheres and granular particles
   const real   radius  (  0.35 );  // The radius of particles of the granular media
   const real   spacing (  0.05 );  // Initial spacing inbetween two particles
   const real   velocity(  0.01 );  // Initial maximum velocity of the particles

   // Environment parameters
   const real   lx     (   30.0 );  // Size of the outlet in x-dimension
   const real   ly     (    5.0 );  // Size of the outlet in y-dimension
   const real   lz     (   60.0 );  // Size of the outlet in z-dimension
   const real   width  (    0.7 );  // Width of the outlet walls
   const real   opening(    5.0 );  // Size of the outlet door
   const real   angle  ( M_PI/8 );  // Angle of the lower outlet walls

   // Time parameters
   const size_t initsteps     (  20000 );  // Initialization steps with closed outlet door
   const size_t focussteps    (    100 );  // Number of initial close-up time steps
   const size_t animationsteps(    200 );  // Number of time steps for the camera animation
   const size_t opensteps     (   2000 );  // Number of time steps for the door opening
   const size_t timesteps     ( 180000 );  // Number of time steps for the flowing granular media
   const real   stepsize      (  0.001 );  // Size of a single time step

   // Process parameters
   const int processesX( 3 );  // Number of processes in x-direction
   const int processesZ( 2 );  // Number of processes in z-direction
   const int processesO( 1 );  // Number of processes at the outlet opening

   // Random number generator parameters
   const size_t seed( 12345 );

   // Verbose mode
   const bool verbose( false );  // Switches the output of the simulation on and off

   // Visualization
   bool povray( true );  // Switches the POV-Ray visualization on and off
   path povray_path( "video/" );

   // Visualization parameters
   const bool   colorProcesses( false );  // Switches the processes visualization on and off
   const bool   animation     (  true );  // Switches the animation of the POV-Ray camera on and off
   const size_t visspacing    (   150 );  // Number of time steps inbetween two POV-Ray files
   const size_t colorwidth    (    10 );  // Number of spheres in z-dimension with a specific color


   /////////////////////////////////////////////////////
   // Initial setups

   const int  processes( theMPISystem()->getSize() );  // The total number of active MPI processes
   const int  rank     ( theMPISystem()->getRank() );  // The rank of the MPI process

   const real space( real(2)*radius+spacing );  // Space initially required by a single particle
   const real hlx  ( real(0.5)*lx+width );      // Half total x-size
   const real plx  ( real(2)*hlx/processesX );  // X-size of a MPI process
   const real plz  ( lz/processesZ );           // Z-size of a MPI process
   const real tana ( std::tan( angle ) );       // Precalculation of the tangent of the 'angle'
   const real minz ( -hlx*tana-real(6) );       // Threshold value for the destruction of bodies
         Vec3 location1, location2;

   // Checking the ratio of the particle radius to the outlet dimensions
   if( space > real(10)*lx || space > real(3)*ly || space > real(10)*lz ) {
      std::cerr << pe_RED
                << "\n Particle radius is too large for the chosen dimensions of the outlet!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the ratio of the particles to the dimensions of a single process
   if( space > plx || space > plz ) {
      std::cerr << pe_RED
                << "\n Particle radius is too large for the chosen process layout!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the angle of the lower outlet walls
   if( angle < real(0) || angle >= real(M_PI/2) ) {
      std::cerr << pe_RED
                << "\n Invalid angle for the lower outlet walls!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the total number of MPI processes
   if( processesX <= 0 || processesZ <= 0 || processesO <= 0 ||
       processes < 2 || processesX*processesZ+processesO != processes ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Setup of the random number generation
   setSeed( seed );

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

   // Creating the material for the particles
   MaterialID granular = createMaterial( "granular", 1.0, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );
   world->setDamping( 0.9 );

   size_t               t( 0 );
   Vec3                 pivotPoint;  // Pivot point for the outlet lid
   Checkpointer         checkpointer;
   Checkpointer::Params checkpointer_params( t, pivotPoint );
   path                 checkpoint_path( "checkpoints/" );            // The path where to store the checkpointing data

   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;

   CustomTexture groundTexture ( "GroundTexture" );
   CustomTexture glassTexture  ( "GlassTexture"  );

   if( povray )
   {
      // Calculation of the camera location and focus point
      // Aspect ratio of the POV-Ray camera: 4/3 => Up-angle = 26.565 deg (0.4636 rad)
      const real x( real(0.3)*lx );
      const real y( real(16.25) / std::tan(0.4636) );
      const real z( real(12.5) - real(0.5)*tana*( lx+2*width-opening ) );
      const Vec3 focus( 0.0   , 0.0, z );

      location1 = Vec3( x*ly/y, -ly, z );
      location2 = Vec3( x     , -y , z );

      // Configuration of the POV-Ray writer
      pov = activateWriter();
      pov->setSpacing( 1 );
      pov->setFilename( ( povray_path / "pic%.pov" ).string().c_str() );
      pov->include( "settings.inc" );
      pov->setBackground( Color( 0, 0, 0 ) );

      // Configuration of the scene lighting
      pov->addLightSource( PointLight   ( location2        , Color( 0.5, 0.5, 0.5 ) ) );
      pov->addLightSource( ParallelLight( Vec3( 0, -50, 0 ), Color( 1.0, 1.0, 1.0 ) ) );
      pov->addLightSource( ParallelLight( Vec3( 0, 0, 100 ), Color( 1.0, 1.0, 1.0 ), Shadowless() ) );

      // Configuration of the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( ( animation )?( location1 ):( location2 ) );
      camera->setFocus   ( focus );

      // Configuration of the texture policy
      if( colorProcesses ) {
         std::ostringstream texture;
         texture << "Texture" << theMPISystem()->getRank()%13;
         pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
      }
      else {
         pov->setTexturePolicy( GranularTexturePolicy() );
      }
   }


   /////////////////////////////////////////////////////
   // Setup of the MPI processes

   // WARNING: Not all code paths of the domain decomposition have been tested since the rewrite.
   // Tested Configurations include:
   //    processesX=5, processesZ=3, processesO=1
   //    processesX=4, processesZ=2, processesO=2

   /*  hpx = 3                      hpx = 3
    *  processesX = 6               processesX = 5
    *  processesZ = 2               processesZ = 2
    *  processesO > 1               processesO > 1
    *
    *                dlx
    *                |^|
    *                                           dlx
    *      +           +                        |^|
    *      |\         /|
    *  z ^ + +       + +  -         z ^ +         +
    *    | |\|\     /|/|  > dlz       | |\       /|
    *  0 + +p+ +   + + +  -           | + +     + +  -
    *    |  \|\|\ /|/|/               | |\|\   /|/|  > dlz
    *    |   + + + + +               0+ +p+ +-+ + +  -
    *    |    \|\|/|/                 |  \|\| |/|/
    *    |     + + +                  |   + +-+ +
    *    |      \|/ angle             |    \| |/ angle
    *    |       + - - - -            |     +-+ - - - -
    *    |  rank: processesO-1        |  rank: processesO-1
    *    | --------------             | --------------
    *    |      ...                   |      ...
    *    | ---------------            | ---------------
    *    |   rank: 0                  |   rank: 0
    *    |                            |
    *    +-+-----+--->                +-+----+---->
    *     -hlx   0   x                 -hlx  0   x
    *
    *
    *  Process p has rank processesO. The process ranks increase in a row-major order that is from left to right first in the hopper and then from bottom to top.
    */

   const int  hpx ( processesX/2 );             // Half number of processes in x-direction (rounded down)
   const real dlx ( (lx+2*width)/processesX );  // X-size of the processes
   const real dlz ( real(0.7)*lz/processesZ );  // Z-size of the processes

   const Vec3 ln  ( Quat( Vec3(0,1,0),  angle ).rotate( Vec3(0,0,1) ) );  // Normal in the left half
   const Vec3 rn  ( Quat( Vec3(0,1,0), -angle ).rotate( Vec3(0,0,1) ) );  // Normal in the right half
   const Vec3 cn  ( Vec3(0, 0, 1) );  // Normal in the middle (if processesX is odd)

   if( processesX == 1 ) {
      if( rank < processesO ) {
         // Connecting the bottom processes (slice space below outlet into x-y-layers, rank 0 is bottom-most layer)
         if( processesO == 1 ) {
            // Special case: only one domain below hopper
            assert( rank == processesO - 1 );

            // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
            //             |<-- point in plane                                      -->|   |<-- normal -->|
            const real cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + 0 * dlz ) * cn               );

            // Define local domain of the topmost bottom process (top-most layer of layers below outlet)
            defineLocalDomain( HalfSpace( -cn, -cd ) );

            // Connecting the top neighbor of the topmost bottom process (top-most layer of layers below outlet)
            connect( processesO, HalfSpace( +cn, +cd ) );
         }
         else {
            const real ztmp( ( processesX > 1 && processesX%2 )?( -(hpx*dlx)*tana ):( -hlx*tana ) );
            const real dz  ( ( ztmp - minz ) / processesO );

            const Vec3 posl( 0, 0, ztmp - ( processesO-rank )*dz );
            const Vec3 posu( 0, 0, ztmp - ( processesO-rank-1 )*dz );

            // Connecting to the top neighbor
            if( rank < processesO - 1 ) {
               connect( rank+1, HalfSpace( Vec3(0, 0, +1), posu ) );
            }
            else {
               // Connecting the top neighbor of the topmost bottom process (top-most layer of layers below outlet)
               assert( rank == processesO - 1 );

               // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
               //             |<-- point in plane                                      -->|   |<-- normal -->|
               const real cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + 0 * dlz ) * cn               );

               connect( processesO, HalfSpace( +cn, +cd ) );
            }

            // Connecting to the bottom neighbor
            if( rank > 0 ) {
               connect( rank-1, HalfSpace( Vec3(0, 0, -1), posl ) );
            }

            // Define the local domains
            if( rank > 0 && rank < processesO - 1 ) {
               defineLocalDomain( intersect(
                  HalfSpace( Vec3(0,0,+1), posl ),
                  HalfSpace( Vec3(0,0,-1), posu ) ) );
            }
            else if( rank == 0 ) {
               defineLocalDomain( HalfSpace( Vec3(0, 0, -1), posu ) );
            }
            else {
               // Define local domain of the topmost bottom process (top-most layer of layers below outlet)
               assert( rank == processesO - 1 );

               // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
               //             |<-- point in plane                                      -->|   |<-- normal -->|
               const real cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + 0 * dlz ) * cn               );

               defineLocalDomain( intersect(
                  HalfSpace( Vec3(0, 0, +1), posl ),
                  HalfSpace( -cn,            -cd   ) ) );
            }
         }
      }
      else {
         // Connect all domains in hopper
         assert( rank >= processesO );

         // Calculating the grid indices of the process
         const int  zindex  ( rank-processesO );  // z-index of the process

         // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
         //                   |<-- point in plane                                                   -->|   |<-- normal -->|
         const real south_cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + zindex * dlz )         * cn               );
         const real north_cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + ( zindex + 1 ) * dlz ) * cn               );

         // Define local domain
         defineLocalDomain( intersect(
            HalfSpace( +cn, +south_cd ),
            HalfSpace( -cn, -north_cd ) ) );

         // Connecting the top neighbor
         if( zindex < processesZ-1 ) {
            connect( rank+processesX, HalfSpace( +cn, +north_cd ) );
         }

         // Connecting the bottom neighbor
         if( zindex == 0 ) {
            // Connecting the first layer below hopper (so there are no bottom-left and bottom-right neighbors)
            connect( processesO-1, HalfSpace( -cn, -south_cd ) );
         }
         else {
            assert( zindex > 0 );

            connect( rank-processesX, HalfSpace( -cn, -south_cd ) );
         }
      }
   }
   else {
      if( rank < processesO ) {
         // Connecting the bottom processes (slice space below outlet into x-y-layers, rank 0 is bottom-most layer)
         if( processesO == 1 ) {
            // Special case: only one domain below hopper
            assert( rank == processesO - 1 );

            // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
            //             |<-- point in plane                                      -->|   |<-- normal -->|
            const real ld( Vec3T( -hlx,             0, 0 * dlz                         ) * ln               );
            const real rd( Vec3T( +hlx,             0, 0 * dlz                         ) * rn               );
            const real cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + 0 * dlz ) * cn               );

            // Define local domain of the topmost bottom process (top-most layer of layers below outlet)
            if( processesX % 2 == 0 ) {
               defineLocalDomain( merge(
                  HalfSpace( -ln, -ld ),
                  HalfSpace( -rn, -rd ) ) );
            }
            else {
               defineLocalDomain( merge(
                  HalfSpace( -ln, -ld ),
                  HalfSpace( -rn, -rd ),
                  HalfSpace( -cn, -cd ) ) );
            }

            // Connecting the top neighbor of the topmost bottom process (top-most layer of layers below outlet)
            int p=processesO;
            for( int i=0; i<hpx; ++i ) {
               const real west_d( ( Vec3T( -hlx + i * dlx,         0, 0 ) * Vec3( +1, 0, 0 ) ) );
               const real east_d( ( Vec3T( -hlx + ( i + 1 ) * dlx, 0, 0 ) * Vec3( +1, 0, 0 ) ) );

               connect( p++, intersect( HalfSpace( +ln,           +ld     ),
                                        HalfSpace( +Vec3(+1,0,0), +west_d ),
                                        HalfSpace( -Vec3(+1,0,0), -east_d ) ) );
            }

            if( processesX%2 != 0 ) {
               const int i = hpx;
               const real west_d( ( Vec3T( -hlx + i * dlx,         0, 0 ) * Vec3( +1, 0, 0 ) ) );
               const real east_d( ( Vec3T( -hlx + ( i + 1 ) * dlx, 0, 0 ) * Vec3( +1, 0, 0 ) ) );

               connect( p++, intersect( HalfSpace( +cn,           +cd ),
                                        HalfSpace( +Vec3(+1,0,0), +west_d ),
                                        HalfSpace( -Vec3(+1,0,0), -east_d ) ) );
            }

            for( int i=hpx+processesX%2; i<processesX; ++i ) {
               const real west_d( ( Vec3T( -hlx + i * dlx,         0, 0 ) * Vec3( +1, 0, 0 ) ) );
               const real east_d( ( Vec3T( -hlx + ( i + 1 ) * dlx, 0, 0 ) * Vec3( +1, 0, 0 ) ) );

               connect( p++, intersect( HalfSpace( +rn, +rd ),
                                        HalfSpace( +Vec3(+1,0,0), +west_d ),
                                        HalfSpace( -Vec3(+1,0,0), -east_d ) ) );
            }
         }
         else {
            const real ztmp( ( processesX > 1 && processesX%2 )?( -(hpx*dlx)*tana ):( -hlx*tana ) );
            const real dz  ( ( ztmp - minz ) / processesO );

            const Vec3 posl( 0, 0, ztmp - ( processesO-rank )*dz );
            const Vec3 posu( 0, 0, ztmp - ( processesO-rank-1 )*dz );

            // Connecting to the top neighbor
            if( rank < processesO - 1 ) {
               connect( rank+1, HalfSpace( Vec3(0, 0, +1), posu ) );
            }
            else {
               // Connecting the top neighbor of the topmost bottom process (top-most layer of layers below outlet)
               assert( rank == processesO - 1 );

               // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
               //             |<-- point in plane                                      -->|   |<-- normal -->|
               const real ld( Vec3T( -hlx,             0, 0 * dlz                         ) * ln               );
               const real rd( Vec3T( +hlx,             0, 0 * dlz                         ) * rn               );
               const real cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + 0 * dlz ) * cn               );

               int p=processesO;
               for( int i=0; i<hpx; ++i ) {
                  const real west_d( -( Vec3T( -hlx + i * dlx,         0, 0 ) * Vec3( +1, 0, 0 ) ) );
                  const real east_d( -( Vec3T( -hlx + ( i + 1 ) * dlx, 0, 0 ) * Vec3( +1, 0, 0 ) ) );

                  connect( p++, intersect( HalfSpace( +ln,           +ld     ),
                                           HalfSpace( +Vec3(+1,0,0), +west_d ),
                                           HalfSpace( -Vec3(+1,0,0), -east_d ) ) );
               }

               if( processesX%2 != 0 ) {
                  const int i = hpx;
                  const real west_d( -( Vec3T( -hlx + i * dlx,         0, 0 ) * Vec3( +1, 0, 0 ) ) );
                  const real east_d( -( Vec3T( -hlx + ( i + 1 ) * dlx, 0, 0 ) * Vec3( +1, 0, 0 ) ) );

                  connect( p++, intersect( HalfSpace( +cn,           +cd ),
                                           HalfSpace( +Vec3(+1,0,0), +west_d ),
                                           HalfSpace( -Vec3(+1,0,0), -east_d ) ) );
               }

               for( int i=hpx+processesX%2; i<processesX; ++i ) {
                  const real west_d( -( Vec3T( -hlx + i * dlx,         0, 0 ) * Vec3( +1, 0, 0 ) ) );
                  const real east_d( -( Vec3T( -hlx + ( i + 1 ) * dlx, 0, 0 ) * Vec3( +1, 0, 0 ) ) );

                  connect( p++, intersect( HalfSpace( +rn, +rd ),
                                           HalfSpace( +Vec3(+1,0,0), +west_d ),
                                           HalfSpace( -Vec3(+1,0,0), -east_d ) ) );
               }
            }

            // Connecting to the bottom neighbor
            if( rank > 0 ) {
               connect( rank-1, HalfSpace( Vec3(0, 0, -1), posl ) );
            }

            // Define the local domains
            if( rank > 0 && rank < processesO - 1 ) {
               defineLocalDomain( intersect(
                  HalfSpace( Vec3(0,0,+1), posl ),
                  HalfSpace( Vec3(0,0,-1), posu ) ) );
            }
            else if( rank == 0 ) {
               defineLocalDomain( HalfSpace( Vec3(0, 0, -1), posu ) );
            }
            else {
               // Define local domain of the topmost bottom process (top-most layer of layers below outlet)
               assert( rank == processesO - 1 );

               // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
               //             |<-- point in plane                                      -->|   |<-- normal -->|
               const real ld( Vec3T( -hlx,             0, 0 * dlz                         ) * ln               );
               const real rd( Vec3T( +hlx,             0, 0 * dlz                         ) * rn               );
               const real cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + 0 * dlz ) * cn               );

               if( processesX % 2 == 0 ) {
                  defineLocalDomain( merge(
                     intersect( HalfSpace( Vec3(0, 0, +1), posl ), HalfSpace( -ln, -ld ) ),
                     intersect( HalfSpace( Vec3(0, 0, +1), posl ), HalfSpace( -rn, -rd ) ) ) );
               }
               else {
                  defineLocalDomain( merge(
                     intersect( HalfSpace( Vec3(0, 0, +1), posl ), HalfSpace( -ln, -ld ) ),
                     intersect( HalfSpace( Vec3(0, 0, +1), posl ), HalfSpace( -rn, -rd ) ),
                     intersect( HalfSpace( Vec3(0, 0, +1), posl ), HalfSpace( -cn, -cd ) ) ) );
               }
            }
         }
      }
      else {
         // Connecting the processes inside the hopper
         assert( rank >= processesO );

         Vec3 n;
         real south_d, north_d;

         // Calculating the grid indices of the process
         const int  xindex  ( (rank-processesO)%processesX );  // x-index of the process
         const int  zindex  ( (rank-processesO)/processesX );  // z-index of the process

         // Calculating origin plane distances for all possible neighboring planes
         // plane equation is (x - a)^T * n = 0 thus distance of origin is a^T * n.
         const real south_ld( Vec3T( -hlx,             0, zindex * dlz                         ) * ln );
         const real south_rd( Vec3T( +hlx,             0, zindex * dlz                         ) * rn );
         const real south_cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + zindex * dlz ) * cn );

         const real north_ld( Vec3T( -hlx,             0, ( zindex + 1 ) * dlz                         ) * ln );
         const real north_rd( Vec3T( +hlx,             0, ( zindex + 1 ) * dlz                         ) * rn );
         const real north_cd( Vec3T( -hlx + hpx * dlx, 0, -tana * ( hpx * dlx ) + ( zindex + 1 ) * dlz ) * cn );

         const real west_d  ( Vec3T( -hlx + xindex * dlx,         0, 0 ) * Vec3( +1, 0, 0 ) );
         const real east_d  ( Vec3T( -hlx + ( xindex + 1 ) * dlx, 0, 0 ) * Vec3( +1, 0, 0 ) );

         // Connecting neighbors above and below, define local domain
         {
            if( xindex < hpx ) {
               n = ln;
               south_d = south_ld;
               north_d = north_ld;
            }
            else if( xindex >= hpx + processesX % 2 ) {
               n = rn;
               south_d = south_rd;
               north_d = north_rd;
            }
            else {
               n = cn;
               south_d = south_cd;
               north_d = north_cd;
            }

            // Define the local domain
            defineLocalDomain( intersect(
               HalfSpace( +n,                +south_d ),
               HalfSpace( -n,                -north_d ),
               HalfSpace( +Vec3( +1, 0, 0 ), +west_d  ),
               HalfSpace( -Vec3( +1, 0, 0 ), -east_d  ) ) );

            // Connecting the top neighbor
            if( zindex < processesZ-1 ) {
               connect( rank+processesX, intersect(
                  HalfSpace( +Vec3(+1,0,0), +west_d  ),
                  HalfSpace( -Vec3(+1,0,0), -east_d  ),
                  HalfSpace( +n,            +north_d ) ) );
            }

            // Connecting the bottom neighbor
            if( zindex == 0 ) {
               // Connecting the first layer below hopper (so there are no bottom-left and bottom-right neighbors)
               if( processesX % 2 == 0 ) {
                  connect( processesO-1, merge(
                     HalfSpace( -ln, -south_ld ),
                     HalfSpace( -rn, -south_rd ) ) );
               }
               else {
                  connect( processesO-1, merge(
                     HalfSpace( -ln, -south_ld ),
                     HalfSpace( -rn, -south_rd ),
                     HalfSpace( -cn, -south_cd ) ) );
               }
            }
            else {
               assert( zindex > 0 );

               connect( rank-processesX, intersect(
                  HalfSpace( +Vec3(+1,0,0), +west_d  ),
                  HalfSpace( -Vec3(+1,0,0), -east_d  ),
                  HalfSpace( -n,            -south_d ) ) );
            }
         }

         // Connecting neighbors to the left
         if( xindex > 0 ) {
            if( xindex - 1 < hpx ) {
               n = ln;
               south_d = south_ld;
               north_d = north_ld;
            }
            else if( xindex - 1 >= hpx + processesX % 2 ) {
               n = rn;
               south_d = south_rd;
               north_d = north_rd;
            }
            else {
               n = cn;
               south_d = south_cd;
               north_d = north_cd;
            }

            // Connecting the left neighbor
            connect( rank-1, intersect(
               HalfSpace( -Vec3(+1, 0, 0), -west_d ),
               HalfSpace( +n,              +south_d ),
               HalfSpace( -n,              -north_d ) ) );

            // Connecting the top-left neighbor
            if( zindex < processesZ-1 ) {
               connect( rank+processesX-1, intersect(
                  HalfSpace( -Vec3(+1, 0, 0), -west_d ),
                  HalfSpace( +n,              +north_d ) ) );
            }

            // Connecting the bottom-left neighbor
            if( zindex > 0 ) {
               connect( rank-processesX-1, intersect(
                  HalfSpace( -Vec3(+1, 0, 0), -west_d ),
                  HalfSpace( -n,              -south_d ) ) );
            }
         }

         // Connecting neighbors to the right
         if( xindex < processesX - 1 ) {
            if( xindex + 1 < hpx ) {
               n = ln;
               south_d = south_ld;
               north_d = north_ld;
            }
            else if( xindex + 1 >= hpx + processesX % 2 ) {
               n = rn;
               south_d = south_rd;
               north_d = north_rd;
            }
            else {
               n = cn;
               south_d = south_cd;
               north_d = north_cd;
            }

            // Connecting the right neighbor
            connect( rank+1, intersect(
               HalfSpace( +Vec3(+1, 0, 0), +east_d ),
               HalfSpace( +n,              +south_d ),
               HalfSpace( -n,              -north_d ) ) );

            // Connecting the top-right neighbor
            if( zindex < processesZ-1 ) {
               connect( rank+processesX+1, intersect(
                  HalfSpace( +Vec3(+1, 0, 0), +east_d ),
                  HalfSpace( +n,              +north_d ) ) );
            }

            // Connecting the bottom-right neighbor
            if( zindex > 0 ) {
               connect( rank-processesX+1, intersect(
                  HalfSpace( +Vec3(+1, 0, 0), +east_d ),
                  HalfSpace( -n,              -south_d ) ) );
            }
         }
      }
   }

#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
#endif


   /////////////////////////////////////////////////////
   // Setup of the objects

   const size_t nx( static_cast<size_t>( lx / space ) );
   const size_t ny( static_cast<size_t>( ly / space ) );
   const size_t nz( static_cast<size_t>( lz / space ) );
   BoxID lid( 0 );   // Handle for the outlet lid

   if( resume.empty() ) {
      /////////////////////////////////////////////////////
      // Setup of the global objects

      pe_GLOBAL_SECTION
      {
         // Creating the ground plane
         PlaneID groundPlane = createPlane( 2, 0.0, 0.0, 1.0, minz-3.0, granular );
         if( povray && colorProcesses ) pov->setTexture( groundPlane, groundTexture );

         // Creating the outlet union
         pe_CREATE_UNION( outlet, 3 )
         {
            const real sizex( 0.5*lx - 0.5*opening + width );
            const real sizez( sizex*tana );
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
            lid = createBox( 4, Vec3( 0, 0, offset3[2]+0.5*width ),
                                Vec3( offset4[0] - offset3[0], ly+width, width ), granular );

            // Fixing the global position of the outlet
            outlet->setFixed( true );

            // Setting the texture of the outlet
            if( povray && colorProcesses ) pov->setTexture( outlet, glassTexture );

            // Calculating the pivot point of the lid
            pivotPoint = left->pointFromBFtoWF ( 0.5*boxx, 0, 0.5*width );
            pivotPoint[0] = -0.5*opening;
         }
      }

      /////////////////////////////////////////////////////
      // Setup of the granular medium

      // Setup of the spheres
      const real   dx( lx / nx );
      const real   dy( ly / ny );
      const real   dz( lz / nz );

      BodyID particle;

      for( size_t i=0; i<nz; ++i ) {
         for( size_t j=0; j<ny; ++j ) {
            for( size_t k=0; k<nx; ++k )
            {
               const Vec3 gpos( -0.5*lx+0.5*dx+k*dx, -0.5*ly+0.5*dy+j*dy, width+0.5*dz+i*dz );
               const Vec3 vel ( rand<real>( -velocity, velocity ),
                                rand<real>( -velocity, velocity ),
                                rand<real>( -velocity, 0.0 ) );

               if( world->ownsPoint( gpos ) ) {
                  const size_t uid( (i/colorwidth)%2 );
                  if( spheres ) particle = createSphere( uid, gpos, radius, granular );
                  else particle = createGranularParticle( uid, gpos, radius, granular );
                  particle->setLinearVel( vel );
               }
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
      checkpointer.read( resume, povray, checkpointer_params );
      // PovRay textures are reassigned through texture policy

      UnionID outlet = static_cast<UnionID>( *findUserID( world->begin(), world->end(), 3 ) );
      lid            = static_cast<BoxID>  ( *findUserID( outlet->begin(), outlet->end(), 4 ) );

      if( povray )
         pov->writeFile( ( povray_path / "resume.pov" ).string().c_str() );
   }

   // Setting the simulation step when to start the visualization
   if( povray && t < initsteps )
      pov->setStart( initsteps - t );

   // Animating the POV-Ray camera
   if( povray && animation ) {
      const size_t start( t < initsteps+focussteps ? initsteps+focussteps-t : 0 );
      const size_t end  ( t < initsteps+focussteps+animationsteps ? initsteps+focussteps+animationsteps-t : 0 );

      if( start != end ) {
         CameraID camera = theCamera();
         theCamera()->animate( LinearShift( start, end, location1, location2 ) );
      }
   }

   // Synchronization of the MPI processes
   world->synchronize();


   /////////////////////////////////////////////////////
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "--------------------------------------------------------------\n"
                << " Process parameters:\n"
                << "   MPI processes in x-direction            = " << processesX << "\n"
                << "   MPI processes in z-direction            = " << processesZ << "\n"
                << "   MPI processes at the outlet opening     = " << processesO << "\n"
                << " Particle parameters:\n"
                << "   Total number of particles               = " << nx*ny*nz << "\n"
                << "   Type of particles                       = " << ( ( spheres )?( "spheres" ):( "granular particles" ) ) << "\n"
                << "   Radius of the particles                 = " << radius << "\n"
                << "   Spacing between the particles           = " << spacing << "\n"
                << "   Initial maximum particle velocity       = " << velocity << "\n"
                << " Environment parameters:\n"
                << "   Size of the outlet in x-dimension       = " << lx << "\n"
                << "   Size of the outlet in y-dimension       = " << ly << "\n"
                << "   Size of the outlet in z-dimension       = " << lz << "\n"
                << "   Width of the outlet walls               = " << width << "\n"
                << "   Size of the outlet door                 = " << opening << "\n"
                << "   Angle of the lower outlet walls         = " << angle << "\n"
                << " Timing parameters:\n"
                << "   Number of initialization steps          = " << initsteps << "\n"
                << "   Number of camera focus steps            = " << focussteps << "\n"
                << "   Number of camera animation steps        = " << animationsteps << "\n"
                << "   Number of opening steps                 = " << opensteps << "\n"
                << "   Number of time steps for the simulation = " << timesteps << "\n"
                << "   Size of a single step                   = " << stepsize << "\n"
                << "   POV-Ray visualization spacing           = " << visspacing << "\n"
                << " Random number generator parameters:\n"
                << "   Seed of the random number generator     = " << getSeed() << std::endl;
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
   size_t t_stop( 0 );

   totalTime.start();

   // Initialization phase
   t_stop += initsteps;
   if( !verbose && t < t_stop ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << initsteps << " initialization steps..." << std::endl;
      }
   }

   for( size_t timestep=0; t<t_stop; ++timestep, ++t )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating initialization step " <<  timestep+1 << "   " << std::flush;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   // Write out checkpoint
   if( resume.empty() ) {
      checkpointer.setPath( checkpoint_path / "settled" );
      checkpointer.write( "settled", checkpointer_params );
   }

   // Camera focus phase
   t_stop += focussteps;
   if( !verbose && t < t_stop ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << focussteps << " camera focus steps..." << std::endl;
      }
   }

   for( size_t timestep=0; t<t_stop; ++timestep, ++t )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating camera focus step " <<  timestep+1 << "   " << std::flush;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   // Camera animation phase
   t_stop += animationsteps;
   if( !verbose && t < t_stop ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << animationsteps << " camera animation steps..." << std::endl;
      }
   }

   for( size_t timestep=0; t<t_stop; ++timestep, ++t )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating camera animation step " <<  timestep+1 << "   " << std::flush;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   // Adjusting the POV-Ray spacing
   if( povray ) {
      pov->setSpacing( visspacing );
   }

   // Opening the door of the granular media outlet
   t_stop += opensteps;
   if( !verbose && t < t_stop ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << opensteps << " opening steps..." << std::endl;
      }
   }

   for( size_t timestep=0; t<t_stop; ++timestep, ++t )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating opening step " <<  timestep+1 << "   " << std::flush;
         }
      }

      lid->rotateAroundPoint( pivotPoint, Vec3( 0, M_PI/(2*opensteps), 0 ) );
      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   if( verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << "\n";
      }
   }

   /*
   if( lid ) {
      pe_LOG_INFO_SECTION( log ) {
         log << "Destroying lid.\n";
      }
      destroy( lid );
   }
   */

   // Running the granular media simulation
   t_stop += timesteps;
   if( !verbose && t < t_stop ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << timesteps << " time steps..." << std::endl;
      }
   }

   for( size_t timestep=0; t<t_stop; ++timestep, ++t )
   {
      pe_EXCLUSIVE_SECTION( 0 ) {
         if( verbose )
            std::cout << "\r Simulating time step " <<  timestep+1 << "   " << std::flush;

         for( World::Iterator body=world->begin(); body!=world->end(); ) {
            if( !body->isFixed() && body->getPosition()[2] < minz )
               body = world->destroy( body );
            else ++body;
         }
      }

      simTime.start();
      world->simulationStep( stepsize );
      simTime.end();
   }

   totalTime.end();

   pe_EXCLUSIVE_SECTION( 0 ) {
      if( verbose ) std::cout << "\n";
      std::cout << "\n"
                << " WC-Time:  Pure simulation : " << simTime.total() << "\n"
                << "           Total time      : " << totalTime.total() << "\n";
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // MPI Finalization

   checkpointer.flush();
   MPI_Finalize();
}
//*************************************************************************************************
