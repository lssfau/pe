//=================================================================================================
/*!
 *  \file MPIHourglass.cpp
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

#include <pe/engine.h>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <sstream>
#include <vector>
using namespace pe;
using namespace pe::timing;
using namespace pe::povray;




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




//=================================================================================================
//
//  POVRAY CAMERA ANIMATIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief First animation for the POV-Ray camera.
 *
 * This class represents the first animation of the POV-Ray camera for the MPIHourglass example.
 * The camera is initially located close to the sand particles to allow a close look at them
 * before slowly shifting backwards to give a complete view of the simulation scenario.
 */
class LinearShift : public CameraAnimation
{
 public:
   explicit LinearShift( size_t begin, size_t end, const Vec3& location1, const Vec3& location2,
                         const Vec3& focus1, const Vec3& focus2 )
      : begin_( begin )                                // First time step of the camera animation
      , end_  ( end   )                                // Last time step of the camera animation
      , dl_   ( (location2-location1) / (end-begin) )  // Change of the location per time step
      , df_   ( (focus2-focus1) / (end-begin) )        // Change of the focus per time step
   {}

   virtual ~LinearShift()
   {}

   virtual void updateLocation( Vec3& location )
   {
      if( TimeStep::step() > begin_ && TimeStep::step() < end_ ) {
         location += dl_;
      }
   }

   virtual void updateFocus( Vec3& focus )
   {
      if( TimeStep::step() > begin_ && TimeStep::step() < end_ ) {
         focus += df_;
      }
   }

 private:
   size_t begin_;  // First time step of the camera animation
   size_t end_;    // Last time step of the camera animation
   Vec3 dl_;       // Change of the location per time step
   Vec3 df_;       // Change of the focus per time step
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Second animation for the POV-Ray camera.
 *
 * This class represents the second animation of the POV-Ray camera for the MPIHourglass example.
 * After the initial backwards shift (see the LinearShift class description), the POV-Ray camera
 * is rotating at a fixed height around the hourglass.
 */
class CircularMotion : public CameraAnimation
{
 public:
   //**Constructor*********************************************************************************
   explicit CircularMotion( real angle )
      : q_ ( Vec3(0,0,1), angle )  // Camera rotation per time step
   {}
   //**********************************************************************************************

   //**Update functions****************************************************************************
   virtual void updateLocation( Vec3& location )
   {
      location = q_.rotate( location );
   }
   //**********************************************************************************************

 private:
   //**Member variables****************************************************************************
   Quat q_;  // Camera rotation per time step
   //**********************************************************************************************
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
/*!\brief Main function for the mpihourglass example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The mpihourglass is an hourglass where particles are flowing from the upper part into the lower
 * part.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // Simulation parameters

   // Particle parameters
   const bool   spheres ( true   );  // Switch between spheres and granular particles
   const real   radius  ( 0.2    );  // The radius of spheres of the granular media
   const real   spacing ( 0.0025 );  // Initial spacing in-between two spheres
   const real   velocity( 0.0025 );  // Initial maximum velocity of the spheres

   // Time parameters
   const size_t initsteps     (  20000 );  // Initialization steps with closed outlet door
   const size_t focussteps    (    100 );  // Number of initial close-up time steps
   const size_t animationsteps(    200 );  // Number of time steps for the camera animation
   const size_t timesteps     ( 280000 );  // Number of time steps for the flowing granular media
   const real   stepsize      (  0.001 );  // Size of a single time step

   // Process parameters
   const int    processesX( 2 );    // Number of processes in x-direction
   const int    processesY( 2 );    // Number of processes in y-direction
   const real   adaption  ( 1.5 );  // Dynamic adaption factor for the sizes of the subdomains

   // Random number generator parameters
   const size_t seed( 12345 );

   // Verbose mode
   const bool verbose( false );  // Switches the output of the simulation on and off

   // Visualization
   bool povray( true );  // Switches the POV-Ray visualization on and off

   // Visualization parameters
   const bool   colorProcesses( false );  // Switches the processes visualization on and off
   const bool   animation     (  true );  // Switches the animation of the POV-Ray camera on and off
   const bool   showInterior  ( false );  // Switches a quarter of the simulation domain on and off
   const size_t visspacing    (   200 );  // Number of time steps in-between two POV-Ray files
   const size_t colorwidth    (     9 );  // Number of particles in z-dimension with a specific color


   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Initial setups

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );
   world->setDamping( 0.85 );

   // Configuration of the MPI system
   MPISystemID mpisystem = theMPISystem();

   // Checking the size of the particles
   if( radius > real(0.3) ) {
      std::cerr << pe_RED
                << "\n Invalid radius for particles! Choose a radius of at maximum 0.3!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the total number of MPI processes
   if( processesX*processesY != mpisystem->getSize() ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Parsing the command line arguments
   for( int i=1; i<argc; ++i ) {
      if( std::strncmp( argv[i], "-no-povray", 10 ) == 0 ) {
         povray = false;
      }
   }

   // Setup of the random number generation
   setSeed( seed );

   // Creating the material for the particles
   MaterialID granular = createMaterial( "granular", 1.0, 0.04, 0.1, 0.1, 0.3, 300, 1e6, 1e5, 2e5 );

   // Fixed simulation parameters
   const size_t N    ( 30 );        // Number of boxes forming the hourglass
   const real   H    ( 9.0 );       // Height of the hourglass
   const real   L    ( 1.5 );       // Size of the center opening
   const real   W    ( 0.5 );       // Thickness of the hourglass walls
   const real   alpha( M_PI/3.9 );  // Slope of the hourglass walls (with respect to the x-axis)

   const real   beta ( M_PI*real(0.5)-alpha );                   // Slope of the hourglass walls (with respect to the z-axis)
   const real   gamma( real(2)*M_PI / N );                       // Sector angle for each box forming the hourglass
   const real   sina ( std::sin( alpha ) );                      // Sinus of the angle alpha
   const real   tana ( std::tan( alpha ) );                      // Tangens of the angle alpha
   const real   Z    ( H / sina );                               // Z-size of a single hourglass box
   const real   hZ   ( Z * real(0.5) );                          // Half Z-size of a single hourglass box
   const real   hL   ( L * real(0.5) );                          // Half the opening size
   const real   hW   ( W * real(0.5) );                          // Half the thickness of an hourglass wall
   const real   R    ( H / tana + hL );                          // Radius of the hourglass
   const real   Y    ( real(2)*R*std::tan( gamma*real(0.5) ) );  // Y-Size of a single hourglass box
   const real   space( real(2)*radius+spacing );                 // Space initially required by a single particle


   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;
   CameraID camera;

   if( povray )
   {
      // Camera settings
      const Vec3 location1( ( showInterior )?( Vec3(2.47487,-2.47487,0.5) ):( Vec3(0,-3.5,0.5) ) );
      const Vec3 location2( ( showInterior )?( Vec3(16.9706,-16.9706,5) ):( Vec3(0,-24,5 ) ) );
      const Vec3 focus1   ( 0,   0  ,  1.5 );
      const Vec3 focus2   ( 0,   0  , -1   );
      const size_t start  ( initsteps+focussteps );
      const size_t end    ( start+animationsteps );

      // Configuration of the POV-Ray writer
      pov = activateWriter();
      pov->setStart( initsteps );
      pov->setSpacing( 1 );
      pov->setFilename( "video/pic%.pov" );
      pov->include( "settings.inc" );
      pov->setBackground( Color( 0, 0, 0 ) );

      // Configuration of the scene lighting
      pov->addLightSource(
         ParallelLight(
            Vec3( 50, 0, 10 ),       // Global location of the light source
            Color( 1.0, 1.0, 1.0 ),  // Color of the light source
            FadeDistance( 160 ),     // Fade distance setting of the POV-Ray light source
            FadePower( 1 )           // Fade power setting of the POV-Ray light source
         )
      );
      pov->addLightSource(
         ParallelLight(
            Vec3( -50, 0, 10 ),      // Global location of the light source
            Color( 1.0, 1.0, 1.0 ),  // Color of the light source
            FadeDistance( 160 ),     // Fade distance setting of the POV-Ray light source
            FadePower( 1 )           // Fade power setting of the POV-Ray light source
         )
      );
      pov->addLightSource(
         ParallelLight(
            Vec3( 0, 50, 10 ),       // Global location of the light source
            Color( 1.0, 1.0, 1.0 ),  // Color of the light source
            FadeDistance( 160 ),     // Fade distance setting of the POV-Ray light source
            FadePower( 1 )           // Fade power setting of the POV-Ray light source
         )
      );
      pov->addLightSource(
         ParallelLight(
            Vec3( 0, -50, 10 ),      // Global location of the light source
            Color( 1.0, 1.0, 1.0 ),  // Color of the light source
            FadeDistance( 160 ),     // Fade distance setting of the POV-Ray light source
            FadePower( 1 )           // Fade power setting of the POV-Ray light source
         )
      );

      // Configuration of the POV-Ray camera
      camera = theCamera();
      camera->setLocation( ( animation )?( location1 ):( location2 ) );
      camera->setFocus   ( ( animation )?( focus1    ):( focus2    ) );

      // Animating the POV-Ray camera
      if( animation )
         camera->animate( LinearShift( start, end, location1, location2, focus1, focus2 ) );

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
   // Setup of the MPI processes: 2D Rectilinear Domain Decomposition

   std::vector<real> sizeX( processesX, 1 );
   std::vector<real> sizeY( processesY, 1 );

   // Calculating the smallest process expansion in x-direction
   for( int i=(processesX-1)/2-1; i>=0; --i ) {
      sizeX[i] = sizeX[i+1] * adaption;
   }
   for( int i=processesX/2+1; i<processesX; ++i ) {
      sizeX[i] = sizeX[i-1] * adaption;
   }

   const real dx( ( 2*R ) / std::accumulate( sizeX.begin(), sizeX.end(), real(0) ) );

   for( int i=0; i<processesX; ++i )
      sizeX[i] *= dx;

   // Calculating the smallest process expansion in y-direction
   for( int i=(processesY-1)/2-1; i>=0; --i ) {
      sizeY[i] = sizeY[i+1] * adaption;
   }
   for( int i=processesY/2+1; i<processesY; ++i ) {
      sizeY[i] = sizeY[i-1] * adaption;
   }

   const real dy( ( 2*R ) / std::accumulate( sizeY.begin(), sizeY.end(), real(0) ) );

   for( int i=0; i<processesY; ++i )
      sizeY[i] *= dy;

   // Calculating the processes dimensions
   std::vector<real> dispX( processesX+1, -R );
   std::vector<real> dispY( processesY+1, -R );

   for( int i=1; i<=processesX; ++i )
      dispX[i] = dispX[i-1] + sizeX[i-1];

   for( int i=1; i<=processesY; ++i )
      dispY[i] = dispY[i-1] + sizeY[i-1];

   // Computing the Cartesian coordinates of the neighboring processes
   int dims   [] = { processesX, processesY };
   int periods[] = { false     , false      };

   int rank;           // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 2, center );

   int west     [] = { center[0]-1, center[1]   };
   int east     [] = { center[0]+1, center[1]   };
   int south    [] = { center[0]  , center[1]-1 };
   int north    [] = { center[0]  , center[1]+1 };
   int southwest[] = { center[0]-1, center[1]-1 };
   int southeast[] = { center[0]+1, center[1]-1 };
   int northwest[] = { center[0]-1, center[1]+1 };
   int northeast[] = { center[0]+1, center[1]+1 };

   // Specify local subdomain
   defineLocalDomain( intersect(
    HalfSpace( Vec3( +1, 0, 0 ), +dispX[center[0]  ] ),
    HalfSpace( Vec3( -1, 0, 0 ), -dispX[center[0]+1] ),
    HalfSpace( Vec3( 0, +1, 0 ), +dispY[center[1]  ] ),
    HalfSpace( Vec3( 0, -1, 0 ), -dispY[center[1]+1] ) ) );

   // Connecting the west neighbor
   if( west[0] >= 0 ) {
      MPI_Cart_rank( cartcomm, west, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(-1,0,0), -dispX[center[0]] ),
         HalfSpace( Vec3(0,+1,0), +dispY[center[1]] ),
         HalfSpace( Vec3(0,-1,0), -dispY[center[1]+1] ) ) );
   }

   // Connecting the east neighbor
   if( east[0] < processesX ) {
      MPI_Cart_rank( cartcomm, east, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(+1,0,0), +dispX[center[0]+1] ),
         HalfSpace( Vec3(0,+1,0), +dispY[center[1]] ),
         HalfSpace( Vec3(0,-1,0), -dispY[center[1]+1] ) ) );
   }

   // Connecting the south neighbor
   if( south[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, south, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,-1,0), -dispY[center[1]] ),
         HalfSpace( Vec3(+1,0,0), +dispX[center[0]] ),
         HalfSpace( Vec3(-1,0,0), -dispX[center[0]+1] ) ) );
   }

   // Connecting the north neighbor
   if( north[1] < processesY ) {
      MPI_Cart_rank( cartcomm, north, &rank );
      connect( rank, intersect(
         HalfSpace( Vec3(0,+1,0), +dispY[center[1]+1] ),
         HalfSpace( Vec3(+1,0,0), +dispX[center[0]] ),
         HalfSpace( Vec3(-1,0,0), -dispX[center[0]+1] ) ) );
   }

   // Connecting the south-west neighbor
   if( southwest[0] >= 0 && southwest[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -dispX[center[0]] ),
                                HalfSpace( Vec3(0,-1,0), -dispY[center[1]] ) ) );
   }

   // Connecting the south-east neighbor
   if( southeast[0] < processesX && southeast[1] >= 0 ) {
      MPI_Cart_rank( cartcomm, southeast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1, 0,0), dispX[center[0]+1] ),
                                HalfSpace( Vec3(0,-1,0), -dispY[center[1]] ) ) );
   }

   // Connecting the north-west neighbor
   if( northwest[0] >= 0 && northwest[1] < processesY ) {
      MPI_Cart_rank( cartcomm, northwest, &rank );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -dispX[center[0]] ),
                                HalfSpace( Vec3( 0,1,0), dispY[center[1]+1] ) ) );
   }

   // Connecting the north-east neighbor
   if( northeast[0] < processesX && northeast[1] < processesY ) {
      MPI_Cart_rank( cartcomm, northeast, &rank );
      connect( rank, intersect( HalfSpace( Vec3(1,0,0), dispX[center[0]+1] ),
                                HalfSpace( Vec3(0,1,0), dispY[center[1]+1] ) ) );
   }

#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
#endif


   /////////////////////////////////////////////////////
   // Setup of the global objects

   BoxID lid( 0 );

   pe_GLOBAL_SECTION
   {
      // Creating the ground plane
      createPlane( 2, 0.0, 0.0, 1.0, -H, granular, false );

      // Creating the upper part of the hourglass
      for( size_t i=0; i<N; ++ i )
      {
         BoxID box = createBox( 3, Vec3( -hW, 0.0,  hZ ), W, Y, Z, granular, false );
         box->rotateAroundPoint( Vec3( 0, 0, 0 ), Vec3( 0, -beta, 0 ) );
         box->translate( -hL, 0.0, 0.0 );
         box->rotateAroundOrigin( 0, 0, i*gamma );
         box->setFixed( true );
      }

      // Creating the lower part of the hourglass
      for( size_t i=0; i<N; ++ i )
      {
         BoxID box = createBox( 3, Vec3( -hW, 0.0, -hZ ), W, Y, Z, granular, false );
         box->rotateAroundPoint( Vec3( 0, 0, 0 ), Vec3( 0, beta, 0 ) );
         box->translate( -hL, 0.0, 0.0 );
         box->rotateAroundOrigin( 0, 0, i*gamma );
         box->setFixed( true );
      }

      // Creating the lid
      lid = createBox( 4, Vec3( 0, 0, -hW ), L, L, W, granular, false );
      lid->setFixed( true );
   }


   /////////////////////////////////////////////////////
   // Setup of the granular medium

   const size_t n( std::floor( H / space ) );
   unsigned long particles( 0 ), primitives( 0 );

   for( size_t i=0; i<n; ++i )
   {
      const real   h  ( i*space + real(0.5)*space );
      const real   r  ( h / tana + real(0.5)*L - radius / sina );
      const size_t m  ( std::floor( 2*r / space ) );
      const size_t uid( (i/colorwidth)%2 );

      for( size_t j=0; j<m; ++j ) {
         for( size_t k=0; k<m; ++k )
         {
            const Vec3 gpos( ( k - real(0.5)*(m-1) )*space, ( j - real(0.5)*(m-1) )*space, h );
            const Vec3 vel ( rand<real>( -velocity, velocity ),
                             rand<real>( -velocity, velocity ),
                             rand<real>( -velocity, 0.0 ) );

            const bool visible( ( !showInterior || gpos[0]<0 || gpos[1]<0 )?(true):(false) );

            if( sq( gpos[0] ) + sq( gpos[1] ) < r*r && world->ownsPoint( gpos ) )
            {
               if( spheres ) {
                  SphereID particle = createSphere( uid, gpos, radius, granular, visible );
                  ++particles;
                  particle->setLinearVel( vel );
               }
               else {
                  UnionID particle = createGranularParticle( uid, gpos, radius, granular );
                  ++particles;
                  primitives += particle->size();
                  particle->setLinearVel( vel );
               }
            }
         }
      }
   }

   // Synchronization of the MPI processes
   world->synchronize();

   // Calculating the total number of particles and primitives
   unsigned long particlesTotal ( 0 );
   unsigned long primitivesTotal( 0 );

   MPI_Reduce( &particles, &particlesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );

   if( !spheres )
      MPI_Reduce( &primitives, &primitivesTotal, 1, MPI_UNSIGNED_LONG, MPI_SUM, 0, cartcomm );


   /////////////////////////////////////////////////////
   // Output of the simulation settings

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "SIMULATION SETUP" << pe_OLDCOLOR
                << "--------------------------------------------------------------\n"
                << " Total number of particles               = " << particlesTotal << "\n";

      if( !spheres )
         std::cout << " Total number of primitives              = " << primitivesTotal << "\n";

      std::cout << " Number of initialization steps          = " << initsteps << "\n"
                << " Number of time steps for the simulation = " << timesteps << "\n"
                << " Seed of the random number generator     = " << getSeed() << "\n"
                << "--------------------------------------------------------------------------------\n" << std::endl;
   }

   // Visualizing the initial setup
   if( povray )
      pov->writeFile( "video/init.pov" );


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
   if( !verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << initsteps << " initialization steps..." << std::endl;
      }
   }

   for( size_t timestep=0; timestep<initsteps; ++timestep )
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

   // Camera focus and animation phase
   if( animation )
   {
      // Camera focus phase
      if( !verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << " Simulating " << focussteps << " camera focus steps..." << std::endl;
         }
      }

      for( size_t timestep=0; timestep<focussteps; ++timestep )
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
      if( !verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << " Simulating " << animationsteps << " camera animation steps..." << std::endl;
         }
      }

      for( size_t timestep=0; timestep<animationsteps; ++timestep )
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
   }

   // Removing the lid
   destroy( lid );

   // Adjusting the POV-Ray visualization
   if( povray )
   {
      // Adjusting the spacing between two POV-Ray visualizations
      pov->setSpacing( visspacing );

      // Adjusting the camera animation
      if( animation && !showInterior ) {
         camera->animate( CircularMotion( 2*M_PI/real(timesteps) ) );
      }
      else camera->removeAnimation();
   }

   // Running the granular media simulation
   if( !verbose ) {
      pe_EXCLUSIVE_SECTION( 0 ) {
         std::cout << " Simulating " << timesteps << " time steps..." << std::endl;
      }
   }

   for( size_t timestep=0; timestep<timesteps; ++timestep )
   {
      if( verbose ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "\r Simulating time step " <<  timestep+1 << "   " << std::flush;
         }
      }
      else if( timestep%10000 == 0 ) {
         pe_EXCLUSIVE_SECTION( 0 ) {
            std::cout << "   Simulating time steps " << timestep+1 << " to " << ( (timestep+10000>timesteps)?(timesteps):(timestep+10000) ) << std::endl;
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

   MPI_Finalize();
}
//*************************************************************************************************
