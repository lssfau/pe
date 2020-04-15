//=================================================================================================
/*!
 *  \file MPILabyrinth.cpp
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
#include <sstream>
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
//  POVRAY CAMERA ANIMATION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Animation for the POV-Ray camera.
 *
 * This class animates the POV-Ray camera for the MPILabyrinth example. The camera is initially
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
class LabyrinthTexturePolicy : public TexturePolicy
{
 public:
   explicit LabyrinthTexturePolicy()
   {}

   virtual ~LabyrinthTexturePolicy()
   {}

   virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
   {
      switch( body->getID() )
      {
         case 0:  return CustomTexture( "GranularTexture1" );
         case 1:  return CustomTexture( "GranularTexture2" );
         default: return CustomTexture( "WallTexture"      );
      }
   }

   using TexturePolicy::getTexture;
};
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Overlap test of a single rigid body with a range of rigid bodies
 *
 * \param begin Iterator to the beginning of the body range.
 * \param end Iterator one past the end of the body range.
 * \param body The rigid body to be tested with the rigid bodies in the range \a begin to \a end.
 * \return \a true in case an overlap is detected, \a false otherwise.
 */
template< typename IteratorType >
inline bool overlap( IteratorType begin, IteratorType end, ConstBodyID body )
{
   for( ; begin!=end; ++begin )
      if( *begin != body && overlap( *begin, body ) )
         return true;
   return false;
}
//*************************************************************************************************




//=================================================================================================
//
//  MAIN FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Main function for the mpilabyrinth example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The miplabyrinth example is a pseudo-3D setup, which is periodically in x- and z-direction.
 * The domain is filled with extruded, isosceles, right-angled triangles. The prisms are arranged
 * such that the right angles of the prisms in the row below are in the middle of the gaps above.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // Simulation parameters

   // Particle parameters
   const bool   spheres ( true   );  // Switch between spheres and granular particles
   const real   radius  ( 0.24  );  // The radius of spheres of the granular media
   const real   spacing ( 0.001  );  // Initial spacing in-between two spheres
   const real   velocity( 0.0025 );  // Initial maximum velocity of the spheres

   // Time parameters
   const size_t initsteps     (  2000 );  // Initialization steps with closed outlet door
   const size_t timesteps     ( 80000 );  // Number of time steps for the flowing granular media
   const real   stepsize      ( 0.005 );  // Size of a single time step

   // Process parameters
   const int processesX( 3 );  // Number of processes in x-direction
   const int processesZ( 3 );  // Number of processes in z-direction

   // Random number generator parameters
   const size_t seed( 12345 );

   // Verbose mode
   const bool verbose( false );  // Switches the output of the simulation on and off

   // Visualization
   bool povray( true );  // Switches the POV-Ray visualization on and off

   // Visualization parameters
   const bool   colorProcesses( false );  // Switches the processes visualization on and off
   const bool   animation     (  true );  // Switches the animation of the POV-Ray camera on and off
   const size_t visspacing    (   164 );  // Number of time steps in-between two POV-Ray files
   const size_t colorwidth    (    51 );  // Number of particles in x-dimension with a specific color


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
   if( processesX < 3 || processesZ < 3 || processesX*processesZ != mpisystem->getSize() ) {
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
   const real lx( 25 );                         // Size of the domain in x-direction
   const real ly(  1 );                         // Size of the domain in y-direction
   const real lz( 48 );                         // Size of the domain in z-direction
   const real archx( std::sqrt(real(2))*2 );    // Size of a single arch-box in x-direction
   const real archy( 1.5*ly );                  // Size of a single arch-box in y-direction
   const real archz( 0.2 );                     // Size of a single arch-box in z-direction
   const real space( real(2)*radius+spacing );  // Space initially required by a single particle


   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;
   CameraID camera;

   if( povray )
   {
      // Camera settings
      const real start    ( 0 );
      const real end      ( 0.25*timesteps );
      const Vec3 location1( 12.5,  -2.0, 24.0 );
      const Vec3 location2( 12.5, -15.0, 24.0 );
      const Vec3 focus    ( 12.5,   0.0, 24.0 );

      // Configuration of the POV-Ray writer
      pov = activateWriter();
      pov->setStart   ( initsteps  );
      pov->setSpacing ( visspacing );
      pov->setFilename( "video/pic%.pov" );
      pov->include    ( "settings.inc" );
      pov->setBackground( Color( 0, 0, 0 ) );

      // Configuration of the scene lighting
      pov->addLightSource(
         ParallelLight(
            location2,              // Global location of the light source
            Color( 0.5, 0.5, 0.5 )  // Color of the light source
         )
      );

      pov->addLightSource(
         PointLight(
            Vec3( 0, -15, 24 ),     // Global location of the light source
            Color( 0.5, 0.5, 0.5 )  // Color of the light source
         )
      );

      pov->addLightSource(
         PointLight(
            Vec3( 25, -15, 24 ),    // Global location of the light source
            Color( 0.5, 0.5, 0.5 )  // Color of the light source
         )
      );

      pov->addLightSource(
         PointLight(
            Vec3( 12.5, -15, 0 ),   // Global location of the light source
            Color( 0.5, 0.5, 0.5 )  // Color of the light source
         )
      );

      pov->addLightSource(
         PointLight(
            Vec3( 12.5, -15, 48 ),  // Global location of the light source
            Color( 0.5, 0.5, 0.5 )  // Color of the light source
         )
      );

      // Configuration of the POV-Ray camera
      camera = theCamera();
      camera->setLocation( ( animation )?( location1 ):( location2 ) );
      camera->setFocus   ( focus );

      // Animating the POV-Ray camera
      if( animation )
         camera->animate( LinearShift( start, end, location1, location2 ) );

      // Configuration of the texture policy
      if( colorProcesses ) {
         std::ostringstream texture;
         texture << "Texture" << theMPISystem()->getRank()%13;
         pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
      }
      else {
         pov->setTexturePolicy( LabyrinthTexturePolicy() );
      }
   }


   /////////////////////////////////////////////////////
   // Setup of the MPI processes: Periodic 2D Regular Domain Decomposition

   const real lpx( lx / processesX );  // Size of a process subdomain in x-direction
   const real lpz( lz / processesZ );  // Size of a process subdomain in z-direction

   int dims   [] = { processesX, processesZ };
   int periods[] = { true      , true       };

   int rank;           // Rank of the neighboring process
   int center[2];      // Definition of the coordinates array 'center'
   MPI_Comm cartcomm;  // The new MPI communicator with Cartesian topology

   MPI_Cart_create( MPI_COMM_WORLD, 2, dims, periods, false, &cartcomm );
   mpisystem->setComm( cartcomm );
   MPI_Cart_coords( cartcomm, mpisystem->getRank(), 2, center );

   int west      [] = { center[0]-1, center[1]   };
   int east      [] = { center[0]+1, center[1]   };
   int bottom    [] = { center[0]  , center[1]-1 };
   int top       [] = { center[0]  , center[1]+1 };
   int bottomwest[] = { center[0]-1, center[1]-1 };
   int bottomeast[] = { center[0]+1, center[1]-1 };
   int topwest   [] = { center[0]-1, center[1]+1 };
   int topeast   [] = { center[0]+1, center[1]+1 };

   // Specify local subdomain (Since the domain is periodic we do not have to remove intersections at the border)
   defineLocalDomain( intersect(
      HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
      HalfSpace( Vec3(-1,0,0), -east[0]*lpx ),
      HalfSpace( Vec3(0,0,+1), +center[1]*lpz ),
      HalfSpace( Vec3(0,0,-1), -top[1]*lpz ) ) );

   // Connecting the west neighbor
   {
      MPI_Cart_rank( cartcomm, west, &rank );
      const Vec3 offset( ( ( west[0]<0 )?( 25 ):( 0 ) ), 0, 0 );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*lpx ),
                                HalfSpace( Vec3(0,0,+1), +center[1]*lpz ),
                                HalfSpace( Vec3(0,0,-1), -top[1]*lpz ) ), offset );
   }

   // Connecting the east neighbor
   {
      MPI_Cart_rank( cartcomm, east, &rank );
      const Vec3 offset( ( ( east[0]==processesX )?( -25 ):( 0 ) ), 0, 0 );
      connect( rank, intersect( HalfSpace( Vec3(+1,0,0), +east[0]*lpx ),
                                HalfSpace( Vec3(0,0,+1), +center[1]*lpz ),
                                HalfSpace( Vec3(0,0,-1), -top[1]*lpz ) ), offset );
   }

   // Connecting the bottom neighbor
   {
      MPI_Cart_rank( cartcomm, bottom, &rank );
      const Vec3 offset( 0, 0, ( ( bottom[1]<0 )?( 48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(0,0,-1), -center[1]*lpz ),
                                HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
                                HalfSpace( Vec3(-1,0,0), -east[0]*lpx ) ), offset );
   }

   // Connecting the top neighbor
   {
      MPI_Cart_rank( cartcomm, top, &rank );
      const Vec3 offset( 0, 0, ( ( top[1]==processesZ )?( -48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(0,0,+1), +top[1]*lpz ),
                                HalfSpace( Vec3(+1,0,0), +center[0]*lpx ),
                                HalfSpace( Vec3(-1,0,0), -east[0]*lpx ) ), offset );
   }

   // Connecting the bottom-west neighbor
   {
      MPI_Cart_rank( cartcomm, bottomwest, &rank );
      const Vec3 offset( ( ( west[0]<0 )?( 25 ):( 0 ) ), 0, ( ( bottom[1]<0 )?( 48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*lpx ),
                                HalfSpace( Vec3(0,0,-1), -center[1]*lpz ) ), offset );
   }

   // Connecting the bottom-east neighbor
   {
      MPI_Cart_rank( cartcomm, bottomeast, &rank );
      const Vec3 offset( ( ( east[0]==processesX )?( -25 ):( 0 ) ), 0, ( ( bottom[1]<0 )?( 48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(+1,0,0), +east[0]*lpx ),
                                HalfSpace( Vec3(0,0,-1), -center[1]*lpz ) ), offset );
   }

   // Connecting the top-west neighbor
   {
      MPI_Cart_rank( cartcomm, topwest, &rank );
      const Vec3 offset( ( ( west[0]<0 )?( 25 ):( 0 ) ), 0, ( ( top[1]==processesZ )?( -48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(-1,0,0), -center[0]*lpx ),
                                HalfSpace( Vec3(0,0,+1), +top[1]*lpz ) ), offset );
   }

   // Connecting the top-east neighbor
   {
      MPI_Cart_rank( cartcomm, topeast, &rank );
      const Vec3 offset( ( ( east[0]==processesX )?( -25 ):( 0 ) ), 0, ( ( top[1]==processesZ )?( -48 ):( 0 ) ) );
      connect( rank, intersect( HalfSpace( Vec3(+1,0,0), +east[0]*lpx ),
                                HalfSpace( Vec3(0,0,+1), +top[1]*lpz ) ), offset );
   }

#ifndef NDEBUG
   //Checking the process setup
   theMPISystem()->checkProcesses();
#endif


   /////////////////////////////////////////////////////
   // Setup of the global objects

   BoxVector boxes;

   pe_GLOBAL_SECTION
   {
      Vec3 gpos( 2.5, 0.5*ly, 47 );

      // Creating the front and back plane
      createPlane( 2, 0,  1, 0,   0, granular, false );
      createPlane( 2, 0, -1, 0, -ly, granular, false );

      // Creating the archs
      for( int i=0; i<8; ++i )
      {
         for( int j=0; j<5; ++j ) {
            BoxID left = createBox( 2, 0, 0, 0, archx, archy, archz, granular, false );
            left->rotate( 0, -M_PI/4, 0 );
            left->translate( gpos - left->pointFromBFtoWF( 0.5*archx, 0, 0.5*archz ) );
            left->setFixed( true );
            boxes.pushBack( left );

            BoxID right = createBox( 2, 0, 0, 0, archx, archy, archz, granular, false );
            right->rotate( 0, M_PI/4, 0 );
            right->translate( gpos - right->pointFromBFtoWF( -0.5*archx, 0, 0.5*archz ) );
            right->setFixed( true );
            boxes.pushBack( right );

            gpos[0] += 5;
         }

         gpos[0]  = 2.5;
         gpos[2] -= 6.0;
      }

      gpos = Vec3( 0, 0.5*ly, 44 );

      for( int i=0; i<8; ++i )
      {
         for( int j=0; j<6; ++j ) {
            BoxID left = createBox( 2, 0, 0, 0, archx, archy, archz, granular, false );
            left->rotate( 0, -M_PI/4, 0 );
            left->translate( gpos - left->pointFromBFtoWF( 0.5*archx, 0, 0.5*archz ) );
            left->setFixed( true );
            boxes.pushBack( left );

            BoxID right = createBox( 2, 0, 0, 0, archx, archy, archz, granular, false );
            right->rotate( 0, M_PI/4, 0 );
            right->translate( gpos - right->pointFromBFtoWF( -0.5*archx, 0, 0.5*archz ) );
            right->setFixed( true );
            boxes.pushBack( right );

            gpos[0] += 5;
         }

         gpos[0]  = 0;
         gpos[2] -= 6.0;
      }
   }


   /////////////////////////////////////////////////////
   // Setup of the granular medium

   const size_t nx( static_cast<size_t>( lx / space ) );
   const size_t ny( static_cast<size_t>( ly / space ) );
   const size_t nz( static_cast<size_t>( lz / space ) );
   const real   dx( lx / nx );
   const real   dy( ly / ny );
   const real   dz( lz / nz );

   unsigned long particles( 0 ), primitives( 0 );

   for( size_t i=0; i<nz; ++i ) {
      for( size_t j=0; j<ny; ++j ) {
         for( size_t k=0; k<nx; ++k )
         {
            const size_t uid ( (k/colorwidth)%2 );
            const Vec3   gpos( 0.5*dx+k*dx, 0.5*dy+j*dy, 0.5*dz+i*dz );
            const Vec3   vel ( rand<real>( -velocity, velocity ),
                               rand<real>( -velocity, velocity ),
                               rand<real>( -velocity, 0.0 ) );

            if( world->ownsPoint( gpos ) )
            {
               if( spheres )
               {
                  SphereID particle = createSphere( uid, gpos, radius, granular );

                  if( overlap( boxes.begin(), boxes.end(), particle ) ) {
                     destroy( particle );
                  }
                  else {
                     ++particles;
                     particle->setLinearVel( vel );
                  }
               }
               else
               {
                  UnionID particle = createGranularParticle( uid, gpos, radius, granular );

                  if( overlap( boxes.begin(), boxes.end(), particle ) ) {
                     destroy( particle );
                  }
                  else {
                     ++particles;
                     primitives += particle->size();
                     particle->setLinearVel( vel );
                  }
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
