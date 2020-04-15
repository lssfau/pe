//=================================================================================================
/*!
 *  \file MPIStair.cpp
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
#include <pe/support.h>
#include <cmath>
#include <iostream>
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
 * This class animates the POV-Ray camera for the MPIStair example. The camera is initially
 * located close to the floor in front of the stair before slowly shifting forward and upwards
 * to give a complete view of the simulation scenario.
 */
class LinearShift : public CameraAnimation
{
 public:
   explicit LinearShift( size_t first, size_t last,
                         const Vec3& location1, const Vec3& location2,
                         const Vec3& focus1, const Vec3& focus2 )
      : first_    ( first     )  // First time step of the camera animation
      , last_     ( last      )  // Last time step of the camera animation
      , location1_( location1 )  // Initial location of the camera
      , location2_( location2 )  // Final location of the camera
      , focus1_   ( focus1    )  // Initial focus point of the camera
      , focus2_   ( focus2    )  // Final focus point of the camera
   {}

   virtual ~LinearShift()
   {}

   virtual void updateLocation( Vec3& location )
   {
      if( TimeStep::step() > first_ && TimeStep::step() < last_ ) {
         const Vec3 disp( ( location1_ - location2_ ) / ( last_ - first_ ) );
         location -= disp;
      }
   }

   virtual void updateFocus( Vec3& focus )
   {
      if( TimeStep::step() > first_ && TimeStep::step() < last_ ) {
         const Vec3 disp( ( focus1_ - focus2_ ) / ( last_ - first_ ) );
         focus -= disp;
      }
   }

 private:
   size_t first_;    // First time step of the camera animation
   size_t last_;     // Last time step of the camera animation
   Vec3 location1_;  // Initial location of the camera
   Vec3 location2_;  // Final location point of the camera
   Vec3 focus1_;     // Initial focus point of the camera
   Vec3 focus2_;     // Final focus point of the camera
};
//*************************************************************************************************




//=================================================================================================
//
//  POVRAY TEXTURE POLICY
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Texture policy for the mpistair example.
 *
 * This class represents the POV-Ray texture policy for the mpistair example.
 */
class ParticleTexturePolicy : public TexturePolicy
{
 public:
   explicit ParticleTexturePolicy()
   {}

   virtual ~ParticleTexturePolicy()
   {}

   virtual const pe::povray::Texture getTexture( ConstBodyID body ) const
   {
      switch( body->getID() )
      {
         case 0 : return CustomTexture( "Texture0"  );
         case 1 : return CustomTexture( "Texture1"  );
         case 2 : return CustomTexture( "Texture2"  );
         case 3 : return CustomTexture( "Texture3"  );
         case 4 : return CustomTexture( "Texture4"  );
         case 5 : return CustomTexture( "Texture5"  );
         case 6 : return CustomTexture( "Texture6"  );
         case 7 : return CustomTexture( "Texture7"  );
         case 8 : return CustomTexture( "Texture8"  );
         case 9 : return CustomTexture( "Texture9"  );
         case 10: return CustomTexture( "Texture10" );
         case 11: return CustomTexture( "Texture11" );
         case 12: return CustomTexture( "Texture12" );
         default: return Texture();
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
/*!\brief Main function for the mpistair example.
 *
 * \param argc Number of command line arguments.
 * \param argv Array of command line arguments.
 * \return Success of the simulation.
 *
 * The mpistair example is a spiral staircase where particles are poured down the stairs.
 */
int main( int argc, char** argv )
{
   /////////////////////////////////////////////////////
   // MPI Initialization

   MPI_Init( &argc, &argv );


   /////////////////////////////////////////////////////
   // Simulation parameters

   // Stair parameters
   const real w    ( 30.0 );  // Width of a single stair; x-expansion
   const real d    (  5.0 );  // Depth of a single stair; y-expansion
   const real h    (  1.0 );  // Height of a single stair; z-expansion
   const real wrail(  1.0 );  // Width of the handrail
   const real hrail(  3.0 );  // Height of the handrail

   // Time parameters
   const size_t timesteps( 12000 );  // Total number of time steps
   const real   stepsize (  0.01 );  // Size of a single time step

   // Parameters of the spherical particles
   const size_t nx( 20 );  // Number of particle generators in x-direction
   const size_t nz(  3 );  // Number of particle generators in z-direction

   // Process parameters
   const int processes1( 6 );  // Total number of processes around the stair
   const int processes2( 2 );  // Total number of processes at the bottom of the stair

   // Random number generator parameters
   const size_t seed( 12345 );

   // Verbose mode
   const bool verbose( true );  // Switches the output of the simulation on and off

   // Visualization
   bool povray( true );  // Switches the POV-Ray visualization on and off

   // Visualization parameters
   const bool   colorProcesses( false );  // Switches the processes visualization on and off
   const bool   animation     (  true );  // Switches the animation of the POV-Ray camera on and off
   const size_t visspacing    (    10 );  // Number of time steps inbetween two POV-Ray files


   /////////////////////////////////////////////////////
   // Initial setups

   const int  processes( theMPISystem()->getSize() );  // The total number of active MPI processes
   const int  rank     ( theMPISystem()->getRank() );  // The rank of the MPI process
   const real space    ( ( real(0.65)*w ) / nx );      // Total space for a single particle
   const real radius   ( real(0.4)*space );            // Radius of a particle
   const real alpha    ( M_PI/processes1 );            // Radius spanned by a stair process
   const real cradius  ( d + 1.0 );                    // Radius of the center capsule

   // Checking the size of the particles
   if( real(2)*cradius*std::sin( alpha*real(0.5) ) < radius ) {
      std::cerr << pe_RED
                << "\n Particles are too large for chosen number of processes!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Checking the total number of MPI processes
   if( processes1 < 1 || processes2 < 1 || processes1+processes2 != processes ) {
      std::cerr << pe_RED
                << "\n Invalid number of MPI processes!\n\n"
                << pe_OLDCOLOR;
      return EXIT_FAILURE;
   }

   // Setup of the random number generation
   setSeed( seed );

   // Parsing the command line arguments
   CommandLineInterface& cli = CommandLineInterface::getInstance();
   cli.parse( argc, argv );
   cli.evaluateOptions();
   variables_map& vm = cli.getVariablesMap();
   if( vm.count( "no-povray" ) > 0 )
      povray = false;

   // Creating the material for the particles
   MaterialID granular = createMaterial( "granular", 1.0, 0.25, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5 );

   // Configuration of the simulation world
   WorldID world = theWorld();
   world->setGravity( 0.0, 0.0, -2.0 );


   /////////////////////////////////////////////////////
   // Setup of the POV-Ray visualization

   WriterID pov;

   const Color lightgray( 0.8, 0.8, 0.8 );
   const CustomTexture groundTexture( "GroundTexture" );
   const CustomTexture wallTexture  ( "WallTexture"   );
   const CustomTexture stoneTexture ( "T_Stone8 scale 10" );
   const PlainTexture  redTexture   ( ColorPigment( 1.0, 0.0, 0.0 ) );
   const CustomTexture goldTexture  ( "T_Gold_1A" );

   if( povray )
   {
      // Configuration of the POV-Ray writer
      pov = activateWriter();
      pov->setSpacing( visspacing );
      pov->setFilename( "video/pic%.pov" );
      pov->include( "settings.inc" );
      pov->setBackground( Color( 0, 0, 0 ) );

      // Configuration of the scene lighting
      pov->addLightSource( PointLight   ( Vec3(  0.5*w, 0.0,  20.0 ), lightgray ) );
      pov->addLightSource( PointLight   ( Vec3( -0.5*w, 0.0,  40.0 ), lightgray ) );
      pov->addLightSource( ParallelLight( Vec3(  0.0,   0.0, 100.0 ), lightgray, Shadowless() ) );

      // Calculation of the camera location and focus point
      // Aspect ratio of the POV-Ray camera: 4/3 => Up-angle = 26.565 deg (0.4636 rad)
      const Vec3 location1( 0.6*w, -20.0  ,  7.0*h );
      const Vec3 location2( 1.0*w,   6.0  , 20.0*h );
      const Vec3 focus1   ( 0.5*w,   0.0  ,  3.0*h );
      const Vec3 focus2   (   0  ,   0.6*w,  6.0*h );

      // Configuration of the POV-Ray camera
      CameraID camera = theCamera();
      camera->setLocation( ( animation )?( location1 ):( location2 ) );
      camera->setFocus   ( ( animation )?( focus1    ):( focus2    ) );

      // Animating the POV-Ray camera
      if( animation )
         camera->animate( LinearShift( 0, timesteps, location1, location2, focus1, focus2 ) );

      // Configuration of the texture policy
      if( colorProcesses ) {
         std::ostringstream texture;
         texture << "Texture" << theMPISystem()->getRank()%13;
         pov->setTexturePolicy( DefaultTexture( CustomTexture( texture.str() ) ) );
      }
      else {
         pov->setTexturePolicy( ParticleTexturePolicy() );
      }
   }


   /////////////////////////////////////////////////////
   // Setup of the MPI processes

   /*        \ ... /
    *      0  \   / processes1-1
    *          \ /
    *    -------O-------
    *      processes1
    *    ---------------
    *  y       ...
    *  ^ ---------------
    *  |   processes-1
    *  |
    *  +---> x
    *
    *  Origin is at "O".
    */

   if( processes1 == 1 ) {
      // The domain at the stairs is not decomposed into slices
      if( rank < processes1 ) {
         // Connecting the processes around the stair
         defineLocalDomain( HalfSpace( Vec3( 0, 1, 0 ), 0 ) );
         connect( 1, HalfSpace( Vec3( 0, -1, 0 ), 0 ) );
      }
      else {
         // Connecting the processes at the bottom of the stair
         const real dy( real(20)/processes2 );
         const int i = rank - processes1;

         connect( rank - 1, HalfSpace( Vec3( 0, 1, 0 ), -i * dy ) );

         if( rank + 1 < processes ) {
            defineLocalDomain( intersect(
               HalfSpace( Vec3( 0, -1, 0 ), i*dy ),
               HalfSpace( Vec3( 0, +1, 0 ), -(i+1)*dy ) ) );

            connect( rank + 1, HalfSpace( Vec3( 0, -1, 0 ), ( i + 1 ) * dy ) );
         }
         else {
            defineLocalDomain( HalfSpace( Vec3(0, -1, 0), i*dy ) );
         }
      }
   }
   else {
      // The domain at the stairs is decomposed into at least two slices
      if( rank < processes1 ) {
         // Connecting the processes around the stair
         // Normals of separating planes of the stair slices pointing in counterclockwise direction
         Vec3 n[4];
         for( int i = rank - 1; i <= rank + 2; ++i ) {
            if( i == processes1 )
               // ensure that no rounding errors are present
               n[i - (rank - 1)] = Vec3( 0, 1, 0 );
            else
               n[i - (rank - 1)] = Quat( Vec3( 0, 0, -1 ), i * alpha ).rotate( Vec3( 0, -1, 0 ) );
         }

         if( rank > 0 ) {
            connect( rank - 1, intersect(
               HalfSpace( -n[0], 0 ),
               HalfSpace(  n[1], 0 ) ) );
         }
         else {
            // first slice of stairs' half circle has no slice but a half space as a "predecessor"
            connect( processes1, HalfSpace( n[1], 0 ) );
            //connect( processes1, HalfSpace( -n[2], 0 ) ); // FIXME; REVIEWED n[1] is the normal of rank 0 which is (0, -1, 0) which is exactly what we need
         }

         defineLocalDomain( intersect(
            HalfSpace( -n[1], 0 ),
            HalfSpace(  n[2], 0 ) ) );

         if( rank < processes1 - 1 ) {
            connect( rank + 1, intersect(
               HalfSpace( -n[2], 0 ),
               HalfSpace(  n[3], 0 ) ) );
         }
         else {
            // last slice of stairs' half circle has no slice but a half space as a successor
            connect( rank + 1, HalfSpace( -n[2], 0 ) );
         }
      }
      else {
         // Connecting the processes at the bottom of the stair
         const real dy( real(20)/processes2 );
         const int i = rank - processes1;

         if( rank == processes1 ) {
            connect( 0, intersect(
               HalfSpace( Vec3( 0, 1, 0 ), 0 ),
               HalfSpace( Quat( Vec3( 0, 0, -1 ), 1 * alpha ).rotate( Vec3( 0, -1, 0 ) ), 0 ) ) );

            connect( processes1 - 1, intersect(
               HalfSpace( Vec3( 0, 1, 0 ), 0 ),
               HalfSpace( Quat( Vec3( 0, 0, -1 ), ( processes1 - 1 ) * alpha ).rotate( Vec3( 0, +1, 0 ) ), 0 ) ) );
            //   HalfSpace( Quat( Vec3( 0, 0, -1 ), ( processes1 - 1 ) * alpha ).rotate( Vec3( 0, -1, 0 ) ), 0 ) ) );  // FIXME; REVIEWED (0,+1,0) is exactly the normal we need
         }
         else
            connect( rank - 1, HalfSpace( Vec3(0, +1, 0), -i*dy ) );

         if( rank + 1 < processes ) {
            defineLocalDomain( intersect(
               HalfSpace( Vec3( 0, -1, 0 ), i*dy ),
               HalfSpace( Vec3( 0, +1, 0 ), -(i+1)*dy ) ) );

            connect( rank + 1, HalfSpace( Vec3(0, -1, 0), (i+1)*dy ) );
         }
         else {
            defineLocalDomain( HalfSpace( Vec3(0, -1, 0), i*dy ) );
         }
      }
   }

#ifndef NDEBUG
   // Checking the process setup
   theMPISystem()->checkProcesses();
#endif


   /////////////////////////////////////////////////////
   // Setup of the global objects

   real sheight( 0.5*h );

   pe_GLOBAL_SECTION
   {
      // Ground plane setup
      PlaneID ground = createPlane( 100, 0.0, 0.0, 1.0, 0.0, granite );
      if( povray ) pov->setTexture( ground, groundTexture );

      // Back plane setup
      PlaneID wall = createPlane( 101, 0.0, -1.0, 0.0, -(w+wrail+5.0), granite );
      if( povray ) pov->setTexture( wall, wallTexture);

      // Creating the stair
      pe_CREATE_UNION( stair, 102 )
      {
         const real   a( std::atan( d/w ) );     // Rotation angle between each stair
         const size_t N( std::ceil( M_PI/a ) );  // Total number of steps
         const real   l( (N+2)*h );              // Length of the center capsule

         size_t id( 0 );
         real rheight( 0.5*hrail ), angle( 0.0 );
         real cosa ( std::cos( angle ) ), sina( std::sin( angle ) );
         real rxpos( w+0.5*wrail ), rypos( 0.0 );

         // Creating the center capsule
         CapsuleID center = createCapsule( id++, 0.0, 0.0, 0.5*l, cradius, l, granite );
         center->rotate( 0.0, M_PI/2.0, 0.0 );
         if( povray ) pov->setTexture( center, stoneTexture );

         for( size_t i=0; i<N+2; ++i )
         {
            BoxID box1 = createBox( ++id, 0.125*w*cosa, 0.125*w*sina, sheight, 0.25*w, d, h, granite );
            box1->rotate( 0.0, 0.0, angle );
            if( povray ) pov->setTexture( box1, stoneTexture );

            BoxID box2 = createBox( ++id, 0.575*w*cosa, 0.575*w*sina, sheight, 0.65*w, d, h, granite );
            box2->rotate( 0.0, 0.0, angle );
            if( povray ) pov->setTexture( box2, redTexture );

            BoxID box3 = createBox( ++id, 0.95*w*cosa, 0.95*w*sina, sheight, 0.1*w, d, h, granite );
            box3->rotate( 0.0, 0.0, angle );
            if( povray ) pov->setTexture( box3, stoneTexture );

            BoxID rail = createBox( ++id, rxpos, rypos, rheight, wrail, d, hrail, granite );
            rail->rotate( 0.0, 0.0, angle );
            if( povray ) pov->setTexture( rail, stoneTexture );

            SphereID sphere = createSphere( ++id, rxpos, rypos, rheight+0.5*(hrail+wrail), 0.5*wrail, granite );
            if( povray ) pov->setTexture( sphere, goldTexture );

            angle   += a;
            cosa    = std::cos(angle);
            sina    = std::sin(angle);
            sheight += h;
            rxpos   = (w+0.5*wrail)*std::cos(angle);
            rypos   = (w+0.5*wrail)*std::sin(angle);
            rheight += h;
         }

         stair->setFixed( true );
      }
   }


   /////////////////////////////////////////////////////
   // Simulation loop

   timing::WcTimer totalTime;
   timing::WcTimer simTime;
   size_t counter( 0 );

   const size_t frequency( std::ceil( real(2.2)*radius / stepsize ) );

   totalTime.start();

   pe_EXCLUSIVE_SECTION( 0 ) {
      std::cout << "\n--" << pe_BROWN << "RIGID BODY SIMULATION" << pe_OLDCOLOR
                << "---------------------------------------------------------" << std::endl;
   }

   // Running the stair simulation
   for( size_t timestep=0; timestep<timesteps; ++timestep )
   {
      pe_EXCLUSIVE_SECTION( 0 )
      {
         if( verbose )
            std::cout << "\r Simulating time step " <<  timestep+1 << "   " << std::flush;

         // Creating new particles
         if( timestep % frequency == 0 ) {
            for( size_t i=0; i<nz; ++i ) {
               real x( -0.25*w-radius );
               for( size_t j=0; j<nx; ++j ) {
                  SphereID sphere = createSphere( rand<size_t>(0,12), Vec3( x, 0.0, sheight+(i+2)*space ), radius, granular );
                  sphere->setLinearVel( 0.0, rand<real>(2.0,2.2), 0.0 );
                  if( overlap( world->begin(), world->end(), sphere ) )
                     destroy( sphere );
                  else ++counter;
                  x -= space;
               }
            }
         }
      }

      pe_EXCLUSIVE_SECTION( processes1-1 )
      {
         // Destroying old particles
         for( World::Iterator body=world->begin(); body!=world->end(); ) {
            if( body->getPosition()[1] < -real(20.5) )
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
      std::cout << "\n\n"
                << " Total number of particles : " << counter << "\n"
                << " WC-Time:  Pure simulation : " << simTime.total() << "\n"
                << "           Total time      : " << totalTime.total() << "\n";
      std::cout << "--------------------------------------------------------------------------------\n" << std::endl;
   }


   /////////////////////////////////////////////////////
   // MPI Finalization

   MPI_Finalize();
}
//*************************************************************************************************
