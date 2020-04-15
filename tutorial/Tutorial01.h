//=================================================================================================
/*!
 *  \file Tutorial01.h
 *  \brief Physics Engine Tutorial 01
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

#ifndef _PE_TUTORIAL01_H_
#define _PE_TUTORIAL01_H_


//*************************************************************************************************
/*!\page tutorial01 Tutorial 1: A first simulation setup
 *
 * \image html boxstack.jpg
 *
 * This tutorial demonstrates a first simulation setup with the \b pe physics engine. Hopefully
 * the images give a fairly good idea of the final goal of this tutorial: we want to cave in
 * the stack of wooden boxes using a metal sphere. This tutorial gives you a first idea of how
 * rigid bodies are created in \b pe and how a simulation is run after the initial setup has
 * been completed.\n
 * The complete source code can be found in "<install-path>/examples/boxstack/". For Linux
 * systems a fitting Makefile is provided to enable you to compile and test this example for
 * yourself.\n
 * Let's start by writing the header of our source file:

   \code
   // For this example we will only need the following headers. In order to include the core of
   // the pe physics engine, you only need to include the <pe/core.h> header. The core contains
   // the essential parts of the physic engine, including all the necessary data types and
   // geometries. Additionally, we will use the <pe/util.h> header for the utility module of
   // the engine. This module contains a random number generator we will use to add a little
   // randomness to our simulation. In order to facilitate the next steps, we will also use a
   // using directive for the pe namespace.
   #include <pe/core.h>
   #include <pe/util.h>
   using namespace pe;
   \endcode

 * The next step will be the definition of a helper function to create small, random angles
 * in the range of \f$ [-\frac{\pi}{20}..\frac{\pi}{20}] \f$.

   \code
   // The angle() function will be used to rotate the boxes by a small angle around the z-axis
   // to make the appearance of the simulation more natural. The function takes no arguments
   // and returns a random angle as real value. real is the preferred floating point data type
   // of the pe engine. Depending on the desired accuracy, real can be a single-precision or
   // double-precision floating point type (whereas the default is double-precision).
   // To generate the random angles, we use one of the two templated rand() functions from the
   // utility module. As template argument we specify that we want to generate a random real
   // value. The function arguments set the range of the generated random value.
   real angle()
   {
      return rand<real>( -PI/20, PI/20 );
   }
   \endcode

 * Let's take a look at the real stuff. The next step will be the main function of our simulation:

   \code
   // We will use the main function without any command line arguments. Any value we will need will
   // be hardcoded into our executable. I know, this is not flexible, but should suffice for this
   // example.
   int main()
   {
      // We start by defining some constant values...
      const unsigned int timesteps ( 1000 );  // Total number of time steps
      const unsigned int H( 4 );              // Height of the box stack

      // ...and some non-constant variables
      unsigned int id( 0 );  // User-specific ID counter

      // Next we configure the simulation world, which will be the container for all our rigid bodies.
      // In most simulations the configuration of the simulation world will be the first thing you
      // have to do. For instance, for this particular simulation, we set the gravity to -0.4 in
      // -z-direction.
      // Note that creating a new rigid body will also activate the simulation world. It is also not
      // necessary to destroy the world after the simulation has finished. It will be automatically
      // destroyed along with all contained rigid bodies. The pe physics engine makes sure every
      // object is destroyed if it is not needed any more.
      WorldID world = theWorld();
      world->setGravity( 0.0, 0.0, -0.4 );

      // The first rigid body in our simulation world is the ground plane. The create function is
      // responsible to automatically add the new plane to the world. The first argument sets the
      // user-specific ID of the plane. We will not need this value and therefore use an automatic
      // ID counter. The next four arguments set the normal or the plane and the displacement from
      // the origin of the global world frame. With the last argument we specify the material of
      // the plane; in this case we use granite.
      createPlane( ++id, 0.0, 0.0, 1.0, 0.0, granite );

      // Next we will create the box stack. Don't worry too much about the loops. They are simply
      // running over all levels of the stack and create an appropriate amount of boxes per level.
      for( unsigned int i=H; i>0; --i ) {
         for( unsigned int j=0; j<i; ++j )
         {
            // In this line we calculate the position of the current box. Important to note is
            // the used data type Vec3. You will find this data type for a three dimensional
            // vector throughout the engine.
            const Vec3 pos( -2.5*(i-1)+j*5.0, 0.0, 2.0+(H-i)*4.0 );

            // The next line creates a new box within the simulation world at the calculated position
            // and with the box side lengths (4,4,4). The last parameter specifies the material of the
            // box (oak wood). This time we remember the return value of the create function. In pe
            // every create function returns an ID of some sort. In this case we get a BoxID, a handle
            // to the created box primitive.
            BoxID box = createBox( ++id, pos, 4.0, 4.0, 4.0, oak );

            // The BoxID is used exactly like a pointer. We use it to slightly rotate the box around
            // the z-axis to improve the appearance of our simulation. The rotate function takes
            // three euler angles in radian measure, one for each axis.
            box->rotate( 0.0, 0.0, angle() );
         }
      }

      // After the box stack has been created, we initialize our metal sphere. The first argument
      // of the createSphere function is again the user-specific ID. The next three values specify
      // the global position of the center of the sphere. After this, we set the radius of the
      // sphere to 1.5 and the material to iron.
      SphereID sphere = createSphere( ++id, 0.0, -25.0, 8.0, 1.5, iron );

      // For making the sphere fly towards the box stack, we initialize the linear velocity of
      // the sphere by 5.5 in y-direction and 0.1 in z-direction.
      sphere->setLinearVel( 0.0, 5.5, 0.1 );


      // Finally we have set up the simulation and are able to run it. The following function
      // performs the specified number of time steps in the simulation world using a time step of
      // 0.05 time units. The simulation will handle all rigid bodies according to the acting
      // forces their velocities and will treat all collisions during this time span. An important
      // point to note is that the time step size is an important balance between accuracy an
      // performance of the simulation: the smaller the time step is, the more accurate the
      // simulation will be, but the more time steps will be needed.
      world->run( timesteps, 0.05 );
   }
   \endcode

 * With only a few lines of code we have created our first rigid body simulation. Feel free to
 * try the source code in "<install-path>/examples/boxstack/" for yourself or to follow any
 * of the links in this tutorial to learn more about a particluar function or class.
 */
//*************************************************************************************************

#endif
