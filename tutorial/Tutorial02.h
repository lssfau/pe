//=================================================================================================
/*!
 *  \file Tutorial02.h
 *  \brief Physics Engine Tutorial 02
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

#ifndef _PE_TUTORIAL02_H_
#define _PE_TUTORIAL02_H_


//*************************************************************************************************
/*!\page tutorial02 Tutorial 2: The POV-Ray visualization module
 *
 * \image html boxstack.jpg
 *
 * Tutorial 1 has given you a first impression of how a simulation setup of the \b pe engine
 * looks like. The setup of a rigid body simulation is definitively by far the most important
 * part of the physics engine, but the engine provides some additional very interesting modules,
 * as for example several modules for the visualization of a rigid body simulation. Tutorial 1
 * remained silent about how you can visualize what's happening during the simulation. Tutorial
 * 2 will therefore demonstrate one of the visualization modules, the POV-Ray ray-tracing module
 * for high-quality ray-tracing visualizations.\n
 * POV-Ray is an open source ray-tracing tool available for free at \b www.povray.org. The basic
 * concept of POV-Ray is the conversion of an ASCII scene description into a ray-traced image of
 * the scene. The following example demonstrates such a POV-Ray file:

   \code
   // As in C/C++ code it is possible to include some input files that may contain additional
   // colors or textures. For instance, the colors.inc header contains several color definitions
   // that will come in handy, whereas the woods.inc and metals.inc header contain wood and metal
   // texture definitions that closely resemble the appearance of real wood or metal.
   #include "colors.inc"
   #include "woods.inc"
   #include "metals.inc"

   // The POV-Ray camera is the center of a POV-Ray visualization and probably the single most
   // important element of a POV-Ray scene. Without a camera, POV-Ray is not able to create an
   // image. Additionally, the location and orientation of the camera defines what elements
   // are visible in a rendered image.
   camera {
      location <8,9,-25>
      look_at <0,7.5,0>
      sky <0,1,0>
   }

   // Light sources are used to illuminate a POV-Ray scene. Without light sources, rigid bodies
   // are only lighted by their own ambient lighting, which gives them a rather two dimensional
   // appearance. Due to shadows, diffuse lighting and reflections, light sources are able to
   // create a fully three dimensional impression.
   light_source {
      <0,30,-10>
      color rgbf <0.95,0.95,0.95,0>
   }

   // The background color defines which color you see if a cast ray doesn't hit any object in
   // your scene.
   background { color rgbf <1,1,1,0> }

   // One example for an object in the scene is the following plane that we use as ground in our
   // simulation. The first parameters of every object define the general size and orientation
   // of the object. In case of the plane, these two values specify the plane normal and the
   // displacement from the origin of the simulation world. The following texture block contains
   // all information about the appearance of the object: the pigment specifies the color or
   // color pattern, the finish defines how the surface of the rigid body interact with light
   // and possible transformations like scale or rotation operations shift the texture to the
   // desired position and orientation.
   plane {
      <0,1,0>, 0
      texture {
         pigment {
            image_map { gif "grass.gif" map_type 0 interpolate 2 }
         }
         finish {
            ambient 0.2
         }
         scale 20
         rotate <-90,-0,-0>
      }
   }

   ...
   \endcode

 * The \b pe approach towards a POV-Ray visualization is the generation of a POV-Ray file for
 * every specified time step. These files can then be rendered offline and be combined into
 * animations.\n
 * Our goal for this tutorial is now to extend the rigid body simulation of tutorial 1 and to
 * setup the POV-Ray module such that we are creating a single POV-Ray file every ten time steps
 * giving us a nice impression of the course of the simulation. Let's start by selecting the
 * necessary headers for this tutorial:

   \code
   // Remember that you included the <pe/core.h> header for the core of the physics engine and
   // the <pe/util.h> header for the random number generation (we still use the angle() function).
   // Additionally, this time we will include the <pe/povray.h> header for all POV-Ray related
   // functionality. This header gives us access to everything we need in order to create POV-Ray
   // files. To make things easier for us, we will also use a using directive for the POV-Ray
   // namespace within the engine, pe::povray.
   #include <pe/core.h>
   #include <pe/util.h>
   #include <pe/povray.h>
   using namespace pe;
   using namespace pe::povray;
   \endcode

 * Let's continue by recalling the first part of tutorial 1: the setup of the boxstack example.

   \code
   // We will use the main function without any command line arguments. Any value we will need will
   // be hardcoded into our executable. I know, this is not flexible, but should suffice for this
   // example.
   int main()
   {
      // Constants and variables
      const unsigned int timesteps ( 1000 );  // Total number of time steps
      const unsigned int H( 4 );              // Height of the box stack
      unsigned int id( 0 );                   // User-specific ID counter

      // Simulation world setup
      WorldID world = activateWorld();
      world->setGravity( 0.0, 0.0, -0.4 );

      // Setup of the ground plane
      createPlane( ++id, 0.0, 0.0, 1.0, 0.0, granite );

      // Setup of the wooden box stack
      for( unsigned int i=H; i>0; --i ) {
         for( unsigned int j=0; j<i; ++j )
         {
            const Vec3 pos( -2.5*(i-1)+j*5.0, 0.0, 2.0+(H-i)*4.0 );
            BoxID box = createBox( ++id, pos, 4.0, 4.0, 4.0, oak );
            box->rotate( 0.0, 0.0, angle() );
         }
      }

      // Setup of the metal sphere
      SphereID sphere = createSphere( ++id, 0.0, -25.0, 8.0, 1.5, iron );
      sphere->setLinearVel( 0.0, 5.5, 0.1 );

      ...
   \endcode

 * At this point we will add the configuration of our POV-Ray visualization.

   \code
      ...

      // Let's start by activating and configuring the POV-Ray writer. The POV-Ray writer is the
      // one instance that creates POV-Ray files according to our specifications. We can simple
      // activate the POV-Ray visualization by a call to the activateWriter() function in the
      // pe::povray namespace. It returns a WriterID that represents a handle to the one existing
      // POV-Ray writer. All currently visible rigid bodies are automatically added to the POV-Ray
      // visualization. Every rigid body that is created after the call to activateWriter() that
      // is made visible is also automatically added to the POV-Ray visualization. So, conveniently
      // there is no need to explicitly add or remove rigid bodies from the POV-Ray visualization.
      WriterID pov = activateWriter();

      // The following lines configure the basic settings of our POV-Ray visualization. The first
      // call to the setSpacing function specifies that every 10 time steps we want to create a
      // POV-Ray file. The include() function will add an include directive to every POV-Ray file
      // so we will now be able to use all colors and textures defined in these headers. The next
      // function specifies the directory and filename for the POV-Ray files. Note that the \a %
      // sign will be replaced by the number of the POV-Ray image. The setBackground() function
      // sets the color of the POV-Ray background that will be used whenever a ray doesn't hit
      // one of our objects.
      pov->setSpacing( 10 );
      pov->include( "colors.inc" );
      pov->include( "woods.inc" );
      pov->include( "metals.inc" );
      pov->setFilename( "./video/box%.pov" );
      pov->setBackground( 1.0, 1.0, 1.0 );

      // Let's continue by configuring the POV-Ray camera. There is exactly one camera in every
      // POV-Ray scene. The call to the theCamera() function returns a handle to this camera.
      // Since the default settings for the location and the focus point of the camera don't
      // suit our requirements, we explicitly specify the location and the focus point.
      CameraID camera = theCamera();
      camera->setLocation( 8.0, -25.0, 9.0 );
      camera->setFocus   ( 0.0,   0.0, 7.5 );

      // We also have to think about lighting our scene. Without light, all rigid bodies are only
      // lighted by their own ambient lighting, which gives them a rather boring, two dimensional
      // appearance. For this simulation we choose a single point light at location (0,-10,30). A
      // point light source emits light of the specified color uniformly in all directions. After
      // the setup, we add the point light to the POV-Ray scene by a call to the addLightSource()
      // function.
      PointLight pointlight(
         Vec3 ( 0.0 , -10.0 , 30.0  ),  // Location of the point light source
         Color( 0.95,   0.95,  0.95 )   // Color of the point light source
      );
      pov->addLightSource( pointlight );

      // The next step is the setup of the appearance of our ground plane. We don't want it to
      // appear as a barren, gray surface, but instead as a nice and as realistic as possible
      // ground for our simulation. The choice in this case is a grass texture. The first step
      // is the setup of a pigment that represents the color pattern of the texture. For our
      // grass texture, we use an image pigment that projects the given image on the plane. The
      // first parameter specifies the file format, the second the name of the image file, the
      // third the kind of mapping and with the fourth parameter we let the image repeat itself
      // in order to cover the entire plane.
      ImagePigment grassPigment( gif, "grass.gif", planar, true );

      // The second step is the definition of a finish for the grass texture. A finish defines
      // how the surface of the rigid body interacts with light. In our case it is enough to
      // define a slightly increased ambient glow for our ground plane.
      Finish grassFinish(
         Ambient( 0.2 )
      );

      // Both the pigment and the finish can now be used to create the grass texture. For this
      // purpose we use the PlainTexture class. Additionally we scale the texture by a factor
      // of twenty and rotate it such that it is projected onto the xy-plane.
      PlainTexture grassTexture(
         grassPigment,
         grassFinish,
         Scale( 20.0 ),
         Rotation( PI/2.0, 0.0, 0.0 )
      );

      // The last remaining step is to apply the texture to our ground plane. By default, every
      // finite rigid bodies (as for instance spheres, boxes and capsules) have an unicolored,
      // orange texture and infinite rigid bodies (as for instance planes) have a unicolored,
      // gray texture. To change the default behavior, we have to specify the appearance of a
      // rigid body by setting the texture explicitly.
      pov->setTexture( plane, grassTexture );

      // The appearance of our ground plane is finished, which only leaves us with the boxes
      // and the single sphere. Let's continue by creating some nice textures for the boxes.
      // Since we want the boxes to appear as wooden boxes, we will use the predefined wood
      // textures from the 'woods.inc' header file. Since the wood textures all use the prefix
      // 'T_Wood' followed by the number of the texture, we will just manually create different
      // wood textures for our boxes by combining the 'Wood_T' prefix with a random number.
      // The first thing to note is the loop over our boxes. This loop uses the pe way of
      // selecting a specific geometry from a range of rigid bodies. As it is custom in C++,
      // we create an iterator over the rigid bodies of the simulation world. However, we
      // use a 'CastIterator<Box>' to specifically traverse all boxes contained in the world.
      // For this purpose we also have to use the templated begin() and end() functions to
      // select the range of boxes contained in the world.
      std::ostringstream oss;
      for( World::CastIterator<Box> b=world->begin<Box>(); b!=world->end<Box>(); ++b )
      {
         // For every box we create a random wood texture from 'Wood_T1' to 'Wood_T12'. Since
         // this time we are not creating our own texture, but instead we use a predefined one
         // we have to use a CustomTexture instead of a PlainTexture.
         oss.str( "" );
         oss << "T_Wood" << rand<unsigned int>( 1, 12 );
         pov->setTexture( *b, CustomTexture( oss.str() ) );
      }

      // The last remaining rigid body is the sphere. Since we used the iron material to create
      // the sphere, we also want to the sphere a metallic appearance. A very nice and shiny
      // option are the chrome textures from the 'metals.inc' header file.
      pov->setTexture( s, CustomTexture( "T_Chrome_1A" ) );

      // After the POV-Ray setup, we can start our simulation as we did in the first tutorial.
      // However, this time we additionally create a POV-Ray file every ten time steps.
      world->run( timesteps, 0.05 );
   }
   \endcode

 * This completes the second tutorial. However, this is by far not the end of what you can do
 * with the POV-Ray module. Feel free to go to the POV-Ray module documentation to find out more
 * about all possible kinds of light sources, textures, pigments, finishes, normals, ... The
 * complete source code for tutorial 1 and 2 can be found in the corresponding example directory
 * "<install-path>/examples/boxstack/".
 */
//*************************************************************************************************

#endif
