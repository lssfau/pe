//=================================================================================================
/*!
 *  \file Tutorial00.h
 *  \brief Physics Engine Tutorial 00
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

#ifndef _PE_TUTORIAL00_H_
#define _PE_TUTORIAL00_H_


//*************************************************************************************************
/*!\page tutorial00 Tutorial 0: Basics
 *
 * \section intro Introduction
 *
 * Welcome to the \b pe rigid body physics engine. Hopyfully this engine provides you with the
 * means to simulate rigid body dynamics in a physically accurate fashion and in a comfortable
 * way.\n
 * The tutorials are building on each other. Therefore it is recommended to read them in the
 * given order. However, feel free to skip one or the other tutorial or to start a detour via
 * one of the given links to learn more about a specific class of the engine.\n
 * Hopefully you enjoy using \b pe. However, in case of problems or bugs, please contact the
 * author of the engine via eMail:
 *
 *                     klaus.iglberger@informatik.uni-erlangen.de\n\n
 *
 *
 * \section physics Physical environment
 *
 * Let's start by defining some basic physical properties and preconditions for the \b pe physics
 * engine. The \b pe engine uses a right-handed coordinate system as illustrated below:
 *
 * \image html coordinates.png
 * \image latex coordinates.eps "Coordinate system" width=200pt
 *
 * Every coordinate, velocity, impulse or force relates to this coordinate system. So whenever you
 * have to specify a vector (x,y,z), the first component relates to the x-axis, the second to the
 * y-axis and the third to the z-axis.\n
 * Furthermore in \b pe all angles and the angluar velocity around an axis are specified in radian
 * measure. The table below shows some examples for angles in radian measure and degrees:
 *
 * <table border="1" cellspacing="0" cellpadding="2">
 *    <tr>
 *       <td align="left"   width="150px"> <b> Radian measure </b> </td>
 *       <td align="center" width="80px"> 0 </td>
 *       <td align="center" width="80px"> \f$ \frac{\pi}{6} \f$ </td>
 *       <td align="center" width="80px"> \f$ \frac{\pi}{4} \f$ </td>
 *       <td align="center" width="80px"> \f$ \frac{\pi}{3} \f$ </td>
 *       <td align="center" width="80px"> \f$ \frac{\pi}{2} \f$ </td>
 *       <td align="center" width="80px"> \f$ \pi \f$ </td>
 *       <td align="center" width="80px"> \f$ \frac{3}{2} \cdot \pi \f$ </td>
 *       <td align="center" width="80px"> \f$ 2 \cdot \pi \f$ </td>
 *    </tr>
 *    <tr>
 *       <td align="left"   width="150px"> <b> Degrees </b> </td>
 *       <td align="center" width="80px"> 0 </td>
 *       <td align="center" width="80px"> 30 </td>
 *       <td align="center" width="80px"> 45 </td>
 *       <td align="center" width="80px"> 60 </td>
 *       <td align="center" width="80px"> 90 </td>
 *       <td align="center" width="80px"> 180 </td>
 *       <td align="center" width="80px"> 270 </td>
 *       <td align="center" width="80px"> 360 </td>
 *    </tr>
 * </table>
 *
 * For example, if you want to rotate a sphere by 90° around the y-axis, the appropriate function
 * call would be

   \code
   sphere->rotate( 0, PI/2, 0 );
   \endcode

 * As already mentioned, the radian measure also applies for angular velocities. If an object has
 * an angular velocity of \f$ (0,\frac{\pi}{2},0) \f$ then the object will rotate by 90° per time
 * unit around the y-axis.\n
 * Note that the right-handed coordinate system and the radian measure for angles even applies
 * in wrapper functions and classes that deal with different systems. For example the raytracing
 * tool PovRay internally uses a left-handed coordinate system. However, the pe::PovRayWriter
 * class that writes PovRay scene files, uses the \b pe coordinate system. So all you have to
 * remember when using the \b pe physics engine is the right-handed coordinate system show above
 * and the radian measure for angles.\n\n
 *
 *
 * \section basics Basic concepts
 *
 * One of the basic concepts in the \b pe engine is the use of IDs. You will encounter all kinds of
 * IDs throughout the engine. Nearly every ID can be used exactly like a pointer. In some cases, the
 * ID is a mere typedef for a plain pointer. However, in other cases the ID is a more sophisticated
 * pointer (e.g. a smart pointer). In order to abstract from the actual type of the pointer, the
 * \b pe engine introduces an ID wherever necessary.

   \code
   WorldID  world;   // Handle for a simulation world
   SphereID sphere;  // Handle for a sphere object
   BoxID    box;     // Handle for a box object
   ...

   world->simulationStep( 0.1 );  // The IDs are used exactly like pointers
   \endcode

 * For every object, there exist two IDs: the standard for non-constant objects and the constant
 * ID for constant objects (e.g. SphereID and ConstSphereID). Note that since the IDs can be treated
 * like pointers, the ConstIDs refer to constant objects. The ID itself can still be changed! In
 * order to make the ID constant, use the const qualifier:

   \code
   SphereID      s  = createSphere( ... );  // Handle for a non-constant sphere object
   ConstSphereID cs = createSphere( ... );  // Handle for a constant sphere object

   s->setLinearVel ( 1, 0, 0 );  // Setting the linear velocity of the non-constant sphere
   cs->setLinearVel( 1, 0, 0 );  // Compile time error: the sphere is constant!

   s  = createSphere( ... );  // Changing the sphere handle to refer to a newly created sphere object
   cs = createSphere( ... );  // The same as above, the ConstSphereID can be changed

   const SphereID s2 = createSphere( ... );  // Creating a non-constant sphere object
   s2 = createSphere( ... );                 // Compile time error: the ID is constant and cannot be changed
   \endcode

 * There is one special ID in the \b pe engine: BodyID. Whenever you create a rigid body, you will
 * get the appropriate ID from the corresponding create function (e.g. SphereID, BoxID, ...). All
 * these IDs can also be used as BodyIDs, since all geometric primitives are also rigid bodies.

   \code
   SphereID sphere = createSphere( ... );  // Creating a new sphere primitive
   BodyID   body   = sphere;               // The sphere is a rigid body at the same time
   \endcode

 * So much for IDs. As already mentioned, you will encounter a lot of IDs when working with \b pe.
 * The next basic concept is about the creation and destruction of objects. In most cases you will
 * find a create/destroy pair of functions:

   \code
   SphereID sphere = createSphere( ... );
   ...
   destroy( sphere );
   \endcode

 * Most IDs are created via a create function and destroyed via a function called destroy. The
 * example above shows the function pair for a sphere primitive: createSphere() creates the sphere
 * and automatically adds it to the rigid body simulation world, the destroy() function destroys
 * it. Note that it is not mandatory to use the destroy function after the object is not needed
 * any more. The \b pe engine makes sure every object is removed again even if no destroy function
 * is called (however, it is good style to use them anyway).\n\n
 *
 *
 * \section names Naming convention
 *
 * The \b pe engine uses a special, yet not uncommon naming convention for classes, functions,
 * members, macros, ... The following table should give you an idea of the naming convention
 * you will encounter throughout the engine:
 *
 * <table border="1" cellspacing="0" cellpadding="2">
 *    <tr>
 *       <td width="360px"> <b> Classes/Types </b> </td>
 *       <td width="200px"> <b> LikeThis </b> </td>
 *    </tr>
 *    <tr>
 *       <td> <b> Functions (both non-member and member) </b> </td>
 *       <td> <b> likeThis </b> </td>
 *    </tr>
 *    <tr>
 *       <td> <b> member data </b> </td>
 *       <td> <b> likeThis_ </b> </td>
 *    </tr>
 *    <tr>
 *       <td> <b> Mathematical constants </b> </td>
 *       <td> <b> M_LIKE_THIS </b> </td>
 *    </tr>
 *    <tr>
 *       <td> <b> Macros </b> </td>
 *       <td> <b> pe_LIKE_THIS </b> </td>
 *    </tr>
 * </table>
 */
//*************************************************************************************************

#endif
