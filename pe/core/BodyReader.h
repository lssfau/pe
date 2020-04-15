//=================================================================================================
/*!
 *  \file pe/core/BodyReader.h
 *  \brief Extractor for rigid body parameter files
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

#ifndef _PE_CORE_BODYREADER_H_
#define _PE_CORE_BODYREADER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <fstream>
#include <iosfwd>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <pe/core/GeomType.h>
#include <pe/core/Types.h>
#include <pe/core/WorldID.h>
#include <pe/math/Vector3.h>
#include <pe/povray/Finish.h>
#include <pe/povray/ImageMap.h>
#include <pe/povray/LightSource.h>
#include <pe/povray/Normal.h>
#include <pe/povray/Pigment.h>
#include <pe/povray/Texture.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body parameter file extractor.
 * \ingroup core
 *
 * The body reader extracts rigid body parameters from a file and performs the necessary setup
 * of the bodies. Additionally, the body reader can set up a POV-Ray visualization for the
 * specified rigid bodies. In the following sections, the file formats and the necessary and
 * optional parameters for both rigid body and POV-Ray setup are explained in detail.\n\n
 *
 *
 * \section content Contents
 *
 * <ol>
 *    <li> \ref general\n </li>
 *
 *    <li> \ref sphere </li>
 *    <li> \ref box </li>
 *    <li> \ref capsule </li>
 *    <li> \ref cylinder </li>
 *    <li> \ref plane </li>
 *    <li> \ref tetrasphere </li>
 *    <li> \ref agglomerate </li>
 *    <li> \ref union\n </li>
 *
 *    <li> \ref spring\n </li>
 *
 *    <li> \ref material\n </li>
 *
 *    <li> \ref povray
 *       <ol>
 *          <li> \ref color </li>
 *          <li> \ref colormap </li>
 *          <li> \ref imagemap </li>
 *          <li> \ref lightsource
 *             <ol>
 *                <li> \ref point_light </li>
 *                <li> \ref spotlight </li>
 *                <li> \ref parallel_light </li>
 *                <li> \ref area_light </li>
 *             </ol>
 *          </li>
 *          <li> \ref pigment
 *             <ol>
 *                <li> \ref color_pigment </li>
 *                <li> \ref agate_pigment </li>
 *                <li> \ref bozo_pigment </li>
 *                <li> \ref granite_pigment </li>
 *                <li> \ref marble_pigment </li>
 *                <li> \ref radial_pigment </li>
 *                <li> \ref spotted_pigment </li>
 *                <li> \ref custom_pigment </li>
 *             </ol>
 *          </li>
 *          <li> \ref finish </li>
 *          <li> \ref normal
 *             <ol>
 *                <li> \ref agate_normal </li>
 *                <li> \ref bozo_normal </li>
 *                <li> \ref bumps_normal </li>
 *                <li> \ref dents_normal </li>
 *                <li> \ref granite_normal </li>
 *                <li> \ref marble_normal </li>
 *                <li> \ref ripples_normal </li>
 *                <li> \ref spotted_normal </li>
 *                <li> \ref waves_normal </li>
 *                <li> \ref wrinkles_normal </li>
 *                <li> \ref custom_normal </li>
 *             </ol>
 *          </li>
 *          <li> \ref texture
 *             <ol>
 *                <li> \ref plain_texture </li>
 *                <li> \ref tiled_texture </li>
 *                <li> \ref layered_texture </li>
 *                <li> \ref custom_texture </li>
 *             </ol>
 *          </li>
 *          <li> \ref modifier
 *             <ol>
 *                <li> \ref frequency </li>
 *                <li> \ref lambda </li>
 *                <li> \ref octaves </li>
 *                <li> \ref omega </li>
 *                <li> \ref phase </li>
 *                <li> \ref turbulence </li>
 *             </ol>
 *          </li>
 *          <li> \ref transformation
 *             <ol>
 *                <li> \ref scale </li>
 *                <li> \ref translate </li>
 *                <li> \ref rotate\n\n\n </li>
 *             </ol>
 *          </li>
 *       </ol>
 *    </li>
 * </ol>
 *
 *
 * \section general General remarks
 *
 * This section explains some general rules for the rigid body extraction.
 *
 * - The body reader is case-sensitve, which means that e.g. \a union will trigger the extraction
 *   of a union, whereas e.g. \a UNION will be not recognized.
 * - User-specific IDs need not be necessarily unique.
 * - When referencing bodies in links or springs a body with the same label must be already
 *   specified previously in the file. Labels must be unique are case-sensitive and cannot
 *   contain whitespace.
 * - A single line comment is started with \f$ // \f$. A multiline comment is started with
 *   \f$ / \ast \f$ and ended with \f$ \ast / \f$.\n\n\n
 *
 *
 * \section sphere Sphere primitives
 *
 * The \a sphere command starts a parameter section for a single sphere. A sphere section requires
 * at least values for the user-specific ID, the center of the sphere, its radius and its
 * material.
 *
 * \image html sphere.png
 * \image latex sphere.eps "Sphere geometry" width=200pt

   \code
   sphere {
      id x                                // User-specific ID of the sphere
      label L                             // Unique label
      center <x,y,z>                      // Global position of the sphere
      radius r                            // Radius of the sphere
      material name/{ ... }               // Material of the sphere
      linear <vx,vy,vz>                   // Sets the linear velocity of the sphere
      angular <ax,ay,az>                  // Sets the angular velocity of the sphere
      translate <dx,dy,dz>                // Translation of the sphere
      rotate <xangle,yangle,zangle>       // Euler rotation of the sphere
      fixed                               // Fixes the sphere's position
      invisible                           // Makes the sphere invisible
      texture {                           // Specifies the POV-Ray appearance of the
         ...                              //   sphere. For details about the texture
      }                                   //   options see the descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the sphere.
 *  - \b center: the global position of the sphere's center of mass.
 *  - \b radius: the radius of the sphere.
 *  - \b material: the material of the sphere. It is possible to either specify the name of an
 *       existing material or to specify a new material. For detailed information about materials,
 *       see section \ref material .
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs.
 *  - \b linear: specifies the linear velocity of the sphere in x-, y- and z-direction.
 *  - \b angular: specifies the angular velocity of the sphere around the x-, y- and z-axis.
 *  - \b translate: translates the sphere from the specified \b center by the displacement
 *       vector <dx,dy,dz>. For each sphere, several \b translate commands may be specified.
 *       The \b translate command is always executed after the \b center command, regardless
 *       of the order of the commands.
 *  - \b rotate: specifies a rotation around the x-, y- and z-axis, in this order. The given
 *       angles are Euler angles in radian measure. In order to change the order of rotations,
 *       several rotations can be specified.
 *  - \b fixed: fixes the global position of the sphere within the global world frame.
 *  - \b invisible: if set, the sphere will be invisible and will not be shown in any
 *       visualization.
 *  - \b texture: specifies the POV-Ray texture of the sphere. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section box Box primitives
 *
 * The \a box command starts a parameter section for a single box. A box section requires at
 * least a user-specific ID, the center of the box, its side lengths and its material.
 *
 * \image html box.png
 * \image latex box.eps "Box geometry" width=200pt

   \code
   box {
      id x                                // User-specific ID of the box
      label L                             // Unique label
      center <x,y,z>                      // Global position of the box
      lengths <lx,ly,lz>                  // Side lengths of the box
      material name/{ ... }               // Material of the box
      linear <vx,vy,vz>                   // Sets the linear velocity of the box
      angular <ax,ay,az>                  // Sets the angular velocity of the box
      translate <dx,dy,dz>                // Translation of the box
      rotate <xangle,yangle,zangle>       // Euler rotation of the box
      fixed                               // Fixes the box's position
      invisible                           // Makes the box invisible
      texture {                           // Specifies the POV-Ray appearance of the
         ...                              //   box. For details about the texture
      }                                   //   options see the descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the box.
 *  - \b center: the global position of the center of mass of the box.
 *  - \b lengths: lengths of the sides in x-, y- and z-direction.
 *  - \b material: the material of the box. It is possible to either specify the name of an
 *       existing material or to specify a new material. For detailed information about materials,
 *       see section \ref material .
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs.
 *  - \b linear: specifies the linear velocity of the box in x-, y- and z-direction.
 *  - \b angular: specifies the angular velocity of the box around the x-, y- and z-axis.
 *  - \b translate: translates the box from the specified \b center by the displacement
 *       vector <dx,dy,dz>. For each box, several \b translate commands may be specified.
 *       The \b translate command is always executed after the \b center command, regardless
 *       of the order of the commands.
 *  - \b rotate: specifies a rotation around the x-, y- and z-axis, in this order. The
 *       given angles are Euler angles in radian measure. In order to change the order of
 *       rotations, several rotations can be specified.
 *  - \b fixed: fixes the global position of the box within the global world frame.
 *  - \b invisible: if set, the box will be invisible and will not be shown in any
 *       visualization.
 *  - \b texture: specifies the POV-Ray texture of the box. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section capsule Capsule primitives
 *
 * The \a capsule command starts a parameter section for a single capsule. If no rotations are
 * applied the capsule initially lies along the x-axis. A capsule section requires at least a
 * user-specific ID, the center of the capsule, its radius and length and its material.
 *
 * \image html capsule.png
 * \image latex capsule.eps "Capsule geometry" width=200pt

   \code
   capsule {
      id x                                // User-specific ID of the capsule
      label L                             // Unique label
      center <x,y,z>                      // Global position of the capsule
      radius radius                       // Radius of the capsule
      length l                            // Length of the cylinder part
      material name/{ ... }               // Material of the capsule
      linear <vx,vy,vz>                   // Sets the linear velocity of the capsule
      angular <ax,ay,az>                  // Sets the angular velocity of the capsule
      translate <dx,dy,dz>                // Translation of the capsule
      rotate <xangle,yangle,zangle>       // Euler rotation of the capsule
      fixed                               // Fixes the capsule's position
      invisible                           // Makes the capsule invisible
      texture {                           // Specifies the POV-Ray appearance of the
         ...                              //   capsule. For details about the texture
      }                                   //   options see the descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the capsule.
 *  - \b center: the global position of the capsule's center of mass.
 *  - \b radius: the radius of the capsule.
 *  - \b length: the length of the cylinder part within the capsule.
 *  - \b material: the material of the capsule. It is possible to either specify the name of an
 *       existing material or to specify a new material. For detailed information about materials,
 *       see section \ref material .
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs.
 *  - \b linear: specifies the linear velocity of the capsule in x-, y- and z-direction.
 *  - \b angular: specifies the angular velocity of the capsule around the x-, y- and z-axis.
 *  - \b translate: translates the capsule from the specified \b center by the displacement
 *       vector <dx,dy,dz>. For each capsule, several \b translate commands may be specified.
 *       The \b translate command is always executed after the \b center command, regardless
 *       of the order of the commands.
 *  - \b rotate: specifies a rotation around the x-, y- and z-axis, in this order. The given
 *       angles are Euler angles in radian measure. In order to change the order of rotations,
 *       several rotations can be specified.
 *  - \b fixed: fixes the global position of the capsule within the global world frame.
 *  - \b invisible: if set, the capsule will be invisible and will not be shown in any
 *       visualization.
 *  - \b texture: specifies the POV-Ray texture of the capsule. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section cylinder Cylinder primitives
 *
 * The \a cylinder command starts a parameter section for a single cylinder primitive. If no
 * additional rotations are specified, the cylinder initially lies along the x-axis. A cylinder
 * section requires at least a user-specific ID, the center of the cylinder, its radius
 * and length and its material.
 *
 * \image html cylinder.png
 * \image latex cylinder.eps "Cylinder geometry" width=200pt

   \code
   cylinder {
      id x                                // User-specific ID of the cylinder
      label L                             // Unique label
      center <x,y,z>                      // Global position of the cylinder
      radius radius                       // Radius of the cylinder
      length l                            // Length of the cylinder
      material name/{ ... }               // Material of the cylinder
      linear <vx,vy,vz>                   // Sets the linear velocity of the cylinder
      angular <ax,ay,az>                  // Sets the angular velocity of the cylinder
      translate <dx,dy,dz>                // Translation of the cylinder
      rotate <xangle,yangle,zangle>       // Euler rotation of the cylinder
      fixed                               // Fixes the cylinder's position
      invisible                           // Makes the cylinder invisible
      texture {                           // Specifies the POV-Ray appearance of the
         ...                              //   cylinder. For details about the texture
      }                                   //   options see the descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the cylinder.
 *  - \b center: the global position of the cylinder's center of mass.
 *  - \b radius: the radius of the cylinder.
 *  - \b length: the length of the cylinder.
 *  - \b material: the material of the cylinder. It is possible to either specify the name of an
 *       existing material or to specify a new material. For detailed information about materials,
 *       see section \ref material .
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs.
 *  - \b linear: specifies the linear velocity of the cylinder in x-, y- and z-direction.
 *  - \b angular: specifies the angular velocity of the cylinder around the x-, y- and z-axis.
 *  - \b translate: translates the cylinder from the specified \b center by the displacement
 *       vector <dx,dy,dz>. For each cylinder, several \b translate commands may be specified.
 *       The \b translate command is always executed after the \b center command, regardless
 *       of the order of the commands.
 *  - \b rotate: specifies a rotation around the x-, y- and z-axis, in this order. The given
 *       angles are Euler angles in radian measure. In order to change the order of rotations,
 *       several rotations can be specified.
 *  - \b fixed: fixes the global position of the cylinder within the global world frame.
 *  - \b invisible: if set, the cylinder will be invisible and will not be shown in any
 *       visualization.
 *  - \b texture: specifies the POV-Ray texture of the cylinder. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section plane Plane primitives
 *
 * The \a plane command will create a single plain with the specified normal and the specified
 * distance/displacement from the origin of the global world frame. A plane section requires at
 * least values for the user-specific ID, the normal and the displacement.
 *
 * \image html plane.png
 * \image latex plane.eps "Plane geometry" width=200pt

   \code
   plane {
      id x                                // User-specific ID of the plane
      label L                             // Unique label
      normal <a,b,c>                      // Normal of the plane
      displacement d                      // Distance to the origin of the global frame
      material name/{ ... }               // Material of the plane
      translate <dx,dy,dz>                // Translation of the plane
      rotate <xangle,yangle,zangle>       // Euler rotation of the plane
      invisible                           // Makes the plane invisible
      texture {                           // Specifies the POV-Ray appearance of the
         ...                              //   plane. For details about the texture
      }                                   //   options see the descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the plane.
 *  - \b normal: the normal of the plane pointing to the outside of the plane.
 *  - \b displacement: the distance to the origin of the global world frame.
 *  - \b material: the material of the plane. It is possible to either specify the name of an
 *       existing material or to specify a new material. For detailed information about materials,
 *       see section \ref material .
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs.
 *  - \b translate: translates the plane by the displacement vector <dx,dy,dz>. For each plane,
 *       several \b translate commands may be specified.
 *  - \b rotate: specifies a rotation around the x-, y- and z-axis, in this order. The given
 *       angles are Euler angles in radian measure. In order to change the order of rotations,
 *       several rotations can be specified.
 *  - \b invisible: if set, the plane will be invisible and will not be shown in any
 *       visualization.
 *  - \b texture: specifies the POV-Ray texture of the plane. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section tetrasphere Tetrasphere unions
 *
 * The setup of a tetrasphere union will create a union containing four spheres that are
 * arranged in a tetrahedral fashion. Each sphere will have the same radius and material.
 * \a center specifies the global position of the center of mass of the entire union. A
 * tetrasphere section requires at least values for the user-specific ID, the center
 * of the tetrasphere and the radius and the material of each of the contained spheres.
 *
 * \image html tetrasphere.png
 * \image latex tetrasphere.eps "Tetrasphere" width=200pt

   \code
   tetrasphere {
      id x                                // User-specific ID of the tetrasphere
      label L                             // Unique label
      center <x,y,z>                      // Global position of the tetrasphere
      radius r                            // Radius of the four spheres of the tetrasphere
      material name/{ ... }               // Material of the tetrasphere
      linear <vx,vy,vz>                   // Sets the linear velocity of the tetrasphere
      angular <ax,ay,az>                  // Sets the angular velocity of the tetrasphere
      translate <dx,dy,dz>                // Translation of the tetrasphere
      rotate <xangle,yangle,zangle>       // Euler rotation of the tetrasphere
      fixed                               // Fixes the tetrasphere's position
      invisible                           // Makes the tetrasphere invisible
      texture {                           // Specifies the POV-Ray appearance of the
         ...                              //   tetrasphere. For details about the texture
      }                                   //   options see the descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the tetrasphere.
 *  - \b center: the global position of the center of mass of the tetrasphere.
 *  - \b radius: the radius of the four contained spheres of the tetrasphere.
 *  - \b material: the material of the tetrasphere. It is possible to either specify the name
 *       of an existing material or to specify a new material. For detailed information about
 *       materials, see section \ref material .
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs.
 *  - \b linear: specifies the linear velocity of the tetrasphere in x-, y- and z-direction.
 *  - \b angular: specifies the angular velocity of the tetrasphere around the x-, y- and z-axis.
 *  - \b translate: translates the tetrasphere from the specified \b center by the displacement
 *       vector <dx,dy,dz>. For each tetrasphere, several \b translate commands may be specified.
 *       The \b translate command is always executed after the \b center command, regardless
 *       of the order of the commands.
 *  - \b rotate: specifies a rotation around the x-, y- and z-axis, in this order. The given
 *       angles are Euler angles in radian measure. The rotation affects all four spheres of
 *       the tetrasphere. In order to change the order of rotations, several rotations can
 *       be specified.
 *  - \b fixed: fixes the global position of the tetrasphere within the global world frame.
 *  - \b invisible: if set, the tetrasphere will be invisible and will not be shown in any
 *       visualization.
 *  - \b texture: specifies the POV-Ray texture of the tetrasphere. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section agglomerate Particle agglomerates
 *
 * The agglomerate keyword start the setup of a random union consisting of a specified number
 * of spheres. Each sphere will have the same \a radius and \a material. The \a threshold value,
 * which has to be in the range \f$ [0..1] \f$, varies the packing density of the resulting union
 * from clustered (0) to branched (1). An agglomerate section requires at least values for the
 * user-specific ID, the center of the agglomerate, the number of contained spheres, and
 * the radius and material of each of the spheres.
 *
 * \image html agglomerate.png
 * \image latex agglomerate.eps "Examples for particle agglomerates" width=200pt

   \code
   agglomerate {
      id x                                // User-specific ID of the agglomerate
      label L                             // Unique label
      center <x,y,z>                      // Global position of the agglomerate
      number N                            // The number of contained spheres
      radius r                            // Radius of the contained spheres
      threshold t                         // Packing density of the agglomerate
      material name/{ ... }               // Material of the agglomerate
      linear <vx,vy,vz>                   // Sets the linear velocity of the agglomerate
      angular <ax,ay,az>                  // Sets the angular velocity of the agglomerate
      translate <dx,dy,dz>                // Translation of the agglomerate
      rotate <xangle,yangle,zangle>       // Euler rotation of the agglomerate
      fixed                               // Fixes the agglomerate's position
      invisible                           // Makes the agglomerate invisible
      texture {                           // Specifies the POV-Ray appearance of the
         ...                              //   agglomerate. For details about the texture
      }                                   //   options see the descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the agglomerate.
 *  - \b center: the global position of the center of mass of the agglomerate.
 *  - \b number: specifies the total number of contained spheres.
 *  - \b radius: the radius of all contained spheres.
 *  - \b threshold: the packing density of the agglomerate. The value must be in the range
 *       \f$ [0..1] \f$, where 0 creates a very clustered and 1 creates a very branched
 *       agglomerate.
 *  - \b material: the material of the agglomerate. It is possible to either specify the name
 *       of an existing material or to specify a new material. For detailed information about
 *       materials, see section \ref material .
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs.
 *  - \b linear: specifies the linear velocity of the agglomerate in x-, y- and z-direction.
 *  - \b angular: specifies the angular velocity of the agglomerate around the x-, y- and z-axis.
 *  - \b translate: translates the the center of mass of the agglomerate by the displacement
 *       vector <dx,dy,dz>. For each agglomerate, several \b translate commands may be specified.
 *       The \b translate command is always executed after the \b center command, regardless
 *       of the order of the commands.
 *  - \b rotate: specifies a rotation around the x-, y- and z-axis, in this order. The given
 *       angles are Euler angles in radian measure. The rotation affects all contained spheres
 *       of the agglomerate. In order to change the order of rotations, several rotations can
 *       be specified.
 *  - \b fixed: fixes the global position of the agglomerate within the global world frame.
 *  - \b invisible: if set, the agglomerate will be invisible and will not be shown in any
 *       visualization.
 *  - \b texture: specifies the POV-Ray texture of the agglomerate. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section union Unions
 *
 * A union represents the most general rigid body to be created by the body reader. A union
 * contains an arbitrary number of spheres, boxes, capsules, cylinders and planes. Between the
 * contained rigid bodies, links can be defined, which will be automatically finished by setting
 * the sections. A union section requires the user-specific ID and at least one contained
 * rigid body (sphere, box, capsule, cylinder or plane). All bodies contained in the union have
 * to be given unique IDs within the union. The same rule applies for the contained links.
 *
 * \image html union1.png
 * \image latex union1.eps "Union examples" width=600pt

   \code
   union {
      id x                                // User-specific ID of the union
      label L                             // Unique label
      center <x,y,z>                      // Moves the union's center of mass to <x,y,z>
      linear <vx,vy,vz>                   // Sets the linear velocity of the union
      angular <ax,ay,az>                  // Sets the angular velocity of the union
      material name/{ ... }               // Sets the material of the entire union
      translate <dx,dy,dz>                // Translation of the union
      rotate <xangle,yangle,zangle>       // Rotates the entire union by the Euler angles
                                          //   'xangle', 'yangle' and 'zangle'
      fixed                               // Fixes the union's position
      invisible                           // Makes the entire union invisible

      random {
         number N                         // Populates the union with 'N' random spheres
         radius r                         //   with radius 'r' and density 'd'. 't' specifies,
         density d                        //   how dense the random spheres are packed
         threshold t                      //   together.
      }
      sphere {
         id x                             // Creates the single sphere 'x' with radius 'r'
         label L                          //   and label L
         position <x,y,z>                 //   at the global position <x,y,z> within the
         radius r                         //   union.
         material name/{ ... }            //   Optionally, the sphere's material can be set
         translate <dx,dy,dz>             //   individually and the sphere can be translated
         rotate <xangle,yangle,zangle>    //   and rotated within the union. If the union is
         invisible                        //   not invisible the sphere can be made invisible.
         texture { ... }                  //   Additionally, the texture can be set individually.
      }
      box {
         id x                             // Creates the single box 'x' with the side lengths
         label L                          //   <lx,ly,lz> and label L
         position <x,y,z>                 //   at the global position <x,y,z> within
         lengths <lx,ly,lz>               //   the union.
         material name/{ ... }            //   Optionally, the box's material can be set
         translate <dx,dy,dz>             //   individually and the box can be translated and
         rotate <xangle,yangle,zangle>    //   rotated within the union. If the union is not
         invisible                        //   invisible the box can be made invisible.
         texture { ... }                  //   Additionally, the texture can be set individually.
      }
      capsule {
         id x                             // Creates the single capsule 'x' with the radius 'r',
         label L                          //   label L
         position <x,y,z>                 //   and the cylinder length 'l' at position <x,y,z>
         radius r                         //   within the union.
         length l                         //   Optionally, the capsule's material can be set
         material name/{ ... }            //   individually and the capsule can be translated
         translate <dx,dy,dz>             //   and rotated within the union. If the union is
         rotate <xangle,yangle,zangle>    //   not invisible the capsule can be made invisible.
         invisible                        //   Additionally, the texture can be set individually.
         texture { ... }                  //
      }
      cylinder {
         id x                             // Creates the single cylinder 'x' with the radius 'r',
         label L                          //   label L
         position <x,y,z>                 //   and the cylinder length 'l' at position <x,y,z>
         radius r                         //   within the union.
         length l                         //   Optionally, the cylinder's material can be set
         material name/{ ... }            //   individually and the cylinder can be translated
         translate <dx,dy,dz>             //   and rotated within the union. If the union is
         rotate <xangle,yangle,zangle>    //   not invisible the cylinder can be made invisible.
         invisible                        //   Additionally, the texture can be set individually.
         texture { ... }                  //
      }
      plane {
         id x                             // Creates the single plane 'x' with label L
         label L                          //   and the specified
         normal <a,b,c>                   //   normal and the distance 'd' from the origin of
         displacement d                   //   the global world frame. Optionally, the plane's
         material name/{ ... }            //   material can be set individually and the plane
         translate <dx,dy,dz>             //   can be translated and rotated within the union.
         rotate <xangle,yangle,zangle>    //   If the union is not invisible the plane can be
         invisible                        //   made invisible.
         texture { ... }                  //   Additionally, the texture can be set individually.
      }
      link {
         id x                             // Creates the link 'x' between the bodies with
         body1 L1                         //   label 'L1' and 'L2', which have to be
         body2 L2                         //   already specified.
      }
      texture {                           // Specifies the POV-Ray appearance of all contained
         ...                              //   bodies that have no individual texture. For
      }                                   //   details about the texture options see the
                                          //   descriptions below.
   }
   \endcode

 * Necessary parameters:
 *  - \b id: the user-specific ID of the union.
 *  - at least one contained rigid body has to be specified
 *
 * Optional parameters:
 *  - \b label: specifies a unique label which can be referenced e.g. by springs or links.
 *  - \b center: Places the union at the global coordinates <x,y,z>. The \a center command may
 *       change the position of all bodies within the union depending on their relative distance
 *       to the center of the union.
 *  - \b linear: Sets the linear velocity of the entire union. Since the entire rigid body can
 *       have only one linear velocity, this command affects both the body commands before and
 *       after the \b linear command.
 *  - \b angular: Sets the angular velocity of the entire union. Since the entire rigid body can
 *       have only one angular velocity, this command affects both the body commands before and
 *       after the \b angular command.
 *  - \b material: Sets the material of the union. All specified bodies with the union that are
 *       not assigned an individual material will be given this material. However, every rigid
 *       body requires a material, either explicitly within the definition of the body or
 *       implicitly within the definition of the union.
 *  - \b translate: translates the entire union from the specified \b center by the displacement
 *       vector <dx,dy,dz>. For each union, several \b translate commands may be specified. The
 *       \b translate command is always executed after the \b center command, regardless of the
 *       order of the commands.
 *  - \b rotate: Rotates all rigid bodies of the union around the x-, y- and z-axis according to
 *       the Euler angles <xangle,yangle,zangle>. The \b rotate command affects all bodies of
 *       the union, even those specified after the \b rotate command. The order of rotations is
 *       first around the x-axis, then the y-axis and last around the z-axis. All angles are
 *       given in radian degrees. In order to change the order of rotations, several rotations
 *       can be specified.
 *  - \b fixed : Fixes the union's global position to its current location. The \a fixed command
 *       may appear before or after the \a center command.
 *  - \b invisible: If set, the entire union will be invisible and will not be shown in any
 *       visualization. If the \a invisible command is specified for the union, all \a invisible
 *       commands for the contained rigid bodies are neglected.
 *  - \b random: The union is populated with \a N random spheres of radius \a r and density
 *       \a d. The spheres are given IDs from 1 to N. The \a t parameter has to be in the range
 *       \f$ [0..1] \f$ and varies the shape of the particle from clustered (0) to branched (1).
 *  - \b sphere: Creates a sphere with the user-specific ID \a x and radius \a r. The sphere is
 *       placed at the global coordinates <x,y,z>. In case the sphere is not explicitly assigned
 *       a material, it implicitly uses the material of the union. Optionally, the sphere can be
 *       translated and rotated within the union. If the union is not set to be invisible, the
 *       \a invisible command can be used to make a specific sphere invisible.
 *  - \b box: Creates a box with the user-specific ID \a x, and the side lenghts <lx,ly,lz>.
 *       The box is placed at the global coordinates <x,y,z>. In case the box is not explicitly
 *       assigned a material, it implicitly uses the material of the union. Optionally, the box
 *       can be translated and rotated within the union. If the union is not set to be invisible,
 *       the \a invisible command can be used to make a specific box invisible.
 *  - \b capsule: Creates a capsule with the user-specific ID \a x, the radius \a r, and the
 *       cylinder length \a l. The capsule is placed at the global coordinates <x,y,z>. In case
 *       the capsule is not explicitly assigned a material, it implicitly uses the material of
 *       the union. If the union is not set to be invisible, the \a invisible command can be used
 *       to make a specific capsule invisible.
 *  - \b cylinder: Creates a cylinder with the user-specific ID \a x, the radius \a r, and the
 *       length \a l. The cylinder is placed at the global coordinates <x,y,z>. In case the
 *       cylinder is not explicitly assigned a material, it implicitly uses the material of
 *       the union. If the union is not set to be invisible, the \a invisible command can be used
 *       to make a specific cylinder invisible.
 *  - \b plane: Creates a plane with the user-specific ID \a x, the normal \a n and the
 *       distance/displacement from the origin of the global world frame \a d. In case the plane
 *       is not explicitly assigned a material, it implicitly uses the material of the union.
 *       Optionally, the plane can be translated and rotated within the union around the origin
 *       of the global frame. If the union is not set to be invisible, the \a invisible command
 *       can be used to make a specific plane invisible.\n
 *       Creating a plane within a union makes the union infinite. The center of mass of the
 *       union will be at the origin of the global world frame. Note that it is currently not
 *       possible to create a plane inside a union in MPI parallel simulations!
 *  - \b link: Creates a link with ID \a x between body \a body1 and body \a body2. \a body1 and
 *       \a body2 have to be the IDs of previously defined rigid bodies (either spheres, boxes,
 *       capsules, or cylinders) of the same union. The first \a sphere, \a box, \a capsule, or
 *       \a cylinder command specifies the first body of the link, the second command specifies
 *       the second body.
 *  - \b texture: specifies the POV-Ray texture of the union. For details about the texture
 *       options see the descriptions below.\n\n\n
 *
 *
 * \section spring Springs
 *
 * The \a spring command starts a parameter section for a single spring-damper system between two
 * rigid bodies. A spring requires at least the user-specific IDs of the two attached bodies and
 * values for the spring stiffness and damping factor:

   \code
   spring {
      body1 b1                              // Creates a spring between the two rigid bodies
      body2 b2                              //   with labels 'b1' and 'b2' at the two body relative
      anchor1 a1                            //   anchor points 'a1' and 'a2'. The stiffness of
      anchor2 a2                            //   the spring is set to 's', the damping of the
      stiffness s                           //   spring is 'd'. Optionally, the rest length
      damping d                             //   of the spring can be specified by by the
      length l                              //   'length' keyword or the spring can be made
      invisible                             //   invisible via the 'invisible' keyword.
   }
   \endcode

 * Necessary parameters:
 *  - \b body1: The label of a previously defined rigid body.
 *  - \b body2: The label of a previously defined rigid body.
 *  - \b stiffness: Sets the stiffness of the spring. The stiffness value has to be in the range
 *       \f$ (0..\infty) \f$.
 *  - \b damping: Sets the damping factor of the spring-damper system. The damping value has to
 *       be in the range \f$ (0..\infty) \f$.
 *
 * Optional parameters:
 *  - \b anchor1: Specifies the first anchor point in the body frame of the first attached rigid
 *       body. In case the anchor point is not specified, the default anchor point (0,0,0) is used.
 *  - \b anchor2: Specifies the second anchor point in the body frame of the second attached rigid
 *       body. In case the anchor point is not specified, the default anchor point (0,0,0) is used.
 *  - \b length: Specifies the rest length of the spring. If the current distance between the
 *       anchor points is shorter or longer than the given rest length, the spring is initially
 *       stressed. The rest length has to be in the range \f$ (0..\infty) \f$.
 *  - \b invisible: if set, the spring will be invisible and will not be shown in any
 *       visualization.
 *
 * The following code demonstrates the generation of two example springs:

   \code
   // Generating a spring between the body with label 'l1' and the body with label 'l2'
   spring {
      body1 l1
      body2 l2
      stiffness 100
      damping 10
   }

   // Generating a spring between the body with label 'l3' and the body with label 'l4'
   // The anchor point for the union is specified explicitly, the anchor point for the capsule is
   // its center of mass.
   spring {
      body1 l3
      body2 l4
      anchor1 <-3,2,4>
      stiffness 10.5
      damping 8.23
   }
   \endcode

 * \b Note: It is currently impossible to create spring-damper systems in MPI parallel simulations
 * (i.e., in simulations with more than one MPI process).\n\n
 *
 *
 * \section material Rigid body materials
 *
 * Materials specify the mass and collision properties of rigid bodies. Depending on the materials
 * of two rigid bodies involved in a collision, the bodies behave differently. The \b pe engine
 * offers a list of predefined materials that can be used without further definitions:
 *
 *  - iron
 *  - copper
 *  - granite
 *  - oak
 *  - fir
 *
 * The material is specified by the keyword \a material and a material identifier. The following
 * example shows the definition of an iron sphere:

   \code
   sphere {
      id 1
      center <2,5,-3>
      radius 1
      material iron
   }
   \endcode

 * Next to the predefined materials it is also possible to create custom materials. Each material
 * requires a unique material identifier (it is not possible to use the same identifier twice),
 * a density value and a couple of collision-specific material properties as for instance the
 * coefficient of restitution.

   \code
   material {
      name glass         // The unique identifier of the material
      density 2.201      // The density of the material
      restitution 0.5    // The coefficient of restitution of the material
      static 0.05        // The coefficient of static friction of the material
      dynamic 0.03       // The coefficient of dynamic friction of the material
      poisson 0.22       // The Poisson's ratio of the material
      young 70           // The Young's modulus of the material
      stiffness 1000     // The stiffness in normal direction of the material's contact region (value not physically accurate)
      dampingn 0         // The damping coefficient in normal direction of the material's contact region (value not physically accurate)
      dampingt 0         // The damping coefficient in tangential direction of the material's contact region (value not physically accurate)
   }
   \endcode

 * Necessary parameters:
 *  - \b name: the unique identifier for the material. This name can be used to assign the
 *       material to the rigid bodies. It is not possible to use the same identifier twice.
 *  - \b density: the density of the material \f$ (0..\infty) \f$ (\f$ \frac{kg}{dm^3} \f$).
 *  - \b restitution: the coefficient of restitution of the material \f$ [0..1] \f$.
 *  - \b static: the coefficient of static friction of the material \f$ [0..\infty) \f$.
 *  - \b dynamic: the coefficient of dynamic friction of the material \f$ [0..\infty) \f$.
 *  - \b poisson: the Poisson's ratio of the material \f$ [-1..0.5] \f$ (\f$ GPa \f$).
 *  - \b young: the Young's modulus of the material \f$ (0..\infty) \f$.
 *  - \b stiffness: the stiffness in normal direction of the material's contact region \f$ (0..\infty) \f$.
 *  - \b dampingn: the damping coefficient in normal direction of the material's contact region \f$ [0..\infty) \f$.
 *  - \b dampingt: the damping coefficient in tangential direction of the material's contact region \f$ [0..\infty) \f$.\n\n\n
 *
 *
 * \section povray POV-Ray visualization
 *
 * The body reader can also extract POV-Ray scene settings and the POV-Ray appearance of the
 * rigid bodies in order to set up a POV-Ray visualization. The POV-Ray commands used in the pe
 * physics engine are very similar to the standard POV-Ray commands. However, only a fraction of
 * the original POV-Ray commands is available in pe. In the following, all POV-Ray commands will
 * be explained. The differences between the original POV-Ray commands and the commands used in
 * the physics engine will be highlighted especially.\n
 * For the setup of a POV-Ray scene the following commands are available:

   \code
   // Including a header file
   #include "colors.inc"   // Includes the "colors.inc" header in all POV-Ray files

   // Declaring colors, color maps, pigments, finishes and textures
   #declare MyColor    = color rgb <1,0,0>
   #declare MyColorMap = color_map { color_map { [0.5 color White] [0.5 color Red] } }
   #declare MyPigment  = pigment { marble color_map { [0.5 color White] [0.5 color Red] } }
   #declare MyFinish   = finish { ambient 0.1 diffuse 0.6 phong 1.0 phong_size 50.0 reflection 0.05 }
   #declare MyNormal   = normal { ripples 0.9 turbulence 0.4 }
   #declare MyTexture  = texture { use pigment MyPigment use finish MyFinish scale 1.0 }

   // Setting the file name(s) for the POV-Ray visualization
   file "./dir/image%.pov"

   // Setting the start of the POV-Ray visualization
   start 100

   // Setting the end of the POV-Ray visualization
   end 900

   // Setting the spacing between two POV-Ray files
   spacing 10

   // Setting the background color of the visualization files
   background { color rgb <1,1,1> }

   // Setting the scene camera
   camera {
      location <0,0,0>  // Sets the location of the camera
      look_at <0,1,0>   // Sets the focus point of the camera
      sky <0,0,1>       // Specifies the sky-/up-direction
   }
   \endcode

 * - \b include: Like in POV-Ray, the \a include command includes a POV-Ray header file. The
 *      \a include command will be added to all generated POV-Ray files, so once a file is added
 *      you can use any declared identifier.
 * - \b declare: The \a declare command creates an identifier for a \ref color "color", a
 *      \ref colormap "color map", a \ref pigment "pigment", a \ref finish "finish", a
 *      \ref normal "normal" or a \ref texture "texture". Instead of declaring the entity
 *      several times, the identifier can be used wherever the entity would have been used.
 * - \b file: The given file name specifies both the path and the file names of the POV-Ray files
 *      generated with the \a povray::Writer::writeFile() function. Since an arbitrary number of
 *      files could be created, the given file name is used as a template to determine the file
 *      names. The given string may not exceed 100 charecters and has to contain exactly one '%'
 *      sign, which will be replaced by the current number of the POV-Ray image. For instance,
 *      the file name "image%.pov" is used to create the POV-Ray images "image0.pov", "image1.pov",
 *      ... The default file name that is used if no other valid file name is specified is
 *      "image%.pov".
 * - \b start: The \a start command specifies the number of the first visualized time step.
 *      The value may be any integer in the range \f$ [0..\infty) \f$. The default start is 0,
 *      which corresponds to an immediate start of the POV-Ray visualization.
 * - \b end: The \a end command specifies the number of the last visualized time step. The
 *      value may be any integer in the range \f$ [0..\infty) \f$. The default end is \f$ \inf \f$,
 *      which keeps the visualization active for the duration of the simulation.
 * - \b spacing: The \a spacing command specifies the number of time steps between two POV-Ray
 *      files. The value may be any integer in the range \f$ [1..\infty) \f$. The default spacing
 *      is 1, which corresponds to a POV-Ray file in every time step.
 * - \b background: This command sets the color of the background in all generated POV-Ray files.
 *      For an explanation of the color syntax, see the \ref color "color" section below. The
 *      default background color is a solid white.
 * - \b camera: This sets the camera for all generated POV-Ray files. As in POV-Ray, the location,
 *      the camera focus point and the sky-/up-direction can be specified. The \a location command
 *      places the camera at the specified global coordinate, the \a look_at command focus the
 *      the camera at the given global coordinate. The \a sky command specifies the sky-/up-
 *      direction of the camera. The default values for \a location, \a look_at and \a sky are
 *      <0,0,0>, <0,1,0> and <0,0,1> respectively. Note, that all vectors and coordinates use
 *      the right-handed pe coordinate system, not the left-handed POV-Ray system!\n\n\n
 *
 *
 * \subsection color POV-Ray colors
 *
 * The setup of colors is done exactly as in POV-Ray. The following examples show you several ways
 * to create colors:

   \code
   color rgb  <1,1,1>       // Creates a rgb color with the three color channels red, green and blue.
   color rgbf <1,1,0,0.5>   // Creates a rgb color with an additional transparency of 0.5.
   color White              // Uses the declared POV-Ray color 'White' from the "colors.inc" header
   \endcode

 * The first style creates a solid color value from the three color channels red, green and blue.
 * All three values have to be in the range \f$ [0..1] \f$. The second style additionally specifies
 * the transparency of the color. The transparency value also has to be in the range \f$ [0..1] \f$.
 * A value of 0 makes the color solid, a value of 1 creates a fully transparent color.\n\n\n
 *
 *
 * \subsection colormap POV-Ray color maps
 *
 * In the \b pe physics engine, color maps are created exactly as in POV-Ray itself. The format of
 * the color map parameters is as following:

   \code
   color_map {
      [value1 color1]
      [value2 color2]
      [value3 color3]
   }
   \endcode

 * The value parameters are in the range \f$ [0..1] \f$ and specify the location of the control
 * points in the color map. For each control point, a color is specified. Between the control
 * points, linear interpolation is used to calculate the color value (this also includes the
 * transparency). In order to specify a color map also note the following rules:
 *
 * - The number of control points has to be in the range \f$ [2..20] \f$.
 * - The value of a control point has to be greater or equal to the previous ones.
 * - If the first control point is greater than 0, all the colors between zero and that control
 *   point will be that color.
 * - Similarly, if the last control point is less than 1.0, all the colors between that control
 *   point and 1.0 will be that color.
 * - If two colors are specified to have the same control point, there will be a sudden change
 *   between colors at that point. The color at that point will be the color that is specified
 *   latest.
 * - If two control points have colors with different filter values, the filter values will be
 *   interpolated too, producing colors with intermediate transparency.
 * - The color map specification is case sensitive: the \a color keyword has to be used in lower
 *   case characters.
 *
 * Instead of specifying a color pattern, the \a random keyword can used to create a random
 * color map consisting of 2 to 5 random colors. Additionally it is possible to specify a
 * predefined POV-Ray color map. This color map identifier is a single string and has to match
 * a declared POV-Ray color map exactly. Invalid identifiers will only be detected by POV-Ray
 * and not by the \b pe physics engine.
 *
 * Here are some examples of color maps:

   \code
   // Color blend from Red to green to blue and back to red
   color_map {
      [0.0 color Red]
      [0.33 color Green]
      [0.67 color Blue]
      [1.0 color Red]
   }

   // A red/white stripped appearance
   color_map {
      [0.0 color Red]
      [0.5 color Red]
      [0.5 color White]
      [1.0 color White]
   }

   // The same red/white stripped appearance, but shorter
   color_map {
      [0.5 color Red]
      [0.5 color White]
   }

   // A random color map consisting of 2 to 5 random colors
   color_map {
      random
   }
   \endcode

 * \n\n \subsection imagemap POV-Ray image maps
 *
 * Whereas color maps only represent a color pattern, image maps offer the possibility to map
 * images on rigid bodies in a POV-Ray visualization. For an explanation of image maps see the
 * details in the ImageMap class descriptions. In the pe physics engine, image maps are created
 * exactly as in POV-Ray itself. The format of the image map parameters is as following:

   \code
   image_map {
      gif | tga | iff
      "image_name"
      [map_type type]
      [once]
   }
   \endcode

 * The first parameter specifies the type of the image file. Image maps can be created from
 * gif-, tga- or iff-files. After the image type, the file name of the image has to be specified.
 * Note the quotations around the file name. Optionally, the mapping type can be specified.
 * The default mapping method (planar) is to project the image onto the xz-plane in such a
 * way that it fits into the 2D-square (0,0,0), (1,0,1). The pixels of the original image
 * are turned into infinitely long boxes that are parallel to the y-axis. Next to the default
 * planar mapping, spherical mapping can be used to project the image onto spheres. For further
 * informations about the different mapping types, see pe::MappingType. As second additional
 * parameter, the keyword \a once may be specified. By default, the image is repeated infinitely
 * in the x- and z-directions. If \a once is specified, the image appears only once and is not
 * repeated. In case of spherical mapping, the \a once keyword has no effect.
 *
 * Here are some examples of image maps:

   \code
   // Image map for a grass ground texture
   image_map {
      gif
      "grass.gif"
      map_type planar
   }

   // Image map for a soccer ball
   image_map {
      gif
      "soccerball.gif"
      map_type sperical
   }
   \endcode

 * \n\n \subsection lightsource Light sources
 *
 * The \a light_source command adds a light source to the POV-Ray scene that is included in
 * each generated POV-Ray file. There are several ways to illuminate a POV-Ray scene. The most
 * basic light source is a point light that emits light of a specified color uniformly in all
 * directions. In contrast to point lights, spotlights are directed light sources. For far
 * away light sources, like for example sunlight, a parallel light source can be used. Area
 * lights consist of several point light and enable soft shadows.\n
 *
 * \subsubsection point_light Point light sources
 *
 * Point light sources consist of a single point of light, emitting light of the specified
 * color uniformly in all directions. The following illustration gives an impression of a
 * point light source:
 *
 * \image html pointlight.png
 * \image latex pointlight.eps "Example for a point light source" width=200pt
 *
 * The syntax for a point like looks like this:

   \code
   light_source {
      gpos   // The global position of the center of the light source
      color  // The color of the emitted light
      ...    // Optional point light source modifiers
   }
   \endcode

 * All light sources are per default point lights if no special keyword is used (as for instance
 * \a parallel or \a area_light), i.e. point light sources don't require an additional keyword.
 * The \a fade_distance and \a fade_power of a point light can be individually specified.
 * Additionally, the \a shadowless modifier can be used to create a light source that casts no
 * shadows. For more information about point light sources, see the PointLight class description.
 *
 * Example:

   \code
   light_source {
      <4,-15,-6>               // Global position of the point light
      color rgb <0.8,0.8,0.8>  // Color of the point light source
      fade_distance 21.3       // Optional light fading distance
      fade_power 2.0           // Optional light fading power
      shadowless               // Optional shadowless keyword
   }
   \endcode

 * \subsubsection spotlight Spotlight sources
 *
 * Spotlights offer the possibility to specify directed light sources. Spotlights can be used
 * to create a cone of light that is bright in the center and falls of to darkness in a soft
 * fringe effect at the edge. The following illustration shows how a spotlight works:
 *
 * \image html spotlight.png
 * \image latex spotlight.eps "Example for a spotlight" width=200pt
 *
 * The syntax for a spotlight has the following form:

   \code
   light_source {
      gpos       // The global position of the center of the light source
      color      // The color of the emitted light
      spotlight  // Necessary keyword to introduce a spotlight
      ...        // Optional spotlight modifiers
   }
   \endcode

 * In order to specify the focus point of a spotlight, the \a point_at command can be used.
 * For the configuration of the light cone(s) emitted by the spotlight, the light modifiers
 * \a falloff, \a radius and \a tightness can be used (see the SpotLight class description for
 * more details). The \a fade_distance and \a fade_power of a spotlight can be individually set.
 * Additionally, the \a shadowless modifier can be used to create a light source that casts no
 * shadows.
 *
 * Example:

   \code
   light_source {
      <2,2,20>
      color rgb <0.8,0.8,0.8>
      spotlight
      point_at <2,2,0>
      falloff 0.75
      radius 0.5
      tightness 2
      fade_distance 21.3
      fade_power 2.0
      shadowless
   }
   \endcode

 * \subsubsection parallel_light Parallel light sources
 *
 * Parallel lights are useful for simulating very distant light sources, such as sunlight. A
 * parallel light source emits parallel light from a plane determined by the global position
 * of the light source and an additional focus point specified by the \a point_at keyword. If
 * no focus point is specified explicitly, the focus point is set to (0,0,0). The following
 * illustration gives an impression of the lighting effects of a parallel light source:
 *
 * \image html parallellight.png
 * \image latex parallellight.eps "Example for a parallel light source" width=200pt
 *
 * The syntax for parallel light sources is as follows:

   \code
   light_source {
      gpos      // The global position of the center of the light source
      color     // The color of the emitted light
      parallel  // Necessary keyword to introduce a parallel light source
      ...       // Optional parallel light source modifiers
   }
   \endcode

 * Valid optional light modifiers for a parallel light source are the \a point_at command to
 * specify the focus point, the \a fade_distance and the \a fade_power command to configure
 * the light fading and the \a shadowless command to create a light source that casts no shadows.
 * For more details about parallel lights, take a look at the ParallelLight class description.
 *
 * Example:

   \code
   light_source {
      <2,2,20>
      color rgb <0.8,0.8,0.8>
      parallel
      point_at <2,2,0>
      fade_distance 21.3
      fade_power 2.0
      shadowless
   }
   \endcode

 * \subsubsection area_light Area light sources
 *
 * Area light sources are represented by a one- or two-dimensional array of individual point
 * lights. Each single point light source of the area light emits light of the specified color
 * uniformly in all directions. In constrast to a single point light source, area lights are able
 * to cast soft shadows because an object can partially block their light (whereas a single point
 * light source is either totally blocked or not blocked). The following images give an impression
 * of an area light source:
 *
 * \image html arealight.png
 * \image latex arealight.eps "Example for an area light source" width=200pt
 *
 * The syntax for area lights looks like this:

   \code
   light_source {
      gpos        // The global position of the center of the light source
      color       // The color of the emitted light
      area_light  // Necessary keyword to introduce an area light source
      axis1       // First axis that spans the area light
      axis2       // Second axis that spans the area light
      size1       // Number of point light sources along the first axis
      size2       // Number of point light sources along the second axis
      ...         // Optional area light source modifiers
   }
   \endcode

 * Note that in contrast to POV-Ray after the axes and numbers of point lights no commas are (and
 * even may not be) used! Area lights can be modified by the \a adaptive command to set the level
 * of adaptive sampling, the \a fade_distance and \a fade_power command to configure the light
 * fading and the \a shadowless command to create a light source that casts no shadows. For more
 * details to area light sources, see the AreaLight class description.
 *
 * Example:

   \code
   light_source {
      <20,30,40>
      color rgb <0.7,0.7,0.1>
      area_light
      <1,0,0> <0,1,0> 5 5
      adaptive 2
      fade_distance 21.3
      fade_power 2.0
      shadowless
   }
   \endcode

 * \n\n \subsection pigment Pigments
 *
 * A pigment declaration in the \b pe physics engine has the following form:

   \code
   pigment {
      // First part: type of the pigment (exactly one of these is required)
      color ...                       // OR
      pigment_type color_map { ... }  // OR
      image_map { ... }               // OR
      use ...

      // Second part: optional pigment modifiers
      frequency ...
      lambda ...
      octaves ...
      omega ...
      phase ...
      turbulence ...

      // Third part: optional transformations
      scale ...
      translate ...
      rotate ...
   }
   \endcode

 * A pigment can consist of three major parts: The pigment type (that specifies the basic color
 * pattern of the pigment), the pigment modifiers (see to the \ref modifier section) and optional
 * transformations (see the \ref transformation section). In the following, all possible pigment
 * types are described in detail.
 *
 * \subsubsection color_pigment Single colored pigments
 *
 * Single-colored pigments are the simplest possible pigments: they consist of a single, fixed
 * color for the entire body. The following images give an idea of what single-colored pigments
 * can look like:
 *
 * \image html singlecolor.png
 * \image latex singlecolor.eps "Examples for single-colored pigments" width=600pt
 *
 * Neither modifiers nor transformations can be used for a single color pigment, since neither
 * of them has any effect on the color pattern of the single colored pigment. For more details,
 * see the ColorPigment class description.
 *
 * Examples:

   \code
   pigment {
      color rgb <1,1,0>
   }

   pigment {
      color rgbf <0.4,0.3,0.7,0.5>
   }

   pigment {
      color Blue  // declared in the "colors.inc" header file
   }
   \endcode

 * \subsubsection agate_pigment Agate pigments
 *
 * Agate pigments provide a very swirly, very turbulent color pattern. The following images give
 * an impression of agate pigments. The first image shows a plain agate pigment without further
 * modifiers. The second image demonstrates an agate pigment with additional turbulence and the
 * third image additionally uses the Omega and Octaves modifier to control the turbulence.
 *
 * \image html agatePigment.png
 * \image latex agatePigment.eps "Examples for agate pigments" width=600pt
 *
 * An agate pigment can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega, \ref octaves) and by the \ref frequency and \ref phase modifiers. Additionally
 * it is possible, to \ref scale, \ref translate and \ref rotate an agate pigment. For more
 * details about agate pigments, see the detailed description of the AgatePigment class.
 *
 * Examples:

   \code
   pigment {
      agate
      color_map { [0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red] }
      turbulence 0.6
      omega 0.4
      octaves 2
   }
   \endcode

 * \subsubsection bozo_pigment Bozo pigments
 *
 * Bozo pigments basically create a series of smooth color splotches on a rigid body. The images
 * show the color pattern of several bozo pigments. The first image shows a plain bozo pigment,
 * whereas the second and third illustration are modified by the Frequency and Turbulence modifier,
 * respectively.
 *
 * \image html bozoPigment.png
 * \image latex bozoPigment.eps "Examples for bozo pigments" width=600pt
 *
 * A bozo pigment can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega, \ref octaves) and by the \ref frequency and \ref phase modifiers. Additionally it
 * is possible, to \ref scale, \ref translate and \ref rotate a bozo pigment. For more details
 * about bozo pigments, see the detailed description of the BozoPigment class.
 *
 * Example:

   \code
   pigment {
      bozo
      color_map { [0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red] }
      turbulence 0.6
      omega 0.5
      frequency 6
   }
   \endcode

 * \subsubsection granite_pigment Granite pigments
 *
 * With the proper color map, granite pigments are able to simulate real granite very convincingly.
 * The following pictures show some examples for granite pigments. Very nice examples for realistic
 * granite pigments can be found in the "stones.inc" POV-Ray header file.
 *
 * \image html granitePigment.png
 * \image latex granitePigment.eps "Examples for granite pigments" width=800pt
 *
 * Granite pigments can be modified by the \ref frequency and \ref phase modifiers as well as by
 * all turbulence modifiers (\ref turbulence, \ref lambda, \ref omega, \ref octaves). Additionally,
 * it is possible to \ref scale, \ref translate and \ref rotate a granite pigment. For more
 * information about granite pigments, see the detailed description of the GranitePigment class.
 *
 * Example:

   \code
   pigment {
      granite
      color_map { [0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red] }
      turbulence 0.6
      omega 0.8
      lambda 2.0
   }
   \endcode

 * \subsubsection marble_pigment Marble pigments
 *
 * Marble pigments map a POV-Ray \ref colormap on an object that can be distorted by a specified
 * level of turbulence. The images demonstrate some possible marble pigments: the first pigment
 * uses no turbulence, the second uses a turbulence value of 0.8 and the third example uses the
 * POV-Ray pigment "T_Stone9" from the "stones.inc" header file.
 *
 * \image html marblePigment.png
 * \image latex marblePigment.eps "Examples for marble pigments" width=600pt
 *
 * A marble pigment can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega, \ref octaves) and by the \ref frequency and \ref phase modifiers. Additionally
 * it is possible, to \ref scale, \ref translate and \ref rotate a marble pigment. For more
 * details about marble pigments, see the detailed description of the MarblePigment class.
 *
 * Example:

   \code
   pigment {
      marble
      color_map { [0.5 color Red][0.5 color White] }
      turbulence 0.8
      omega 0.4
      lambda 1.5
   }
   \endcode

 * \subsubsection radial_pigment Radial pigments
 *
 * Radial pigments wrap a given \ref colormap clockwise around the z-axis, starting in the
 * positive x direction. The following images give an impression of possible radial pigments:
 *
 * \image html radialPigment.png
 * \image latex radialPigment.eps "Examples for radial pigments" width=400pt
 *
 * A radial pigment can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega, \ref octaves) and by the \ref frequency and \ref phase modifiers. Additionally
 * it is possible, to \ref scale, \ref translate and \ref rotate a marble pigment. For more
 * details about marble pigments, see the detailed description of the RadialPigment class.
 *
 * Example:

   \code
   pigment {
      radial
      color_map { [0.0 color Red][0.33 color Yellow][0.66 color Blue][1.0 color Red] }
      frequency 8.0
      phase 0.2
   }
   \endcode

 * \subsubsection spotted_pigment Spotted pigments
 *
 * Spotted pigments use a \ref colormap to create colored spots on the rigid bodies. The images
 * demonstrate several spotted pigments:
 *
 * \image html spottedPigment.png
 * \image latex spottedPigment.eps "Examples for spotted pigments" width=800pt
 *
 * Spotted pigments can only be modified by the \ref frequency and \ref phase modifiers. However,
 * it is also possible to \ref scale, \ref translate and \ref rotate a spotted pigment. For more
 * details, see the SpottedPigment class description.
 *
 * Example:

   \code
   pigment {
      spotted
      color_map { [0.2 color Red][0.45 color Green][0.55 color Yellow][0.8 color Blue] }
      frequency 1.1
   }
   \endcode

 * \subsubsection image_pigment Image pigments
 *
 * Image pigments create a POV-Ray pigment by projecting the given image map onto a rigid body.
 * The images show three possible applications for image pigments. The first and second picture
 * show grass and water textures projected onto a plane, the third demonstrates spherical mapping
 * of a globe image onto a sphere.
 *
 * \image html imagemap.png
 * \image latex imagemap.eps "Examples for image maps" width=550pt
 *
 * Only transformations (\ref scale, \ref translate and \ref rotate) can be used to modify the
 * impression of an image pigment. For more informations, see the ImagePigment class description.
 *
 * Example:

   \code
   pigment {
      image_map { gif "grass.gif" map_type planar }
   }
   \endcode

 * \subsubsection custom_pigment Custom pigments
 *
 * Custom pigments use predefined/declared pigment identifiers. In contrast to POV-Ray, the use
 * of an declared POV-Ray identifier in the \b pe physics engine uses the \a use keyword:

   \code
   pigment {
      use MyPigment
   }
   \endcode

 * In order to modify a custom pigment, the transformations \ref scale, \ref translate and
 * \ref rotate can be used.\n\n\n
 *
 *
 * \subsection finish Finishes
 *
 * A POV-Ray finish is created just like in POV-Ray directly. The following example gives an
 * overview over the possible finish options:

   \code
   finish {
      ambient ...
      diffuse ...
      phong ...
      phong_size ...
      reflection ...
      refraction ...
      roughness ...
      specular ...
   }
   \endcode

 * - \b ambient: The ambient lighting value specifies the luminance of the body itself. The value
 *      has to be in the range \f$ [0..1] \f$. Otherwise a \a std::invalid_argument exception
 *      is thrown. If the ambient lighting is set to 0, the rigid body will appear black if it is
 *      not directly lighted by a light source.
 * - \b diffuse: The diffuse lighting specifies how much light is diffused at the surface of the
 *      rigid body. The value has to be in the range \f$ [0..1] \f$.
 * - \b phong: Setting the phong value creates a highlight on the rigid body that is the color of
 *      the light source. The phong value specifies the saturation of the highlight and has to be
 *      in the range \f$ [0..1] \f$. Phong highlights are similar to specular highlights. However,
 *      specular highlights are more accurate as far as physical laws are concerned. Note that
 *      phong highlights and specular highlights are usually not both used on the same body!
 * - \b phong_size: The phong size value specifies the size of the phong highlight. The phong size
 *      has to be in the range \f$ [0..\infty] \f$.
 * - \b reflection: The reflection finish gives a body a mirrored or partially mirrored surface.
 *      This body will then reflect other bodies in the scene. The value has to be in the range
 *      \f$ [0..1] \f$. A value of 0 turns the reflection off, a value of 1 gives the body an
 *      almost perfectly mirrored surface.
 * - \b refraction: Refraction only has meaning on rigid bodies that have at least a litte bit of
 *      transparency. Refraction is the bending of light rays as they pass into a more dense or
 *      less dense medium. As light does not go through opaque things, they don't refract. Without
 *      refraction, transparent bodies look like colored air. The refraction value has to be in
 *      the range \f$ [1..\infty) \f$. A value of 1.0 is the POV-Ray default and will not change
 *      the refraction of a body. Examples for some physical refraction values are 1.000292 for
 *      air, 1.33 for water or 1.5 for glass.
 * - \b roughness: Roughness can be used to set the size of a specular highlight. The roughness
 *      has to be a value in the range \f$ (0..1] \f$: a smaller roughness value will decrease
 *      the size of the specular highlight, a larger value will increase the size.
 * - \b specular: Adding a specular highlight to a finish creates a highlight on the rigid body
 *      that is the color of the light source. The specular value specifies the saturation of
 *      the highlight and has to be in the range \f$ [0..1] \f$. Specular highlights are similar
 *      to phong highlights. However, specular highlights are more accurate as far as physical
 *      laws are concerned. Note that specular highlights and phong highlights are usually not
 *      both used on the same body!
 *
 * Additionally it is possible to use predefined/declared finish identifiers. A declared finish
 * is created by using the keyword \a use:

   \code
   finish {
      use MyFinish
   }
   \endcode

 * \n\n \subsection normal Normals
 *
 * Normals are created exactly like in POV-Ray: the \a normal keyword starts the declaration of
 * a normal, that has to be direcly followed by a block in braces. The first keyword inside the
 * block has to be the kind of normal followed by a value in the range from 0 to 1, inclusive,
 * for the size of the normal effect. The following code shows the form of a normal definition:

   \code
   normal {
      ...  // Type of the normal and size parameter
      ...  // Additional modifiers
      ...  // Additional transformations
   }
   \endcode

 * \subsubsection agate_normal Agate normals
 *
 * The first available kind of normals is the agate normal. It creates an impressive, 3-dimensional
 * pattern of turbulent valleys:
 *
 * \image html agateNormal.png
 * \image latex agateNormal.eps "POV-Ray agate normal" width=400pt
 *
 * The agate pattern can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega and \ref octaves), the \ref frequency and \ref phase modifier and all available
 * transformations (\ref scale, \ref translate and \ref rotate). For more details about the agate
 * normal, see the AgateNormal class description.
 *
 * Example:

   \code
   normal {
      agate 0.9
      turbulence 0.6
      octaves 6
      omega 0.5
      lambda 2.0
      frequency 2
      phase 0.3
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection bozo_normal Bozo normals
 *
 * The second kind of normals is the bozo normal. Bozo normals create a random, bumpy pattern
 * similar to the bumps normal (see the \ref bumps_normal section). However, the effect of the
 * bozo normal is weaker and softer.
 *
 * \image html bozoNormal.png
 * \image latex bozoNormal.eps "POV-Ray bozo normal" width=400pt
 *
 * The bozo effect can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega and \ref octaves), the \a frequency and the \ref phase modifier and all available
 * transformations (\ref scale, \ref translate and \ref rotate). For more details, see the Bumps
 * class description.
 *
 * Example:

   \code
   normal {
      bozo 0.6
      turbulence 0.4
      octaves 5
      omega 0.6
      lambda 2.0
      frequency 3
      phase 0.1
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection bumps_normal Bumps normals
 *
 * A normal type similar to the bozo normal but with a stronger effect is the bumps normal. It
 * adds a random bumpy pattern to the texture:
 *
 * \image html bumps.png
 * \image latex bumps.eps "POV-Ray bumps normal" width=400pt
 *
 * The bumps pattern can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega and \ref octaves) and all available transformations (\ref scale, \ref translate and
 * \ref rotate). For more details, see the Bumps class description.
 *
 * Example:

   \code
   normal {
      bumps 0.9
      turbulence 0.6
      octaves 6
      omega 0.5
      lambda 2.0
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection dents_normal Dents normals
 *
 * Dents normals make an object look like someone attacked it with a sledgehammer:
 *
 * \image html dents.png
 * \image latex dents.eps "POV-Ray dents normal" width=400pt
 *
 * The effect can be modified by all turbulence modifiers (\ref turbulence, \ref lambda, \ref omega
 * and \ref octaves) and all transformations (\ref scale, \ref translate and \ref rotate). For a
 * more detailed description, see the Dents class description.
 *
 * Example:

   \code
   normal {
      dents 0.6
      turbulence 0.6
      octaves 6
      omega 0.5
      lambda 2.0
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection granite_normal Granite normals
 *
 * Granite normals create a strong, rocky normal effect that closly resembles a rough stone
 * formation. For an example of a granite normal, see the illustration below:
 *
 * \image html graniteNormal.png
 * \image latex graniteNormal.eps "POV-Ray granite normal" width=400pt
 *
 * The normal effect of a granite normal can be tuned by all turbulence modifiers (\ref turbulence,
 * \ref lambda, \ref omega and \ref octaves), the \ref frequency and \a phase modifier and all
 * transformations (\ref scale, \ref translate and \ref rotate).
 *
 * Example:

   \code
   normal {
      granite 1.0
      turbulence 0.2
      octaves 3
      omega 0.2
      lambda 2.1
      frequency 1
      phase 0.1
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection marble_normal Marble normals
 *
 * Marble normals offer a strong impression of a rough surface as for example stones or wood. Note
 * however that this effect is only achieved by at least a small amount of turbulence. Otherwise,
 * the effect may look boring due to the regular, striped structure. An example for a marble normal
 * is illustrated in the image below:
 *
 * \image html marbleNormal.png
 * \image latex marbleNormal.eps "POV-Ray marble normal" width=400pt
 *
 * The normal effect of a granite normal can be tuned by all turbulence modifiers (\ref turbulence,
 * \ref lambda, \ref omega and \ref octaves), the \ref frequency and \a phase modifier and all
 * transformations (\ref scale, \ref translate and \ref rotate).
 *
 * Example:

   \code
   normal {
      marble 0.8
      turbulence 0.3
      octaves 5
      omega 0.2
      lambda 1.9
      frequency 2.3
      phase 0.15
      scale 3.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection ripples_normal Ripples normals
 *
 * A ripples normal creates evenly spaced, smooth ripples on the surface of an object:
 *
 * \image html ripples.png
 * \image latex ripples.eps "POV-Ray ripples normal" width=400pt
 *
 * It can be modified by all turbulence modifiers (\ref turbulence, \ref lambda, \ref omega and
 * \ref octaves), the \ref frequency and \ref phase modifier and all transformations (\ref scale,
 * \ref translate and \ref rotate). For more details, see the Ripples class description.
 *
 * Example:

   \code
   normal {
      ripples 0.7
      frequency 2.3
      phase 0.3
      turbulence 0.6
      octaves 6
      omega 0.5
      lambda 2.0
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection spotted_normal Spotted normals
 *
 * Using a SpottedNormal creates a slight pertubation in the perfectly smooth surface of a rigid
 * body. This bumpy effect comes in very handy in case a nearly smooth surface is required, as for
 * instance old metal. The following image demonstrates the effect of a spotted normal:
 *
 * \image html spottedNormal.png
 * \image latex spottedNormal.eps "POV-Ray spotted normal" width=400pt
 *
 * It can be modified by all turbulence modifiers (\ref turbulence, \ref lambda, \ref omega and
 * \ref octaves), the \ref frequency and \ref phase modifier and all transformations (\ref scale,
 * \ref translate and \ref rotate). For more details, see the SpottedNormal class description.
 *
 * Example:

   \code
   normal {
      spotted 0.9
      frequency 2.1
      phase 0.2
      turbulence 0.8
      octaves 5
      omega 0.4
      lambda 2.1
      scale 2.3
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection waves_normal Waves normals
 *
 * Adding a waves normal to an object creates rough and tumble waves on its surface:
 *
 * \image html waves.png
 * \image latex waves.eps "POV-Ray waves normal" width=400pt
 *
 * In order to modify the waves effect, all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega and \ref octaves), the \ref frequency and \ref phase modifier and all available
 * transformations (\ref scale, \ref translate and \ref rotate) can be used. For more information,
 * see the Waves class description.
 *
 * Example:

   \code
   normal {
      ripples 0.4
      frequency 1.7
      phase 0.1
      turbulence 0.8
      octaves 3
      omega 0.5
      lambda 2.0
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection wrinkles_normal Wrinkles normals
 *
 * This normal basically makes an object look like it had been wadded up and then stretched back
 * out again:
 *
 * \image html wrinkles.png
 * \image latex wrinkles.eps "POV-Ray dents normal" width=400pt
 *
 * The wrinkles effect can be modified by all turbulence modifiers (\ref turbulence, \ref lambda,
 * \ref omega and \ref octaves) and by all available transformations (\ref scale, \ref translate
 * and \ref rotate). For more details see the Wrinkles class description.
 *
 * Example:

   \code
   normal {
      wrinkles 0.4
      turbulence 0.4
      octaves 2
      omega 0.6
      lambda 2.1
      scale 2.0
      translate <1,2,3>
      rotate <-0.1,-0.2,0-3>
   }
   \endcode

 * \subsubsection custom_normal Custom normals
 *
 * Custom normals offer the possibility to use predefined/declared normal identifiers. The example
 * demonstrates the setup of a custom normal by the use of the \a use keyword:

   \code
   pigment {
      use MyNormal
   }
   \endcode

 * In order to modify a custom normal, the transformations \ref scale, \ref translate and
 * \ref rotate can be used.\n\n\n

 * \subsection texture Textures
 *
 * A texture gives a rigid body a specific appearance in the POV-Ray visualization. The basic
 * component of a texture is the pigment that defines the basic color composition (see the
 * \ref pigment module). The pigment can be enhanced by certain finish effects (as for instance
 * glowing effects or reflections, see the \ref finish section) or normal transformations (like
 * bumpy or rippled surfaces, see the \ref normal section).
 *
 * \subsubsection plain_texture Plain textures

   \code
   texture {
      pigment { ... }
      finish { ... }
      normal { ... }
      scale ...
      translate ...
      rotate ...
   }
   \endcode

 * Plain textures are individual combinations of \ref pigment, \ref finish and \ref normal. The
 * following images give an impression of several plain textures:
 *
 * \image html texture.png
 * \image latex texture.eps "Examples for POV-Ray textures" width=600pt
 *
 * Neither a pigment, nor a finish or normal are strictly necessary, however a texture with a
 * pigment will be just black. In order to modify a texture, all transformations (\ref scale,
 * \ref translate and \ref rotate) can be used.
 *
 * Example:

   \code
   texture {
      pigment {
         marble
         color_map { [0.5 color Red][0.5 color White] }
         turbulence 0.5
         omega 0.4
      }
      finish {
         ambient 0.1
         diffuse 0.6
         phong 0.9
         phong_size 50
         reflection 0.05
      }
   }
   \endcode

 * \subsubsection tiled_texture Tiled textures

   \code
   texture {
      tiles {
         texture { ... }
         texture { ... }
      }
      scale ...
      translate ...
      rotate ...
   }
   \endcode

 * A tiled texture consists of two other POV-Ray textures. The following images give an impression
 * of tiled textures:
 *
 * \image html tiled.png
 * \image latex tiled.eps "Examples for POV-Ray textures" width=400pt
 *
 * Tiled textures can be scaled, translated and rotated via the \ref scale, \ref translate and
 * \ref rotate transformations.
 *
 * \subsection layered_texture Layered textures

   \code
   texture { ... }  // The bottom layer
   texture { ... }  // A semi-transparent layer
   texture { ... }  // The top semi-transparent layer
   \endcode

 * Layered textures are the result of several POV-Ray textures that are partially transparent
 * and are laid on top of each other to create more complex textures. The illustration gives an
 * impression of a layered texture:
 *
 * \image html layered.png
 * \image latex layered.eps "Examples for layered POV-Ray textures" width=600pt
 *
 * \subsubsection custom_texture Custom textures
 *
 * It is also possible to use predefined/declared POV-Ray textures. In constrast to POV-Ray, where
 * it is sufficient to just use the identifier, the \b pe physics engine requires the use of the
 * keyword \a use:

   \code
   texture {
      use MyTexture
   }
   \endcode

 * Modifications are possible via the \ref scale, \ref translate and \ref rotate transformations.
 * The following images give an impression of declared POV-Ray textures:
 *
 * \image html declared.png
 * \image latex declared.eps "Examples for POV-Ray textures" width=820pt
 *
 *
 * \n\n \subsection modifier Modifiers
 *
 * Modifiers can be used for both pigments and normals to manipulate the resulting effect. The
 * following is a complete list of all available modifiers. Note that not every modifier can be
 * used in every pigment or normal. A list of valid modifiers is listed individually for every
 * pigment or normal (see the \ref pigment and \ref normal sections).
 *
 * \subsubsection frequency frequency
 *
 * Allowed values: \f$ (0..1] \f$
 *
 * The frequency modifier controls how many times a specific pattern (for instance a color map or
 * ripples) is used over the range from 0 to 1. A value greater than 1 will compress the pattern,
 * a value less than 1 will stretch it. Negative values will reverse the pattern. However, a value
 * of 0 is invalid. The following sample images demonstrate the effect of the frequency modifier.
 * The first image uses the default frequency value of 1, the second uses a frequency value of 5.
 * The third image was rendered using a value of -2, therefore reversing the color scheme.
 *
 * \image html frequency.png
 * \image latex frequency.eps "POV-Ray frequency modifier" width=600pt
 *
 * \subsubsection lambda lambda
 *
 * Allowed values: \f$ (1..\infty) \f$
 *
 * The lambda value controls the choice of the random displacement directions that create the
 * turbulence effect. Lambda can be assigned any value larger than 1. A lambda value close to 1
 * causes the step to be in approximately the same direction. Higher lambda values increase the
 * probability for different directions. The default for the lambda parameter is 2.\n
 * The following pictures give an impression of the effect of the lambda modifier. The first
 * image was rendered using a lambda value of 1.01, creating a very smooth turbulence. The
 * second image uses the default value of 2.0 and the third image a lambda value of 3.0.
 *
 * \image html lambda.png
 * \image latex lambda.eps "POV-Ray lambda modifier" width=600pt
 *
 * \subsubsection octaves octaves
 *
 * Allowed values: \f$ [1..10] \f$
 *
 * The octaves modifier controls the number of semi-random steps taken by the turbulence function
 * when it is generating turbulence. The octaves value can be any integral number between 1 and 10
 * (the default is 6). Due to the exponential decrease of the step size due to the Omega value, a
 * larger number of steps has only marginal effect, whereas a smaller number of steps creates a
 * smoother, wavy kind of turbulence.\n
 * The images show the effect of the octaves modifier on turbulence. Image one shows turbulence
 * with only 2 steps. Image two uses 4 random steps, whereas image three demonstrates the POV-Ray
 * default of 6 steps. The more steps are used during the turbulence calculations, the more caotic
 * the turbulence effect becomes.
 *
 * \image html octaves.png
 * \image latex octaves.eps "POV-Ray octaves modifier" width=600pt
 *
 * \subsubsection omega omega
 *
 * Allowed values: \f$ (0..1) \f$
 *
 * The omega modifier controls the size of the semi-random steps during the creation of the
 * turbulence effect: every subsequent step is omega times as long as the previous step. The
 * default omega value is 0.5. Higher values tend to make the turbulence more random and chaotic,
 * smaller values tend to smooth the pattern.\n
 * The following images give an impression of the effects the omega modifier has on turbulence.
 * The first image shows a turbulence pattern with an omega value of 0.2. The second image uses
 * the POV-Ray default of 0.5, whereas the third image was rendered with omega equal to 0.8.
 *
 * \image html omega.png
 * \image latex omega.eps "POV-Ray omega modifier" width=600pt
 *
 * \subsubsection phase phase
 *
 * Allowed values: \f$ (0..1] \f$
 *
 * The phase modifier can be used to offset a color map or certain normals in the range from 0
 * to 1. Phase values of 0 (the default) and 1 result in no changes of the texture. All value
 * in-between change the phase. For instance, for a RadialPigment a phase modifier results in a
 * rotation of the pigment around the y-axis:
 *
 * \image html phase.png
 * \image latex phase.eps "POV-Ray phase modifier" width=400pt
 *
 * \subsubsection turbulence turbulence
 *
 * Allowed values: \f$ [0..1] \f$
 *
 * The turbulence modifier can be used to distort a specific pattern (for instance pigments or
 * normals) to some extent. The turbulance value has to be in the range 0 to 1 inclusive. A high
 * value corresponds to lots of turbulence, a low value means only a litte turbulence.\n
 * The effect of turbulence is created by a displacement of the original color pixels on the
 * surface of an object. Turbulence works by taking a number of semi-random steps and using the
 * original color at the destination point. The random number generator used to produce these
 * steps is deterministic, so rendering a scene file with turbulence will always result in the
 * same image.
 *
 * \image html turbulence.png
 * \image latex turbulence.eps "POV-Ray turbulence modifier" width=600pt
 *
 *
 * \n\n \subsection transformation Transformations
 *
 * The term "transformation" refers to scaling, translation and rotation operations on textures,
 * pigments and normals. Any number of transformations may be applied to an object, i.e. the
 * \a rotate keyword may appear several times within a single block.
 *
 * \subsubsection scale scale
 *
 * Allowed values: \f$ (-\infty..0) \cup (0..\infty) \f$
 *
 * The default size of a texture, pigment or normal is 1. In order to either stretch or squeeze
 * them, the \a scale transformation can be used. A scale value larger than 1 will stretch the texture, pigment or normal, a value smaller
 * than 1 will squeeze it. A negative value will reverse the texture, pigment or normal.
 * However, a scale value of 0 is invalid.
 *
 * \subsubsection translate translate
 *
 * Allowed values: \f$ (-\infty..\infty) \f$
 *
 * A \a translate transformation changes the location/reference point of a texture, pigment or
 * normal. The order of the arguments doesn't matter (in contrast to the arguments order of a
 * rotation).
 *
 * \subsubsection rotate rotate
 *
 * Allowed values: \f$ (-\infty..\infty) \f$
 *
 * Rotation transformations are applied in the order x, y and z. In order to change the order,
 * several rotations have to be specified. Note that all three angles have to be specified in
 * radian measure.\n
 * The following pictures give an impression of texture rotations. The first image shows an image
 * map of planet earth without any rotations. The second image shows the same image map rotated
 * by \f$ \pi/2 \f$ (radian measure) around the z-axis. In the third image the rotation was
 * increased to \f$ \pi \f$ (radian measure) around the z-axis, whereas the fourth image shows a
 * rotation of \f$ \pi/2 \f$ around the x-axis.
 *
 * \image html rotation.png
 * \image latex rotation.eps "Examples for POV-Ray textures" width=600pt
 */
class PE_PUBLIC BodyReader
{
public:
   //**Private class Error*************************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Container for error messages.
   //
   // Note that this class is defined in a public section of the BodyReader class in order to
   // avoid compilation errors with the Microsoft Visual Studio compilers.
   */
   class Error
   {
    private:
      //**Friend declarations**********************************************************************
      template< typename Type > friend Error& operator<<( Error& error, Type output );
      //*******************************************************************************************

    public:
      //**Constructors*****************************************************************************
      /*!\name Constructors */
      //@{
      Error();
      Error( const Error& error );
      //@}
      //*******************************************************************************************

      //**Destructor*******************************************************************************
      /*!\name Destructor */
      //@{
      ~Error();
      //@}
      //*******************************************************************************************

      //**Copy assignment operator*****************************************************************
      /*!\name Copy assignment operator */
      //@{
      Error& operator=( const Error& error );
      //@}
      //*******************************************************************************************

      //**Conversion operators*********************************************************************
      /*!\name Conversion operators */
      //@{
      inline operator bool() const;
      //@}
      //*******************************************************************************************

      //**Utility functions************************************************************************
      /*!\name Utility functions */
      //@{
      inline bool        hasError() const;
      inline std::string getError() const;
      inline void        clear();
      inline void        append( const Error& error );
      //@}
      //*******************************************************************************************

    private:
      //**Member variables*************************************************************************
      /*!\name Member variables */
      //@{
      bool error_;
      std::ostringstream message_;
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Declarations for nested structures**********************************************************
   struct SphereParameters;
   struct BoxParameters;
   struct CapsuleParameters;
   struct CylinderParameters;
   struct PlaneParameters;
   struct LinkParameters;
   struct AgglomerateParameters;
   struct UnionParameters;
   struct SpringParameters;
   struct EqualSign;
   struct Semicolon;
   struct LeadingBracket;
   struct TrailingBracket;
   //**********************************************************************************************

private:
   //**Type definitions****************************************************************************
   /*! \cond PE_INTERNAL */
   typedef std::vector<Vec3>                Rotations;                //!< Rigid body rotations.
   typedef std::vector<SphereParameters>    SphereParameterVector;    //!< Vector for sphere parameters.
   typedef std::vector<BoxParameters>       BoxParameterVector;       //!< Vector for box parameters.
   typedef std::vector<CapsuleParameters>   CapsuleParameterVector;   //!< Vector for capsule parameters.
   typedef std::vector<CylinderParameters>  CylinderParameterVector;  //!< Vector for cylinder parameters.
   typedef std::vector<PlaneParameters>     PlaneParameterVector;     //!< Vector for plane parameters.
   typedef std::vector<LinkParameters>      LinkParameterVector;      //!< Vector for link parameters.
   typedef std::vector<UnionParameters>     UnionParameterVector;     //!< Vector for union parameters.

   typedef std::stringstream::pos_type      sstreamPos;  //!< Stream position.
   typedef std::pair<sstreamPos,size_t>     Pair;        //!< Pair of a stream position and a line.
   typedef std::vector<Pair>                lineVector;  //!< Vector of position/line pairs.
   typedef std::map<std::string, BodyID>    LabelMap;    //!< Maps labels to rigid bodies.
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Indices for the scaling factors for length, velocity and weight parameters******************
   /*! Indices for the scaling factors for length, velocity and weight parameters. */
   enum {
      length   = 0,  //!< Index of the length scaling factor.
      velocity = 1,  //!< Index of the velocity scaling factor.
      weight   = 2   //!< Index of the weight scaling factor.
   };
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   BodyReader();
   BodyReader( const BodyReader& o );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~BodyReader();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   /*!\name Copy assignment operator */
   //@{
   BodyReader& operator=( const BodyReader& c );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getOffset()         const;
   inline const Vec3& getScalingFactors() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setOffset( real x, real y, real z );
   inline void setOffset( const Vec3& offset );
   inline void setScalingFactors( real lengthFactor, real velFactor, real weightFactor );
   inline void setPrecision( std::streamsize precision );
   //@}
   //**********************************************************************************************

   //**Error functions*****************************************************************************
   /*!\name Error functions */
   //@{
   inline bool        hasError()   const;
   inline std::string getError()   const;
   inline void        clearError();
   //@}
   //**********************************************************************************************

   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   void readFile ( const char* const filename, bool povray );
   void writeFile( const char* const filename,
                   std::ofstream::openmode mode = std::ofstream::trunc );
   void writeFile( const char* const filename, ConstSphereID sphere,
                   std::ofstream::openmode mode = std::ofstream::trunc );
   void writeFile( const char* const filename, ConstBoxID box,
                   std::ofstream::openmode mode = std::ofstream::trunc );
   void writeFile( const char* const filename, ConstCapsuleID capsule,
                   std::ofstream::openmode mode = std::ofstream::trunc );
   void writeFile( const char* const filename, ConstCylinderID cylinder,
                   std::ofstream::openmode mode = std::ofstream::trunc );
   void writeFile( const char* const filename, ConstPlaneID plane,
                   std::ofstream::openmode mode = std::ofstream::trunc );
   void writeFile( const char* const filename, ConstUnionID u,
                   std::ofstream::openmode mode = std::ofstream::trunc );
   //@}
   //**********************************************************************************************

private:
   //**I/O functions*******************************************************************************
   /*!\name I/O functions */
   //@{
   bool extractSphere( SphereParameters& sphere, Error& error, bool superordinate );
   bool extractBox( BoxParameters& box, Error& error, bool superordinate );
   bool extractCapsule( CapsuleParameters& capsule, Error& error, bool superordiante );
   bool extractCylinder( CylinderParameters& cylinder, Error& error, bool superordinate );
   bool extractPlane( PlaneParameters& plane, Error& error );
   bool extractAgglomerate( AgglomerateParameters& agglomerate, Error& error );
   bool extractUnion( UnionParameters& u, Error& error );
   bool extractSpring( SpringParameters& spring, Error& error );
   bool extractMaterial( MaterialID& material, Error& error );
   bool extractTexture( povray::Texture& texture, Error& error );
   bool extractPigment( povray::Pigment& pigment, Error& error );
   bool extractFinish( povray::Finish& finish, Error& error );
   bool extractNormal( povray::Normal& normal, Error& error );
   bool extractImageMap( povray::ImageMap& imagemap, Error& error );
   bool extractCamera( Error& error );
   bool extractLightSource( povray::LightSource& lightsource, Error& error );
   void writeFile( std::ostream& os, ConstSphereID sphere ) const;
   void writeFile( std::ostream& os, ConstBoxID box ) const;
   void writeFile( std::ostream& os, ConstCapsuleID capsule ) const;
   void writeFile( std::ostream& os, ConstCylinderID cylinder ) const;
   void writeFile( std::ostream& os, ConstPlaneID plane ) const;
   void writeFile( std::ostream& os, ConstUnionID u ) const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   size_t getLineNumber();
   void finishBlock();
   void skipBlock();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Error error_;                //!< Container for all error messages.
   Vec3 offset_;                //!< Displacement offset for the position of the generated bodies.
   Vec3 scaling_;               //!< Scaling factors for length, velocity and weight parameters.
   std::streamsize precision_;  //!< The precision for all output operations.

   std::string word_;           //!< Buffer for extracted keywords.
   size_t commandLine_;         //!< Line number of the current command.
   lineVector lineNumbers_;     //!< Vector for the line numbers of the input file.
   std::stringstream input_;    //!< Buffer for the preprocessed rigid body parameters.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend std::istream& operator>>( std::istream& is, EqualSign& s );
   friend std::istream& operator>>( std::istream& is, Semicolon& s );
   friend std::istream& operator>>( std::istream& is, LeadingBracket& lb );
   friend std::istream& operator>>( std::istream& is, TrailingBracket& lb );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the specified offset.
 *
 * \return The specified offset.
 *
 * The default offset is set to (0,0,0).
 */
inline const Vec3& BodyReader::getOffset() const
{
   return offset_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the specified scaling factors.
 *
 * \return The specified scaling factors.
 *
 * The default scaling factors are set to (1,1,1).
 */
inline const Vec3& BodyReader::getScalingFactors() const
{
   return scaling_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the offset for the rigid body setup.
 *
 * \param x The x-offset value.
 * \param y The y-offset value.
 * \param z The z-offset value.
 * \return void
 *
 * The offset of a body reader can be used to shift all rigid bodies during the setup by the
 * given offset values. The default offset is set to (0,0,0).
 */
inline void BodyReader::setOffset( real x, real y, real z )
{
   offset_[0] = x;
   offset_[1] = y;
   offset_[2] = z;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the offset for the rigid body setup.
 *
 * \param offset The offset values.
 * \return void
 *
 * The offset of a body reader can be used to shift all rigid bodies during the setup by the
 * given offset values. The default offset is set to (0,0,0).
 */
inline void BodyReader::setOffset( const Vec3& offset )
{
   offset_ = offset;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the scaling factors for the rigid body setup.
 *
 * \param lengthFactor Scaling factor for length parameters.
 * \param velFactor Scaling factor for velocity parameters.
 * \param weightFactor Scaling factor for weight parameters.
 * \return void
 * \exception std::invalid_argument Invalid scaling factor.
 *
 * The scaling factors of a body reader can be used to scale all length, velocity and weight
 * parameters during the setup by the given factors. The default scaling factors are set to
 * (1,1,1).
 */
inline void BodyReader::setScalingFactors( real lengthFactor, real velFactor, real weightFactor )
{
   if( lengthFactor <= real(0) || velFactor <= real(0) || weightFactor <= real(0) )
      throw std::invalid_argument( "Invalid scaling factor!" );

   scaling_[length]   = lengthFactor;
   scaling_[velocity] = velFactor;
   scaling_[weight]   = weightFactor;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the precision for the writeFile() functions.
 *
 * \param precision The number of digits of precision.
 * \return void
 *
 * This function specifies the number of digits of precision for all writeFile() functions.
 * The default precision are 6 valid digits.
 */
inline void BodyReader::setPrecision( std::streamsize precision )
{
   precision_ = precision;
}
//*************************************************************************************************




//=================================================================================================
//
//  ERROR FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the internal state of the body reader.
 *
 * \return \a true if an error occured, \a false if not.
 */
inline bool BodyReader::hasError() const
{
   return error_.hasError();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the error messages of the body reader.
 *
 * \return The error messages.
 */
inline std::string BodyReader::getError() const
{
   return error_.getError();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the state of the body reader.
 *
 * \return void
 *
 * The clear function resets the state of the body reader and removes any error message.
 */
inline void BodyReader::clearError()
{
   error_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::ERROR
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether any error has occured.
 *
 * \return \a true if an error has occured, \a false if not.
 */
inline BodyReader::Error::operator bool() const
{
   return error_;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns whether any error has occured.
 *
 * \return \a true if an error has occured, \a false if not.
 */
inline bool BodyReader::Error::hasError() const
{
   return error_;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Returns the error messages of the error object.
 *
 * \return The error messages.
 */
inline std::string BodyReader::Error::getError() const
{
   return message_.str();
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Resetting the error object.
 *
 * \return void
 *
 * This function resets the state of the error object.
 */
inline void BodyReader::Error::clear()
{
   error_ = false;
   message_.str( "" );
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Resetting the error object.
 *
 * \return void
 *
 * This function resets the state of the error object.
 */
inline void BodyReader::Error::append( const Error& error )
{
   if( error.error_ ) {
      error_ = true;
      message_ << error.message_.str();
   }
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Global output operator for the BodyReader::Error class.
 *
 * \param error Reference to the Error object.
 * \param output Error output to be stored in the Error object.
 * \return Reference to the Error object.
 */
template< typename Type >
BodyReader::Error& operator<<( BodyReader::Error& error, Type output )
{
   error.error_ = true;
   error.message_ << output;
   return error;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::SPHEREPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single sphere. */
struct BodyReader::SphereParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   SphereParameters();
   SphereParameters( const SphereParameters& sp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~SphereParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool fixed;               //!< Fixation flag of the sphere.
   bool visible;             //!< Visibility flag.
   bool materialSet;         //!< Material flag.
   bool textureSet;          //!< POV-Ray texture flag.
   size_t line;              //!< Line number of the sphere in the parameter file.
   size_t id;                //!< User-specific ID of the sphere.
   std::string label;        //!< Unique label.
   MaterialID material;      //!< Material of the sphere.
   real radius;              //!< The radius of the sphere.
   Vec3 center;              //!< Global position of the center of mass.
   Vec3 linear;              //!< Global linear velocity of the sphere.
   Vec3 angular;             //!< Global angular velocity of the sphere.
   Vec3 translation;         //!< Total translation of the sphere.
   Rotations rotations;      //!< The specified sphere rotations.
   povray::Texture texture;  //!< The POV-Ray texture of the sphere.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************



//=================================================================================================
//
//  CLASS BODYREADER::BOXPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single box. */
struct BodyReader::BoxParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   BoxParameters();
   BoxParameters( const BoxParameters& bp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~BoxParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool fixed;               //!< Fixation flag of the box.
   bool visible;             //!< Visibility flag.
   bool materialSet;         //!< Material flag.
   bool textureSet;          //!< POV-Ray texture flag.
   size_t line;              //!< Line number of the box in the parameter file.
   size_t id;                //!< User-specific ID of the box.
   std::string label;        //!< Unique label.
   MaterialID material;      //!< Material of the box.
   Vec3 center;              //!< Global position of the center of mass.
   Vec3 lengths;             //!< The side lengths of the box.
   Vec3 linear;              //!< Global linear velocity of the box.
   Vec3 angular;             //!< Global angular velocity of the box.
   Vec3 translation;         //!< Total translation of the box.
   Rotations rotations;      //!< The specified box rotations.
   povray::Texture texture;  //!< The POV-Ray texture of the box.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::CAPSULEPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single capsule. */
struct BodyReader::CapsuleParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   CapsuleParameters();
   CapsuleParameters( const CapsuleParameters& cp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~CapsuleParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool fixed;               //!< Fixation flag of the capsule.
   bool visible;             //!< Visibility flag.
   bool materialSet;         //!< Material flag.
   bool textureSet;          //!< POV-Ray texture flag.
   size_t line;              //!< Line number of the capsule in the parameter file.
   size_t id;                //!< User-specific ID of the capsule.
   std::string label;        //!< Unique label.
   MaterialID material;      //!< The material of the capsule.
   real radius;              //!< The radius of the capsule.
   real length;              //!< The length of the capsule.
   Vec3 center;              //!< Global position of the center of mass.
   Vec3 linear;              //!< Global linear velocity of the capsule.
   Vec3 angular;             //!< Global angular velocity of the capsule.
   Vec3 translation;         //!< Total translation of the capsule.
   Rotations rotations;      //!< The specified capsule rotations.
   povray::Texture texture;  //!< The POV-Ray texture of the capsule.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::CYLINDERPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single cylinder. */
struct BodyReader::CylinderParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   CylinderParameters();
   CylinderParameters( const CylinderParameters& cp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~CylinderParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool fixed;               //!< Fixation flag of the cylinder.
   bool visible;             //!< Visibility flag.
   bool materialSet;         //!< Material flag.
   bool textureSet;          //!< POV-Ray texture flag.
   size_t line;              //!< Line number of the cylinder in the parameter file.
   size_t id;                //!< User-specific ID of the cylinder.
   std::string label;        //!< Unique label.
   MaterialID material;      //!< The material of the cylinder.
   real radius;              //!< The radius of the cylinder.
   real length;              //!< The length of the cylinder.
   Vec3 center;              //!< Global position of the center of mass.
   Vec3 linear;              //!< Global linear velocity of the cylinder.
   Vec3 angular;             //!< Global angular velocity of the cylinder.
   Vec3 translation;         //!< Total translation of the cylinder.
   Rotations rotations;      //!< The specified cylinder rotations.
   povray::Texture texture;  //!< The POV-Ray texture of the cylinder.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::PLANEPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single plane. */
struct BodyReader::PlaneParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   PlaneParameters();
   PlaneParameters( const PlaneParameters& pp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~PlaneParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool visible;             //!< Visibility flag.
   bool materialSet;         //!< Material flag.
   bool textureSet;          //!< POV-Ray texture flag.
   size_t line;              //!< Line number of the plane in the parameter file.
   size_t id;                //!< User-specific ID of the plane.
   std::string label;        //!< Unique label.
   MaterialID material;      //!< The material of the plane.
   real displacement;        //!< The plane displacement.
   Vec3 normal;              //!< The normal of the plane.
   Vec3 translation;         //!< Total translation of the plane.
   Rotations rotations;      //!< The specified plane rotations.
   povray::Texture texture;  //!< The POV-Ray texture of the plane.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::LINKPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single link. */
struct BodyReader::LinkParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   LinkParameters();
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   bool operator==( const LinkParameters& rhs );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t id;          //!< User-specific ID of the link.
   std::string body1;  //!< Reference to the first linked primitive's label.
   std::string body2;  //!< Reference to the second linked primitive's label.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::AGGLOMERATEPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a particle agglomerate. */
struct BodyReader::AgglomerateParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   AgglomerateParameters();
   AgglomerateParameters( const AgglomerateParameters& cp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~AgglomerateParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool fixed;               //!< Fixation flag of the agglomerate.
   bool visible;             //!< Visibility flag of the agglomerate.
   bool materialSet;         //!< Material flag.
   bool textureSet;          //!< POV-Ray texture flag.
   size_t line;              //!< Line number of the agglomerate in the parameter file.
   size_t id;                //!< User-specific ID of the agglomerate.
   std::string label;        //!< Unique label.
   size_t number;            //!< Number of spheres contained in the agglomerate.
   MaterialID material;      //!< The material of the agglomerate.
   real radius;              //!< The radius of the agglomerate.
   real threshold;           //!< Degree of clustering of the contained spheres.
   Vec3 center;              //!< Global position of the center of mass.
   Vec3 linear;              //!< Global linear velocity of the agglomerate.
   Vec3 angular;             //!< Global angular velocity of the agglomerate.
   Vec3 translation;         //!< Total translation of the agglomerate.
   Rotations rotations;      //!< The specified agglomerate rotations.
   povray::Texture texture;  //!< The POV-Ray texture of the agglomerate.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::UNIONPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single union. */
struct BodyReader::UnionParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   UnionParameters();
   UnionParameters( const UnionParameters& cp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~UnionParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool fixed;                         //!< Fixation flag of the union.
   bool visible;                       //!< Visibility flag of the union.
   bool materialSet;                   //!< Material flag.
   bool centerSet;                     //!< Center flag.
   bool textureSet;                    //!< POV-Ray texture flag.
   size_t line;                        //!< Line number of the union in the parameter file.
   size_t id;                          //!< User-specific ID of the union.
   std::string label;                  //!< Unique label.
   MaterialID material;                //!< The material of the union.
   Vec3 center;                        //!< Global position of the center of mass.
   Vec3 linear;                        //!< Global linear velocity of the union.
   Vec3 angular;                       //!< Global angular velocity of the union.
   Vec3 translation;                   //!< Total translation of the union.
   Rotations rotations;                //!< The specified union rotations.
   SphereParameterVector spheres;      //!< The spheres within the union.
   BoxParameterVector boxes;           //!< The boxes within the union.
   CapsuleParameterVector capsules;    //!< The capsules within the union.
   CylinderParameterVector cylinders;  //!< The cylinders within the union.
   PlaneParameterVector planes;        //!< The planes within the union.
   LinkParameterVector links;          //!< The links within the union.
   povray::Texture texture;            //!< The POV-Ray texture of the union.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::SPRINGPARAMETERS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Input parameters for a single spring. */
struct BodyReader::SpringParameters
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   SpringParameters();
   SpringParameters( const SpringParameters& sp );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~SpringParameters();
   //@}
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool visible;       //!< Visibility flag.
   size_t line;        //!< Line number of the spring in the parameter file.
   real stiffness;     //!< The stiffness of the spring.
   real damping;       //!< The damping factor of the spring.
   real length;        //!< The length in non-deformed state.
   std::string body1;  //!< Reference to the first attached rigid body's label.
   std::string body2;  //!< Reference to the second attached rigid body's label.
   Vec3 anchor1;       //!< The first body's anchor point in body relative coordinates.
   Vec3 anchor2;       //!< The second body's anchor point in body relative coordinates.
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::EQUALSIGN
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Extraction class for '=' characters. */
struct BodyReader::EqualSign
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::SEMICOLON
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Extraction class for ';' characters. */
struct BodyReader::Semicolon
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::LEADINGBRACKET
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Extraction class for '{' characters. */
struct BodyReader::LeadingBracket
{};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  CLASS BODYREADER::TRAILINGBRACKET
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Extraction class for '}' characters. */
struct BodyReader::TrailingBracket
{};
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
