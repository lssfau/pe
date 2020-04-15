//=================================================================================================
/*!
 *  \file pe/core/Types.h
 *  \brief Header file for core module type definitions
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

#ifndef _PE_CORE_TYPES_H_
#define _PE_CORE_TYPES_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

class Attachable;
class BallJoint;
class BodyManager;
class Box;
class Capsule;
class Contact;
class Cylinder;
class FixedJoint;
class ForceGenerator;
class GeomPrimitive;
class Gravity;
class HingeJoint;
class Joint;
class Link;
class Material;
class MPISystem;
class Plane;
class Process;
class RigidBody;
class Section;
class SliderJoint;
class Sphere;
class Spring;
class TriangleMesh;
class Union;




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Handle for an Attachable.
 * \ingroup core
 */
typedef Attachable*  AttachableID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Attachable.
 * \ingroup core
 */
typedef const Attachable*  ConstAttachableID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a BallJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::BallJointID represents a handle to a currently active ball joint in the rigid body
 * simulation.
 *
 * \b Note: The pe::BallJointID handle refers to a non-constant ball joint. In contrast, the
 * pe::ConstBallJointID handle refers to a constant ball joint.
 */
typedef BallJoint*  BallJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a BallJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::ConstBallJointID represents a handle to a currently active ball joint in the rigid body
 * simulation.
 *
 * \b Note: The pe::ConstBallJointID handle refers to a constant ball joint. In contrast, the
 * pe::BallJointID handle refers to a non-constant ball joint.
 */
typedef const BallJoint*  ConstBallJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a BodyManager.
 * \ingroup core
 */
typedef BodyManager*  ManagerID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant BodyManager.
 * \ingroup core
 */
typedef const BodyManager*  ConstManagerID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Box primitive.
 * \ingroup core
 *
 * A pe::BoxID represents a handle to a currently active Box primitive in the rigid body simulation.
 * A handle to a box primitive is returned by the box creation functions:
 *
 * - pe::createBox( size_t id, real x, real y, real z, real lx, real ly, real lz, MaterialID material, bool visible )
 * - pe::createBox( size_t id, const Vec3 &gPos, real lx, real ly, real lz, MaterialID material, bool visible )
 * - pe::createBox( size_t id, real x, real y, real z, const Vec3 &lengths, MaterialID material, bool visible )
 * - pe::createBox( size_t id, const Vec3 &gPos, const Vec3 &lengths, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new box primitive and box handle:

   \code
   // Creates the iron box 1 at the global position ( 4.2, 3.7, -0.6 ) with side lengths
   // of ( 1.2, 4.8, 0.8 )
   BoxID box = createBox( 1, 4.2, 3.7, -0.6, 1.2, 4.8, 0.8, iron );
   \endcode

 * \b Note: A pe::BoxID can implicitly be converted to both a pe::GeomID and a pe::BodyID.

   \code
   BoxID box;
   GeomID geom = box;
   BodyID body = geom;
   \endcode

 * \b Note: The pe::BoxID handle refers to a non-constant box primitive. In contrast, the
 * pe::ConstBoxID handle refers to a constant box primitive.
 */
typedef Box*  BoxID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Box primitive.
 * \ingroup core
 *
 * A pe::ConstBoxID represents a handle to a currently active Box primitive in the rigid body
 * simulation. A handle to a box primitive is returned by the box creation functions:
 *
 * - pe::createBox( size_t id, real x, real y, real z, real lx, real ly, real lz, MaterialID material, bool visible )
 * - pe::createBox( size_t id, const Vec3 &gPos, real lx, real ly, real lz, MaterialID material, bool visible )
 * - pe::createBox( size_t id, real x, real y, real z, const Vec3 &lengths, MaterialID material, bool visible )
 * - pe::createBox( size_t id, const Vec3 &gPos, const Vec3 &lengths, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new box primitive and box handle:

   \code
   // Creates the iron box 1 at the global position ( 4.2, 3.7, -0.6 ) with side lengths
   // of ( 1.2, 4.8, 0.8 )
   BoxID box = createBox( 1, 4.2, 3.7, -0.6, 1.2, 4.8, 0.8, iron );
   \endcode

 * \b Note: A pe::ConstBoxID can implicitly be converted to both a pe::ConstGeomID and a
 * pe::ConstBodyID.

   \code
   ConstBoxID box;
   ConstGeomID geom = box;
   ConstBodyID body = geom;
   \endcode

 * \b Note: The pe::ConstBoxID handle refers to a constant box primitive. In contrast, the
 * pe::BoxID handle refers to a non-constant box primitive.
 */
typedef const Box*  ConstBoxID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Capsule primitive.
 * \ingroup core
 *
 * A pe::CapsuleID represents a handle to a currently active Capsule primitive in the rigid body
 * simulation. A handle to a capsule primitive is returned by the capsule creation functions:
 *
 * - pe::createCapsule( size_t id, real x, real y, real z, real radius, real length, MaterialID material, bool visible )
 * - pe::createCapsule( size_t id, const Vec3 &gPos, real radius, real length, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new capsule primitive and capsule handle:

   \code
   // Creates the iron capsule 1 at the global position ( 4.2, 3.7, -0.6 ) with radius 1.6
   // and length 6.4
   CapsuleID capsule = createCapsule( 1, 4.2, 3.7, -0.6, 1.6, 6.4, iron );
   \endcode

 * \b Note: A pe::CapsuleID can implicitly be converted to both a pe::GeomID and a pe::BodyID.

   \code
   CapsuleID capsule;
   GeomID geom = capsule;
   BodyID body = geom;
   \endcode

 * \b Note: The pe::CapsuleID handle refers to a non-constant capsule primitive. In contrast, the
 * pe::ConstCapsuleID handle refers to a constant capsule primitive.
 */
typedef Capsule*  CapsuleID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Capsule primitive.
 * \ingroup core
 *
 * A pe::ConstCapsuleID represents a handle to a currently active Capsule primitive in the rigid
 * body simulation. A handle to a capsule primitive is returned by the capsule creation functions:
 *
 * - pe::createCapsule( size_t id, real x, real y, real z, real radius, real length, MaterialID material, bool visible )
 * - pe::createCapsule( size_t id, const Vec3 &gPos, real radius, real length, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new capsule primitive and capsule handle:

   \code
   // Creates the iron capsule 1 at the global position ( 4.2, 3.7, -0.6 ) with radius 1.6
   // and length 6.4
   CapsuleID capsule = createCapsule( 1, 4.2, 3.7, -0.6, 1.6, 6.4, iron );
   \endcode

 * \b Note: A pe::ConstCapsuleID can implicitly be converted to both a pe::ConstGeomID and a
 * pe::ConstBodyID.

   \code
   ConstCapsuleID capsule;
   ConstGeomID geom = capsule;
   ConstBodyID body = geom;
   \endcode

 * \b Note: The pe::ConstCapsuleID handle refers to a constant capsule primitive. In contrast, the
 * pe::CapsuleID handle refers to a non-constant capsule primitive.
 */
typedef const Capsule*  ConstCapsuleID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a contact between two rigid bodies.
 * \ingroup core
 */
typedef Contact*  ContactID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant contact between two rigid bodies.
 * \ingroup core
 */
typedef const Contact*  ConstContactID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Cylinder primitive.
 * \ingroup core
 *
 * A pe::CylinderID represents a handle to a currently active Cylinder primitive in the rigid
 * body simulation. A handle to a cylinder primitive is returned by the cylinder creation
 * functions:
 *
 * - pe::createCylinder( size_t id, real x, real y, real z, real radius, real length, MaterialID material, bool visible )
 * - pe::createCylinder( size_t id, const Vec3 &gPos, real radius, real length, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new cylinder primitive and cylinder handle:

   \code
   // Creates the iron cylinder 1 at the global position ( 4.2, 3.7, -0.6 ) with radius 1.6
   // and length 6.4
   CylinderID cylinder = createCylinder( 1, 4.2, 3.7, -0.6, 1.6, 6.4, iron );
   \endcode

 * \b Note: A pe::CylinderID can implicitly be converted to both a pe::GeomID and a pe::BodyID.

   \code
   CylinderID cylinder;
   GeomID geom = cylinder;
   BodyID body = geom;
   \endcode

 * \b Note: The pe::CylinderID handle refers to a non-constant cylinder primitive. In contrast,
 * the pe::ConstCylinderID handle refers to a constant cylinder primitive.
 */
typedef Cylinder*  CylinderID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Cylinder primitive.
 * \ingroup core
 *
 * A pe::ConstCylinderID represents a handle to a currently active Cylinder primitive in the
 * rigid body simulation. A handle to a cylinder primitive is returned by the cylinder creation
 * functions:
 *
 * - pe::createCylinder( size_t id, real x, real y, real z, real radius, real length, MaterialID material, bool visible )
 * - pe::createCylinder( size_t id, const Vec3 &gPos, real radius, real length, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new cylinder primitive and cylinder handle:

   \code
   // Creates the iron cylinder 1 at the global position ( 4.2, 3.7, -0.6 ) with radius 1.6
   // and length 6.4
   CylinderID cylinder = createCylinder( 1, 4.2, 3.7, -0.6, 1.6, 6.4, iron );
   \endcode

 * \b Note: A pe::ConstCylinderID can implicitly be converted to both a pe::ConstGeomID and a
 * pe::ConstBodyID.

   \code
   ConstCylinderID cylinder;
   ConstGeomID geom = cylinder;
   ConstBodyID body = geom;
   \endcode

 * \b Note: The pe::ConstCylinderID handle refers to a constant cylinder primitive. In contrast,
 * the pe::CylinderID handle refers to a non-constant cylinder primitive.
 */
typedef const Cylinder*  ConstCylinderID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a FixedJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::FixedJointID represents a handle to a currently active fixed joint in the rigid body
 * simulation.
 *
 * \b Note: The pe::FixedJointID handle refers to a non-constant fixed joint. In contrast, the
 * pe::ConstFixedJointID handle refers to a constant fixed joint.
 */
typedef FixedJoint*  FixedJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a FixedJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::ConstFixedJointID represents a handle to a currently active fixed joint in the rigid body
 * simulation.
 *
 * \b Note: The pe::ConstFixedJointID handle refers to a constant fixed joint. In contrast, the
 * pe::FixedJointID handle refers to a non-constant fixed joint.
 */
typedef const FixedJoint*  ConstFixedJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a force generator.
 * \ingroup core
 */
typedef ForceGenerator*  ForceGeneratorID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant force generator.
 * \ingroup core
 */
typedef const ForceGenerator*  ConstForceGeneratorID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a geometric primitive.
 * \ingroup core
 *
 * A pe::GeomID represents a handle to a currently active geometric primitive in the rigid body
 * simulation. Possible geometric primitives include Sphere, Box, Capsule, Cylinder, and Plane.
 *
 * \b Note: A pe::GeomID can implicitly be converted to a pe::BodyID.

   \code
   GeomID geom;
   BodyID body = geom;
   \endcode

 * \b Note: The pe::GeomID handle refers to a non-constant geometric primitive. In contrast, the
 * pe::ConstGeomID handle refers to a constant geometric primitive.
 */
typedef GeomPrimitive*  GeomID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant geometric primitive.
 * \ingroup core
 *
 * A pe::ConstGeomID represents a handle to a currently active geometric primitive in the rigid
 * body simulation. Possible geometric primitives include Sphere, Box, Capsule, Cylinder, and
 * Plane.
 *
 * \b Note: A pe::ConstGeomID can implicitly be converted to a pe::ConstBodyID.

   \code
   ConstGeomID geom;
   ConstBodyID body = geom;
   \endcode

 * \b Note: The pe::ConstGeomID handle refers to a constant geometric primitive. In contrast, the
 * pe::GeomID handle refers to a non-constant geometric primitive.
 */
typedef const GeomPrimitive*  ConstGeomID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Gravity force generator object.
 * \ingroup core
 *
 * A pe::GravityID represents a handle to a currently active Gravity force generator. A
 * handle to a gravity force generator is returned by the gravity setup functions:
 *
 * - pe::attachGravity( BodyID body, real gx, real gy, real gz );
 * - pe::attachGravity( BodyID body, const Vec3& gravity );
 *
 * The following example illustrates the setup of a new gravity force generator:

   \code
   using namespace pe;

   // Creating a sphere primitive
   SphereID sphere = createSphere( 1, 2.4, -1.4, 5.0, 0.9, oak );

   // Attaching a gravity force generator to the sphere
   GravityID gravity = attachGravity( sphere, Vec3( 0.0, 0.0, -9.81 ) );
   \endcode

 * \b Note: The pe::GravityID handle refers to a non-constant gravity force generator. In
 * contrast, the pe::ConstGravityID handle refers to a constant gravity force generator.
 */
typedef Gravity*  GravityID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Gravity force generator object.
 * \ingroup core
 *
 * A pe::ConstGravityID represents a handle to a currently active Gravity force generator.
 * A handle to a gravity force generator is returned by the gravity setup functions:
 *
 * - pe::attachGravity( BodyID body, real gx, real gy, real gz );
 * - pe::attachGravity( BodyID body, const Vec3& gravity );
 *
 * The following example illustrates the setup of a new gravity force generator:

   \code
   using namespace pe;

   // Creating a sphere primitive
   SphereID sphere = createSphere( 1, 2.4, -1.4, 5.0, 0.9, oak );

   // Attaching a gravity force generator to the sphere
   GravityID gravity = attachGravity( sphere, Vec3( 0.0, 0.0, -9.81 ) );
   \endcode

 * \b Note: The pe::ConstGravityID handle refers to a constant gravity force generator. In
 * contrast, the pe::GravityID handle refers to a non-constant gravity force generator.
 */
typedef const Gravity*  ConstGravityID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a HingeJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::HingeJointID represents a handle to a currently active hinge joint in the rigid body
 * simulation.
 *
 * \b Note: The pe::HingeJointID handle refers to a non-constant hinge joint. In contrast, the
 * pe::ConstHingeJointID handle refers to a constant hinge joint.
 */
typedef HingeJoint*  HingeJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a HingeJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::ConstHingeJointID represents a handle to a currently active hinge joint in the rigid
 * body simulation.
 *
 * \b Note: The pe::ConstHingeJointID handle refers to a constant hinge joint. In contrast, the
 * pe::HingeJointID handle refers to a non-constant hinge joint.
 */
typedef const HingeJoint*  ConstHingeJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Joint between two rigid bodies.
 * \ingroup core
 *
 * A pe::JointID represents a handle to a currently active joint in the rigid body simulation.
 * Possible joints include ball joints, fixed joints, slider joints, etc.
 *
 * \b Note: The pe::JointID handle refers to a non-constant joint. In contrast, the
 * pe::ConstJointID handle refers to a constant joint.
 */
typedef Joint*  JointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Joint between two rigid bodies.
 * \ingroup core
 *
 * A pe::ConstJointID represents a handle to a currently active joint in the rigid body simulation.
 * Possible joints include ball joints, fixed joints, slider joints, etc.
 *
 * \b Note: The pe::ConstJointID handle refers to a constant joint. In contrast, the pe::JointID
 * handle refers to a non-constant joint.
 */
typedef const Joint*  ConstJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Link.
 * \ingroup core
 *
 * A pe::LinkID represents a handle to a specific Link between two rigid bodies contained in
 * a Union. A handle to a link is returned by the link creation function:
 *
 * - pe::createLink( UnionID u, size_t id, BodyID b1, BodyID b2 )
 *
 * The following example illustrates the creation of a new link between two sphere primitives:

   \code
   // Creating two touching sphere primitives
   SphereID s1 = createSphere( 1, -1.0, 0.0, 0.0, 1.0, iron );
   SphereID s2 = createSphere( 2,  1.0, 0.0, 0.0, 1.0, iron );

   // Adding the two sphere primitives to a union
   UnionID  u  = createUnion( 0 );
   u->add( s1 );
   u->add( s2 );

   // Creating a link between the two spheres
   LinkID link = createLink( u, 1, s1, s2 );
   \endcode

 * \b Note: The pe::LinkID handle refers to a non-constant link. In contrast, the pe::ConstLinkID
 * handle refers to a constant link.
 */
typedef Link*  LinkID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Link.
 * \ingroup core
 *
 * A pe::ConstLinkID represents a handle to a specific Link between two rigid bodies contained in
 * a Union. A handle to a link is returned by the link creation function:
 *
 * - pe::createLink( UnionID u, size_t id, BodyID b1, BodyID b2 )
 *
 * The following example illustrates the creation of a new link between two sphere primitives:

   \code
   // Creating two touching sphere primitives
   SphereID s1 = createSphere( 1, -1.0, 0.0, 0.0, 1.0, iron );
   SphereID s2 = createSphere( 2,  1.0, 0.0, 0.0, 1.0, iron );

   // Adding the two sphere primitives to a union
   UnionID  u  = createUnion( 0 );
   u->add( s1 );
   u->add( s2 );

   // Creating a link between the two spheres
   LinkID link = createLink( u, 1, s1, s2 );
   \endcode

 * \b Note: The pe::ConstLinkID handle refers to a constant link. In contrast, the pe::LinkID
 * handle refers to a non-constant link.
 */
typedef const Link*  ConstLinkID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Vector for materials.
 * \ingroup core
 */
typedef std::vector<Material>  Materials;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unique material ID.
 * \ingroup core
 *
 * Every registered material has a unique MaterialID that can be used wherever the material is
 * required. The \b pe engine provides a couple of predefined materials (see the \ref materials
 * module). However, it is possible to define a custom material via the createMaterial() function:

   \code
   // Creates the material "myMaterial" with the following material properties:
   //  - material density               : 2.54
   //  - coefficient of restitution     : 0.8
   //  - coefficient of static friction : 0.1
   //  - coefficient of dynamic friction: 0.05
   MaterialID myMaterial = createMaterial( "myMaterial", 2.54, 0.8, 0.1, 0.05 );
   \endcode
 */
typedef Materials::size_type  MaterialID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Plane primitive.
 * \ingroup core
 *
 * A pe::PlaneID represents a handle to a currently active Plane primitive in the rigid body
 * simulation. A handle to a plane primitive is returned by the plane creation functions:
 *
 * - pe::createPlane( size_t id, real a, real b, real c, real d, MaterialID material, bool visible )
 * - pe::createPlane( size_t id, const Vec3 &normal, real d, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new plane primitive and plane handle:

   \code
   // Creates the granite plane 1 with normal (0,0,1) and displacement 0.5
   PlaneID plane = createPlane( 1, 0.0, 0.0, 1.0, 0.5, granite );
   \endcode

 * \b Note: A pe::PlaneID can implicitly be converted to both a pe::GeomID and a pe::BodyID.

   \code
   PlaneID plane;
   GeomID geom = plane;
   BodyID body = geom;
   \endcode

 * \b Note: The pe::PlaneID handle refers to a non-constant plane primitive. In contrast, the
 * pe::ConstPlaneID handle refers to a constant plane primitive.
 */
typedef Plane*  PlaneID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Plane primitive.
 * \ingroup core
 *
 * A pe::ConstPlaneID represents a handle to a currently active Plane primitive in the rigid body
 * simulation. A handle to a plane primitive is returned by the plane creation functions:
 *
 * - pe::createPlane( size_t id, real a, real b, real c, real d, MaterialID material, bool visible )
 * - pe::createPlane( size_t id, const Vec3 &normal, real d, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new plane primitive and plane handle:

   \code
   // Creates the granite plane 1 with normal (0,0,1) and displacement 0.5
   PlaneID plane = createPlane( 1, 0.0, 0.0, 1.0, 0.5, granite );
   \endcode

 * \b Note: A pe::ConstPlaneID can implicitly be converted to both a pe::ConstGeomID and a
 * pe::ConstBodyID.

   \code
   ConstPlaneID plane;
   ConstGeomID geom = plane;
   ConstBodyID body = geom;
   \endcode

 * \b Note: The pe::ConstPlaneID handle refers to a constant plane primitive. In contrast, the
 * pe::PlaneID handle refers to a non-constant plane primitive.
 */
typedef const Plane*  ConstPlaneID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a remote MPI process.
 * \ingroup mpi
 *
 * A pe::ProcessID represents a handle to a remote MPI process during a MPI parallel
 * simulation. A remote process is created via the connect() functions:
 *
 * -# connect( int rank, real a, real b, real c, real d );
 * -# connect( int rank, Vec3 normal, real d );
 *
 * For more details, see the class description of the Process class.
 *
 * \b Note: The pe::ProcessID handle refers to a non-constant remote process. In contrast,
 * the pe::ConstProcessID handle refers to a constant process.
 */
typedef Process*  ProcessID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant remote MPI process.
 * \ingroup mpi
 *
 * A pe::ConstProcessID represents a handle to a remote MPI process during a MPI parallel
 * simulation. A remote process is created via the connect() functions:
 *
 * -# connect( int rank, real a, real b, real c, real d );
 * -# connect( int rank, Vec3 normal, real d );
 *
 * For more details, see the class description of the Process class.
 *
 * \b Note: The pe::ConstProcessID handle refers to a constant remote process. In contrast,
 * the pe::ProcessID handle refers to a non-constant process.
 */
typedef const Process*  ConstProcessID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a RigidBody.
 * \ingroup core
 *
 * A pe::BodyID represents a handle to a currently active rigid body in the rigid body
 * simulation. Possible rigid bodies include e.g. Sphere, Box, Capsule, Cylinder, Plane,
 * and Union.
 *
 * \b Note: The pe::BodyID handle refers to a non-constant rigid body. In contrast, the
 * pe::ConstBodyID handle refers to a constant rigid body.
 */
typedef RigidBody*  BodyID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant RigidBody.
 * \ingroup core
 *
 * A pe::ConstBodyID represents a handle to a currently active rigid body in the rigid body
 * simulation. Possible rigid bodies include e.g. Sphere, Box, Capsule, Cylinder, Plane, and
 * Union.
 *
 * \b Note: The pe::ConstBodyID handle refers to a constant rigid body. In contrast, the
 * pe::BodyID handle refers to a non-constant rigid body.
 */
typedef const RigidBody*  ConstBodyID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a SliderJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::SliderJointID represents a handle to a currently active slider joint in the rigid
 * body simulation.
 *
 * \b Note: The pe::SliderJointID handle refers to a non-constant slider joint. In contrast,
 * the pe::ConstSliderJointID handle refers to a constant slider joint.
 */
typedef SliderJoint*  SliderJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant SliderJoint between two rigid bodies.
 * \ingroup core
 *
 * A pe::ConstSliderJointID represents a handle to a currently active slider joint in the rigid
 * body simulation.
 *
 * \b Note: The pe::ConstSliderJointID handle refers to a constant slider joint. In contrast,
 * the pe::SliderJointID handle refers to a non-constant slider joint.
 */
typedef const SliderJoint*  ConstSliderJointID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Sphere primitive.
 * \ingroup core
 *
 * A pe::SphereID represents a handle to a currently active Sphere primitive in the rigid body
 * simulation. A handle to a sphere primitive is returned by the sphere creation functions:
 *
 * - pe::createSphere( size_t id, real x, real y, real z, real radius, MaterialID material, bool visible )
 * - pe::createSphere( size_t id, const Vec3 &gPos, real radius, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new sphere primitive and sphere handle:

   \code
   // Creates the iron sphere 1 at the global position ( 4.2, 3.7, -0.6 ) with a radius of 1.2
   SphereID sphere = createSphere( 1, 4.2, 3.7, -0.6, 1.2, iron );
   \endcode

 * \b Note: A pe::SphereID can implicitly be converted to both a pe::GeomID and a pe::BodyID.

   \code
   SphereID sphere;
   GeomID geom = sphere;
   BodyID body = geom;
   \endcode

 * \b Note: The pe::SphereID handle refers to a non-constant sphere primitive. In contrast, the
 * pe::ConstSphereID handle refers to a constant sphere primitive.
 */
typedef Sphere*  SphereID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Sphere primitive.
 * \ingroup core
 *
 * A pe::ConstSphereID represents a handle to a currently active Sphere primitive in the rigid
 * body simulation. A handle to a sphere primitive is returned by the sphere creation functions:
 *
 * - pe::createSphere( size_t id, real x, real y, real z, real radius, MaterialID material, bool visible )
 * - pe::createSphere( size_t id, const Vec3 &gPos, real radius, MaterialID material, bool visible )
 *
 * The following example illustrates the creation of a new sphere primitive and sphere handle:

   \code
   // Creates the iron sphere 1 at the global position ( 4.2, 3.7, -0.6 ) with a radius of 1.2
   SphereID sphere = createSphere( 1, 4.2, 3.7, -0.6, 1.2, iron );
   \endcode

 * \b Note: A pe::ConstSphereID can implicitly be converted to both a pe::ConstGeomID and a
 * pe::ConstBodyID.

   \code
   ConstSphereID sphere;
   ConstGeomID geom = sphere;
   ConstBodyID body = geom;
   \endcode

 * \b Note: The pe::ConstSphereID handle refers to a constant sphere primitive. In contrast, the
 * pe::SphereID handle refers to a non-constant sphere primitive.
 */
typedef const Sphere*  ConstSphereID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Spring object.
 * \ingroup core
 *
 * A pe::SpringID represents a handle to a currently active Spring force generator. A
 * handle to a spring force generator is returned by the spring setup functions:
 *
 * - pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, bool visible );
 * - pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, bool visible );
 * - pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, real length, bool visible );
 * - pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, real length, bool visible );
 *
 * The following example illustrates the setup of a new spring force generator:

   \code
   using namespace pe;

   // Creating two spherical rigid bodies
   SphereID sphere1 = createSphere( 1, -4.0, 0.0, 0.0, 1.0, fir    );
   SphereID sphere2 = createSphere( 2,  4.0, 0.0, 0.0, 1.0, copper );

   // Attaching a spring force generator to the two spheres
   SpringID spring = attachSpring( sphere1, sphere2, 1000, 100 );
   \endcode

 * \b Note: The pe::SpringID handle refers to a non-constant spring force generator. In
 * contrast, the pe::ConstSpringID handle refers to a constant spring force generator.
 */
typedef Spring* SpringID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Spring object.
 * \ingroup core
 *
 * A pe::ConstSpringID represents a handle to a currently active Spring force generator.
 * A handle to a spring force generator is returned by the spring setup functions:
 *
 * - pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, bool visible );
 * - pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, bool visible );
 * - pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, real length, bool visible );
 * - pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, real length, bool visible );
 *
 * The following example illustrates the setup of a new spring force generator:

   \code
   using namespace pe;

   // Creating two spherical rigid bodies
   SphereID sphere1 = createSphere( 1, -4.0, 0.0, 0.0, 1.0, fir    );
   SphereID sphere2 = createSphere( 2,  4.0, 0.0, 0.0, 1.0, copper );

   // Attaching a spring force generator to the two spheres
   SpringID spring = attachSpring( sphere1, sphere2, 1000, 100 );
   \endcode

 * \b Note: The pe::ConstSpringID handle refers to a constant spring force generator. In
 * contrast, the pe::SpringID handle refers to a non-constant spring force generator.
 */
typedef const Spring* ConstSpringID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a TriangleMesh.
 * \ingroup core
 */
typedef TriangleMesh*  TriangleMeshID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant TriangleMesh.
 * \ingroup core
 */
typedef const TriangleMesh*  ConstTriangleMeshID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a Union.
 * \ingroup core
 *
 * A pe::UnionID represents a handle to a currently active Union in the rigid body simulation. A
 * handle to a union is returned by the union creation function:
 *
 * - pe::createUnion( size_t id, bool visible )
 *
 * The following example illustrates the creation of a new union body and union handle:

   \code
   // Creates the union 1
   UnionID u = createUnion( 1 );
   \endcode

 * \b Note: A UnionID can implicitly be converted to a pe::BodyID.

   \code
   UnionID u;
   BodyID body = u;
   \endcode

 * \b Note: The pe::UnionID handle refers to a non-constant union. In contrast, the pe::ConstUnionID
 * handle refers to a constant union.
 */
typedef Union*  UnionID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for a constant Union.
 * \ingroup core
 *
 * A pe::ConstUnionID represents a handle to a currently active Union in the rigid body simulation.
 * A handle to a union is returned by the union creation function:
 *
 * - pe::createUnion( size_t id, bool visible )
 *
 * The following example illustrates the creation of a new union body and union handle:

   \code
   // Creates the union 1
   UnionID u = createUnion( 1 );
   \endcode

 * \b Note: A pe::ConstUnionID can implicitly be converted to a pe::ConstBodyID.

   \code
   ConstUnionID u;
   ConstBodyID body = u;
   \endcode

 * \b Note: The pe::ConstUnionID handle refers to a constant union. In contrast, the pe::UnionID
 * handle refers to a non-constant union.
 */
typedef const Union*  ConstUnionID;
//*************************************************************************************************

} // namespace pe

#endif
