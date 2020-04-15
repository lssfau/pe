//=================================================================================================
/*!
 *  \file pe/core/detection/fine/MaxContacts.h
 *  \brief Contact generation module
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2013-2014 Tobias Scharpff
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

#ifndef _PE_CORE_DETECTION_FINE_MAXCONTACTS_H_
#define _PE_CORE_DETECTION_FINE_MAXCONTACTS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <map>
#include <utility>
#include <set>

#include <pe/core/detection/fine/GJK.h>
#include <pe/core/detection/fine/EPA.h>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/GeomTools.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Types.h>
#include <pe/math/Accuracy.h>
#include <pe/math/Epsilon.h>
#include <pe/math/Functions.h>
#include <pe/math/Infinity.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/shims/Square.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/logging/DetailSection.h>
#include <pe/util/NonCreatable.h>
#include <pe/util/Timing.h>
#include <pe/util/Types.h>


namespace pe {

namespace detection {

namespace fine {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact generation module.
 * \ingroup fine_collision_detection
 *
 * The MaxContacts class is part of the fine collision detection module of the \b pe physics
 * engine and provides the functionality to create contacts between colliding rigid bodies.
 * The primary focus of this class is physical accuracy and therefore it creates the maximal
 * number of contact points necessary to handle collisions between rigid bodies physically
 * accurate (hence the name of the class).
 */
class MaxContacts : private NonCreatable
{
public:
   //**Contact generation functions****************************************************************
   /*!\name Contact generation functions */
   //@{
   template< typename CC > static        void collide                ( BodyID b1    , BodyID b2    , CC& contacts );
   template< typename CC > static inline void collideSphereSphere    ( SphereID s1  , SphereID s2  , CC& contacts );
   template< typename CC > static        void collideSphereBox       ( SphereID s   , BoxID b      , CC& contacts );
   template< typename CC > static        void collideSphereCapsule   ( SphereID s   , CapsuleID c  , CC& contacts );
   template< typename CC > static        void collideSphereCylinder  ( SphereID s   , CylinderID c , CC& contacts );
   template< typename CC > static inline void collideSpherePlane     ( SphereID s   , PlaneID p    , CC& contacts );
   template< typename CC > static        void collideSphereTMesh     ( SphereID s   , TriangleMeshID m     , CC& contacts );
   template< typename CC > static inline void collideSphereUnion     ( SphereID s   , UnionID u    , CC& contacts );
   template< typename CC > static        void collideBoxBox          ( BoxID b1     , BoxID b2     , CC& contacts );
   template< typename CC > static        void collideBoxCapsule      ( BoxID b      , CapsuleID c  , CC& contacts );
   template< typename CC > static        void collideBoxCylinder     ( BoxID b      , CylinderID c , CC& contacts );
   template< typename CC > static        void collideBoxPlane        ( BoxID b      , PlaneID p    , CC& contacts );
   template< typename CC > static        void collideBoxTMesh        ( BoxID b      , TriangleMeshID m     , CC& contacts );
   template< typename CC > static inline void collideBoxUnion        ( BoxID b      , UnionID u    , CC& contacts );
   template< typename CC > static        void collideCapsuleCapsule  ( CapsuleID c1 , CapsuleID c2 , CC& contacts );
   template< typename CC > static        void collideCapsuleCylinder ( CapsuleID ca , CylinderID cy, CC& contacts );
   template< typename CC > static        void collideCapsulePlane    ( CapsuleID c  , PlaneID p    , CC& contacts );
   template< typename CC > static        void collideCapsuleTMesh    ( CapsuleID c  , TriangleMeshID m     , CC& contacts );
   template< typename CC > static inline void collideCapsuleUnion    ( CapsuleID c  , UnionID u    , CC& contacts );
   template< typename CC > static        void collideCylinderCylinder( CylinderID c1, CylinderID c2, CC& contacts );
   template< typename CC > static        void collideCylinderPlane   ( CylinderID c , PlaneID p    , CC& contacts );
   template< typename CC > static        void collideCylinderTMesh   ( CylinderID c , TriangleMeshID m     , CC& contacts );
   template< typename CC > static inline void collideCylinderUnion   ( CylinderID c , UnionID u    , CC& contacts );
   template< typename CC > static inline void collidePlaneTMesh      ( PlaneID p    , TriangleMeshID  u    , CC& contacts );
   template< typename CC > static inline void collidePlaneUnion      ( PlaneID p    , UnionID u    , CC& contacts );
   template< typename CC > static inline void collideTMeshTMesh      ( TriangleMeshID  m1   , TriangleMeshID m2    , CC& contacts );
   template< typename CC > static inline void collideTMeshUnion      ( TriangleMeshID  m    , UnionID u    , CC& contacts );
   template< typename CC > static inline void collideUnionUnion      ( UnionID u1   , UnionID u2   , CC& contacts );
   //@}
   //**********************************************************************************************

protected:
   //**Contact generation utility functions********************************************************
   /*!\name Contact generation utility functions */
   //@{
   template < typename Type1 , typename Type2 >
   static inline bool gjkEPAcollideHybrid(Type1 geom1, Type2 geom2, Vec3& normal, Vec3& contactPoint, real& penetrationDepth);

   template < typename Type1 , typename Type2 >
   static inline bool gjkEPAcollide(Type1 geom1, Type2 geom2, Vec3& normal, Vec3& contactPoint, real& penetrationDepth);

   template< typename Type >
   static inline bool gjkEPAcollide(Type geom, TriangleMeshID mesh, Vec3& normal, Vec3& contactPoint, real& penetrationDepth);
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//================================================================================================
//
// CONTACT GENERATION UTILITY FUNCTIONS
//
//================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type1 , typename Type2 >
inline bool MaxContacts::gjkEPAcollideHybrid(Type1 geom1, Type2 geom2, Vec3& normal, Vec3& contactPoint, real& penetrationDepth)
{
   // For more information on hybrid GJK/EPA see page 166 in "Collision Detecton in Interactive 3D
   // Environments" by Gino van den Bergen.

   GJK gjk;
   penetrationDepth = gjk.doGJK< Type1, Type2 >(geom1, geom2, normal, contactPoint);
   if(penetrationDepth > contactThreshold) {
      // not close enough create no contact
      return false;
   }
   else if(penetrationDepth < 0.01*contactThreshold) {
      // objects are quite close use GJKcontactTrashold + EPAcontactTrashold to calc distance
      if(gjk.doGJKcontactThreshold<Type1, Type2>(geom1, geom2)) {
         //possible penetration
         EPA epa;
         return epa.doEPAcontactThreshold<Type1, Type2>(geom1, geom2, gjk, normal, contactPoint, penetrationDepth);
      }
   }
   else {
      // objects are close but separated use data calculated by GJK
      return true;
   }
   //never to be reached
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type1 , typename Type2 > // ID-Type of the geometry 
inline bool MaxContacts::gjkEPAcollide(Type1 geom1, Type2 geom2, Vec3& normal, Vec3& contactPoint, real& penetrationDepth)
{
   GJK gjk;
   if(gjk.doGJKcontactThreshold<Type1, Type2>(geom1, geom2)) {
      //possible penetration
      EPA epa;
      return epa.doEPAcontactThreshold<Type1, Type2>(geom1, geom2, gjk, normal, contactPoint, penetrationDepth);
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type > // ID-Type of the geometry which is not a triangle mesh
inline bool MaxContacts::gjkEPAcollide(Type geom, TriangleMeshID mesh, Vec3& normal, Vec3& contactPoint, real& penetrationDepth)
{
   return gjkEPAcollide<Type, TriangleMeshID>(geom, mesh, normal, contactPoint, penetrationDepth);
}
//*************************************************************************************************




//=================================================================================================
//
//  CONTACT GENERATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact generation between two colliding rigid bodies.
 * \ingroup contact_generation
 *
 * \param b1 The first colliding body.
 * \param b2 The second colliding body.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * Calculation of all necessary contact points between two rigid bodies. This function works as
 * a dispatch function that checks the actual type of the rigid bodies and calls specialized
 * contact functions.
 *
 * \b Note: This function assumes that the two given rigid bodies are close to each other and
 * therefore potentially colliding. No further coarse collision detection test (like checking
 * the axis-aligned bounding boxes of the two bodies) is applied!
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collide( BodyID b1, BodyID b2, CC& contacts )
{
   // Performing a collision test between the two rigid bodies
   switch( b1->getType() )
   {
      // Performing a collision test between a sphere and the second rigid body
      case sphereType:
         switch( b2->getType() ) {
            case sphereType:
               collideSphereSphere( static_body_cast<Sphere>( b1 ),
                                    static_body_cast<Sphere>( b2 ), contacts );
               break;
            case boxType:
               collideSphereBox( static_body_cast<Sphere>( b1 ),
                                 static_body_cast<Box>( b2 ), contacts );
               break;
            case capsuleType:
               collideSphereCapsule( static_body_cast<Sphere>( b1 ),
                                     static_body_cast<Capsule>( b2 ), contacts );
               break;
            case cylinderType:
               collideSphereCylinder( static_body_cast<Sphere>( b1 ),
                                      static_body_cast<Cylinder>( b2 ), contacts );
               break;
            case planeType:
               collideSpherePlane( static_body_cast<Sphere>( b1 ),
                                   static_body_cast<Plane>( b2 ), contacts );
               break;
            case triangleMeshType:
               collideSphereTMesh( static_body_cast<Sphere>( b1 ),
                                   static_body_cast<TriangleMesh>( b2 ), contacts );
               break;
            case unionType:
               collideSphereUnion( static_body_cast<Sphere>( b1 ),
                                   static_body_cast<Union>( b2 ), contacts );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a collision test between a box and the second rigid body
      case boxType:
         switch( b2->getType() ) {
            case sphereType:
               collideSphereBox( static_body_cast<Sphere>( b2 ),
                                 static_body_cast<Box>( b1 ), contacts );
               break;
            case boxType:
               collideBoxBox( static_body_cast<Box>( b1 ),
                              static_body_cast<Box>( b2 ), contacts );
               break;
            case capsuleType:
               collideBoxCapsule( static_body_cast<Box>( b1 ),
                                  static_body_cast<Capsule>( b2 ), contacts );
               break;
            case cylinderType:
               collideBoxCylinder( static_body_cast<Box>( b1 ),
                                   static_body_cast<Cylinder>( b2 ), contacts );
               break;
            case planeType:
               collideBoxPlane( static_body_cast<Box>( b1 ),
                                static_body_cast<Plane>( b2 ), contacts );
               break;
            case triangleMeshType:
               collideBoxTMesh( static_body_cast<Box>( b1 ),
                                static_body_cast<TriangleMesh>( b2 ), contacts );
               break;
            case unionType:
               collideBoxUnion( static_body_cast<Box>( b1 ),
                                static_body_cast<Union>( b2 ), contacts );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a collision test between a capsule and the second rigid body
      case capsuleType:
         switch( b2->getType() ) {
            case sphereType:
               collideSphereCapsule( static_body_cast<Sphere>( b2 ),
                                     static_body_cast<Capsule>( b1 ), contacts );
               break;
            case boxType:
               collideBoxCapsule( static_body_cast<Box>( b2 ),
                                  static_body_cast<Capsule>( b1 ), contacts );
               break;
            case capsuleType:
               collideCapsuleCapsule( static_body_cast<Capsule>( b1 ),
                                      static_body_cast<Capsule>( b2 ), contacts );
               break;
            case cylinderType:
               collideCapsuleCylinder( static_body_cast<Capsule>( b1 ),
                                       static_body_cast<Cylinder>( b2 ), contacts );
               break;
            case planeType:
               collideCapsulePlane( static_body_cast<Capsule>( b1 ),
                                    static_body_cast<Plane>( b2 ), contacts );
               break;
            case triangleMeshType:
               collideCapsuleTMesh( static_body_cast<Capsule>( b1 ),
                                    static_body_cast<TriangleMesh>( b2 ), contacts );
               break;
            case unionType:
               collideCapsuleUnion( static_body_cast<Capsule>( b1 ),
                                    static_body_cast<Union>( b2 ), contacts );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a collision test between a capsule and the second rigid body
      case cylinderType:
         switch( b2->getType() ) {
            case sphereType:
               collideSphereCylinder( static_body_cast<Sphere>( b2 ),
                                      static_body_cast<Cylinder>( b1 ), contacts );
               break;
            case boxType:
               collideBoxCylinder( static_body_cast<Box>( b2 ),
                                   static_body_cast<Cylinder>( b1 ), contacts );
               break;
            case capsuleType:
               collideCapsuleCylinder( static_body_cast<Capsule>( b2 ),
                                       static_body_cast<Cylinder>( b1 ), contacts );
               break;
            case cylinderType:
               collideCylinderCylinder( static_body_cast<Cylinder>( b1 ),
                                        static_body_cast<Cylinder>( b2 ), contacts );
               break;
            case planeType:
               collideCylinderPlane( static_body_cast<Cylinder>( b1 ),
                                     static_body_cast<Plane>( b2 ), contacts );
               break;
            case triangleMeshType:
               collideCylinderTMesh( static_body_cast<Cylinder>( b1 ),
                                     static_body_cast<TriangleMesh>( b2 ), contacts );
               break;
            case unionType:
               collideCylinderUnion( static_body_cast<Cylinder>( b1 ),
                                     static_body_cast<Union>( b2 ), contacts );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a collision test between a plane and the second rigid body
      case planeType:
         switch( b2->getType() ) {
            case sphereType:
               collideSpherePlane( static_body_cast<Sphere>( b2 ),
                                   static_body_cast<Plane>( b1 ), contacts );
               break;
            case boxType:
               collideBoxPlane( static_body_cast<Box>( b2 ),
                                static_body_cast<Plane>( b1 ), contacts );
               break;
            case capsuleType:
               collideCapsulePlane( static_body_cast<Capsule>( b2 ),
                                    static_body_cast<Plane>( b1 ), contacts );
               break;
            case cylinderType:
               collideCylinderPlane( static_body_cast<Cylinder>( b2 ),
                                     static_body_cast<Plane>( b1 ), contacts );
               break;
            case planeType:
               return;
               break;
            case triangleMeshType:
               collidePlaneTMesh( static_body_cast<Plane>( b1 ),
                                  static_body_cast<TriangleMesh>( b2 ), contacts );
               break;
            case unionType:
               collidePlaneUnion( static_body_cast<Plane>( b1 ),
                                  static_body_cast<Union>( b2 ), contacts );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a collision test between a triangle mesh and the second rigid body
      case triangleMeshType:
         switch( b2->getType() ) {
            case sphereType:
               collideSphereTMesh( static_body_cast<Sphere>( b2 ),
                                   static_body_cast<TriangleMesh>( b1 ), contacts );
               break;
            case boxType:
               collideBoxTMesh( static_body_cast<Box>( b2 ),
                                static_body_cast<TriangleMesh>( b1 ), contacts );
               break;
            case capsuleType:
               collideCapsuleTMesh( static_body_cast<Capsule>( b2 ),
                                   static_body_cast<TriangleMesh>( b1 ), contacts );
               break;
            case cylinderType:
               collideCylinderTMesh( static_body_cast<Cylinder>( b2 ),
                                     static_body_cast<TriangleMesh>( b1 ), contacts );
               break;
            case planeType:
               collidePlaneTMesh( static_body_cast<Plane>( b2 ),
                                  static_body_cast<TriangleMesh>( b1 ), contacts );
               break;
            case triangleMeshType:
               collideTMeshTMesh( static_body_cast<TriangleMesh>( b1 ),
                                  static_body_cast<TriangleMesh>( b2 ), contacts );
               break;
            case unionType:
               collideTMeshUnion( static_body_cast<TriangleMesh>( b1 ),
                                  static_body_cast<Union>( b2 ), contacts );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a collision test between a union and the second rigid body
      case unionType:
         switch( b2->getType() ) {
            case sphereType:
               collideSphereUnion( static_body_cast<Sphere>( b2 ),
                                   static_body_cast<Union>( b1 ), contacts );
               break;
            case boxType:
               collideBoxUnion( static_body_cast<Box>( b2 ),
                                static_body_cast<Union>( b1 ), contacts );
               break;
            case capsuleType:
               collideCapsuleUnion( static_body_cast<Capsule>( b2 ),
                                    static_body_cast<Union>( b1 ), contacts );
               break;
            case cylinderType:
               collideCylinderUnion( static_body_cast<Cylinder>( b2 ),
                                     static_body_cast<Union>( b1 ), contacts );
               break;
            case planeType:
               collidePlaneUnion( static_body_cast<Plane>( b2 ),
                                  static_body_cast<Union>( b1 ), contacts );
               break;
            case triangleMeshType:
               collideTMeshUnion( static_body_cast<TriangleMesh>( b2 ),
                                  static_body_cast<Union>( b1 ), contacts );
               break;
            case unionType:
               collideUnionUnion( static_body_cast<Union>( b1 ),
                                  static_body_cast<Union>( b2 ), contacts );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Treatment of unknown rigid body types
      default:
         std::ostringstream oss;
         oss << "Unknown body type (" << b1->getType() << ")!";
         throw std::runtime_error( oss.str() );
         break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between two colliding Sphere primitives.
 * \ingroup contact_generation
 *
 * \param s1 The first colliding sphere.
 * \param s2 The second colliding sphere.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * In the case of a collision between two spheres, only a single contact is generated. This
 * contact point lies on the connection between both centers of mass, relative to the radii
 * of the spheres.
 *
 * \image html collideSphereSphere.png
 * \image latex collideSphereSphere.eps "Collision between two spheres" width=236pt
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideSphereSphere( SphereID s1, SphereID s2, CC& contacts )
{
   // Force a defined order of collision detection across processes
   if( s2->getSystemID() < s1->getSystemID() )
      std::swap( s1, s2 );

   Vec3 normal( s1->getPosition() - s2->getPosition() );
   const real dist( normal.length() - s1->getRadius() - s2->getRadius() );

   if( dist < contactThreshold ) {
      normal.normalize();
      const real k( s2->getRadius() + real(0.5) * dist );
      const Vec3 gPos( s2->getPosition() + normal * k );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between sphere " << s1->getID()
             << " and sphere " << s2->getID() << " (dist=" << dist << ")";
      }

      contacts.addVertexFaceContact( s1, s2, gPos, normal, dist );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Sphere and a Box.
 * \ingroup contact_generation
 *
 * \param s The colliding sphere.
 * \param b The colliding box.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * In the case of a contact between a sphere and a box, only a single contact point is generated.
 * This contact point is calculated by a projection of the relative distance of the sphere's
 * center of mass from the box's center of mass on the body frame coordinate system of the box.
 * For all three axes, the projection is limited by the lengths of the sides of the box. This
 * projection is transfered to the global world frame, which gives the distance from the box's
 * center of mass to the contact point.
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideSphereBox( SphereID s, BoxID b, CC& contacts )
{
   const Vec3 l( real(0.5)*b->getLengths() );  // Box half side lengths
   const Vec3& spos( s->getPosition() );       // Global position of the sphere
   const Vec3& bpos( b->getPosition() );       // Global position of the box
   const Rot3& R( b->getRotation() );          // Rotation of the box

   bool outside( false );

   // Calculating the distance between the sphere and the box
   const Vec3 d( spos - bpos );

   // Calculating the the sphere/box distance in the box's frame of reference
   Vec3 p( d[0]*R[0] + d[1]*R[3] + d[2]*R[6],
           d[0]*R[1] + d[1]*R[4] + d[2]*R[7],
           d[0]*R[2] + d[1]*R[5] + d[2]*R[8] );

   // Projection of the x-component
   if     ( p[0] < -l[0] ) { p[0] = -l[0]; outside = true; }
   else if( p[0] >  l[0] ) { p[0] =  l[0]; outside = true; }

   // Projection of the y-component
   if     ( p[1] < -l[1] ) { p[1] = -l[1]; outside = true; }
   else if( p[1] >  l[1] ) { p[1] =  l[1]; outside = true; }

   // Projection of the z-component
   if     ( p[2] < -l[2] ) { p[2] = -l[2]; outside = true; }
   else if( p[2] >  l[2] ) { p[2] =  l[2]; outside = true; }

   // Special treatment if the sphere's center of mass is inside the box
   // In this case, a contact at the sphere's center of mass with a normal away from
   // the closest face of the box is generated.
   if( !outside )
   {
      real dist( std::fabs(p[0]) - l[0] );
      size_t face( 0 );

      for( size_t i=1; i<3; ++i ) {
         const real tmp( std::fabs(p[i]) - l[i] );
         if( dist < tmp ) {
            dist = tmp;
            face = i;
         }
      }

      // Calculating the normal direction of the contact
      Vec3 n;
      n[face] = ( p[face] > real(0) ) ? real(1) : real(-1);
      n = R * n;

      // Calculating the contact distance
      dist -= s->getRadius();

      // Creating a single contact between the sphere and the box
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between sphere " << s->getID()
             << " and box " << b->getID() << " (dist=" << dist << ")";
      }

      contacts.addVertexFaceContact( s, b, spos, n, dist );
      return;
   }

   const Vec3 q( R * p );  // Transformation from the projection to the global world frame
   const Vec3 n( d - q );  // Normal direction of the contact (pointing from the box to the sphere)

   const real dist( n.length() - s->getRadius() );  // Distance between the sphere and the box

   // Creating a single contact between the sphere and the box
   if( dist < contactThreshold )
   {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between sphere " << s->getID()
             << " and box " << b->getID() << " (dist=" << dist << ")";
      }

      contacts.addVertexFaceContact( s, b, bpos+q, n.getNormalized(), dist );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Sphere and a Capsule.
 * \ingroup contact_generation
 *
 * \param s The colliding sphere.
 * \param c The colliding capsule.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * In case of a contact between a sphere and a capsule, a single contact point is generated.
 * This contact point is calculated by first estimating a sphere within the capsule whose
 * center of mass is on the centerline of the capsule and closest to the center of mass of
 * the colliding sphere. After this, a sphere-sphere collision is performed between the
 * capsule representation and the colliding sphere.
 *
 * \image html collideSphereCapsule.png
 * \image latex collideSphereCapsule.eps "Collision between a sphere and a capsule" width=374pt
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideSphereCapsule( SphereID s, CapsuleID c, CC& contacts )
{
   const real length( real(0.5)*c->getLength() );  // Half cylinder length
   const Vec3& spos( s->getPosition() );           // Global position of the sphere
   const Vec3& cpos( c->getPosition() );           // Global position of the capsule
   const Rot3& R( c->getRotation() );              // Rotation of the capsule

   // Calculating the relative x-position of the sphere in the frame of the capsule
   real sx( R[0]*(spos[0]-cpos[0]) + R[3]*(spos[1]-cpos[1]) + R[6]*(spos[2]-cpos[2]) );

   // Calculation the center of the sphere representing the capsule
   if( sx > length ) {
      sx = length;
   }
   else if( sx < -length ) {
      sx = -length;
   }

   const Vec3 spos2( sx*R[0]+cpos[0], sx*R[3]+cpos[1], sx*R[6]+cpos[2] );

   // Performing a sphere-sphere collision between the colliding sphere and the
   // capsule representation
   Vec3 normal( spos - spos2 );
   const real dist( normal.length() - s->getRadius() - c->getRadius() );

   if( dist < contactThreshold ) {
      normal.normalize();
      const real k( c->getRadius() + real(0.5) * dist );
      const Vec3 gPos( spos2 + normal * k );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between sphere " << s->getID()
             << " and capsule " << c->getID() << " (dist=" << dist << ")";
      }

      contacts.addVertexFaceContact( s, c, gPos, normal, dist );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Sphere and a Cylinder.
 * \ingroup contact_generation
 *
 * \param s The colliding sphere.
 * \param c The colliding cylinder.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideSphereCylinder( SphereID s, CylinderID c, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(gjkEPAcollideHybrid< SphereID, CylinderID >(s, c, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (c) to object1 (s)
      contacts.addVertexFaceContact( s, c, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between sphere " << s->getID()
            << " and cylinder " << c->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Sphere and a Plane.
 * \ingroup contact_generation
 *
 * \param s The colliding sphere.
 * \param p The colliding plane.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * In the case of a contact between a sphere and a plane, only a single contact point is
 * generated. The contact point is calculated with a projection of the global position of
 * the sphere's center of mass and the plane's normal, from which follows the distance between
 * the sphere and the plane.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideSpherePlane( SphereID s, PlaneID p, CC& contacts )
{
   const real k( trans( p->getNormal() ) * s->getPosition() );
   const real dist( k - s->getRadius() - p->getDisplacement() );

   if( dist < contactThreshold ) {
      const Vec3 gPos( s->getPosition() - ( s->getRadius() + dist ) * p->getNormal() );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between sphere " << s->getID()
             << " and plane " << p->getID() << " (dist=" << dist << ")";
      }

      contacts.addVertexFaceContact( s, p, gPos, p->getNormal(), dist );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Sphere and a Triangle Mesh.
 * \ingroup contact_generation
 *
 * \param s The colliding sphere.
 * \param m The colliding triangle mesh.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideSphereTMesh( SphereID s, TriangleMeshID m, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;
   
   if(gjkEPAcollideHybrid< SphereID >(s, m, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (m) to object1 (s)
      contacts.addVertexFaceContact( s, m, contactPoint, normal, penetrationDepth );
      if(normal[2] < 0) {
         //std::cerr << normal << "\t depth=" << penetrationDepth << std::endl;
      }
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between sphere " << s->getID()
            << " and triangle mesh " << m->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Sphere and a Union.
 * \ingroup contact_generation
 *
 * \param s The colliding sphere.
 * \param u The colliding union.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between a sphere and a union is treated as collisions between the sphere
 * and all the subbodies of the union.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideSphereUnion( SphereID s, UnionID u, CC& contacts )
{
   for( Union::Iterator it=u->begin(); it!=u->end(); ++it ) {
      collide( s, *it, contacts );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between two colliding Box primitives.
 * \ingroup contact_generation
 *
 * \param b1 The first colliding box.
 * \param b2 The second colliding box.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 *
 * \image html collideBoxBox.png
 * \image latex collideBoxBox.eps "Collision between two boxes" width=800pt
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideBoxBox( BoxID b1, BoxID b2, CC& contacts )
{
   // Force a defined order of collision detection across processes
   if( b2->getSystemID() < b1->getSystemID() )
      std::swap( b1, b2 );

   // TODO: Performance optimization

   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Fine collision detection between box " << b1->getID() << " and box " << b2->getID();
   }

   real maxDepth( -inf );
   bool invertNormal( false );
   unsigned int contactCase( 0 );
   Vec3 contactNormal;

   const Rot3& R1( b1->getRotation() );
   const Rot3& R2( b2->getRotation() );

   // Calculating the rotation of box 2 relative to the orientation of box 1
   const Rot3 b2_rR( trans( b1->getRotation() ) * b2->getRotation() );

   // Calculating the absolute values of the relative rotation
   const Mat3 b2_rQ( fabs( b2_rR ) );

   // Calculating the position of box 2 relative to the orientation of box 1
   const Vec3 b2_rPos( trans( b1->getRotation() ) * ( b2->getPosition() - b1->getPosition() ) );

   // Calculating the half Lengths of both boxes
   const real hl1[] = { real(0.5) * b1->getLengths()[0],
                        real(0.5) * b1->getLengths()[1],
                        real(0.5) * b1->getLengths()[2] };
   const real hl2[] = { real(0.5) * b2->getLengths()[0],
                        real(0.5) * b2->getLengths()[1],
                        real(0.5) * b2->getLengths()[2] };


   //----- Testing the three axes of box 1 -----

   real term1;

   // l = au
   term1 = std::fabs(b2_rPos[0]) - ( hl1[0] + hl2[0]*b2_rQ[0] + hl2[1]*b2_rQ[1] + hl2[2]*b2_rQ[2] );

   if( term1 > contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 1 (l = au) failed!";
      }
      return;
   }
   else /* if( term1 > maxDepth ) */ {
      maxDepth      = term1;
      contactNormal = Vec3( R1[0], R1[3], R1[6] );
      contactCase   = 1;
      invertNormal  = ( b2_rPos[0] < real(0) );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Contact test 1 succeeded!\n";
         log << "            maxDepth = " << maxDepth;
      }
   }

   // l = av
   term1 = std::fabs(b2_rPos[1]) - ( hl1[1] + hl2[0]*b2_rQ[3] + hl2[1]*b2_rQ[4] + hl2[2]*b2_rQ[5] );

   if( term1 > contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 2 (l = av) failed!";
      }
      return;
   }
   else if( term1 > maxDepth ) {
      maxDepth      = term1;
      contactNormal = Vec3( R1[1], R1[4], R1[7] );
      contactCase   = 2;
      invertNormal  = ( b2_rPos[1] < real(0) );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Contact test 2 succeeded!\n";
         log << "            maxDepth = " << maxDepth;
      }
   }

   // l = aw
   term1 = std::fabs(b2_rPos[2]) - ( hl1[2] + hl2[0]*b2_rQ[6] + hl2[1]*b2_rQ[7] + hl2[2]*b2_rQ[8] );

   if( term1 > contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 3 (l = aw) failed!";
      }
      return;
   }
   else if( term1 > maxDepth ) {
      maxDepth      = term1;
      contactNormal = Vec3( R1[2], R1[5], R1[8] );
      contactCase   = 3;
      invertNormal  = ( b2_rPos[2] < real(0) );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Contact test 3 succeeded!\n";
         log << "            maxDepth = " << maxDepth;
      }
   }


   //----- Testing the three axes of box 2 -----

   real term2;

   // l = bu
   term1 = b2_rPos[0]*b2_rR[0] + b2_rPos[1]*b2_rR[3] + b2_rPos[2]*b2_rR[6];
   term2 = std::fabs(term1) - ( hl1[0]*b2_rQ[0] + hl1[1]*b2_rQ[3] + hl1[2]*b2_rQ[6] + hl2[0] );

   if( term2 > contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 4 (l = bu) failed!";
      }
      return;
   }
   else if( term2 > maxDepth ) {
      maxDepth      = term2;
      contactNormal = Vec3( R2[0], R2[3], R2[6] );
      contactCase   = 4;
      invertNormal  = ( term1 < real(0) );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Contact test 4 succeeded!\n";
         log << "            maxDepth = " << maxDepth;
      }
   }

   // l = bv
   term1 = b2_rPos[0]*b2_rR[1] + b2_rPos[1]*b2_rR[4] + b2_rPos[2]*b2_rR[7];
   term2 = std::fabs(term1) - ( hl1[0]*b2_rQ[1] + hl1[1]*b2_rQ[4] + hl1[2]*b2_rQ[7] + hl2[1] );

   if( term2 > contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 5 (l = bv) failed!";
      }
      return;
   }
   else if( term2 > maxDepth ) {
      maxDepth      = term2;
      contactNormal = Vec3( R2[1], R2[4], R2[7] );
      contactCase   = 5;
      invertNormal  = ( term1 < real(0) );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Contact test 5 succeeded!\n";
         log << "            maxDepth = " << maxDepth;
      }
   }

   // l = bw
   term1 = b2_rPos[0]*b2_rR[2] + b2_rPos[1]*b2_rR[5] + b2_rPos[2]*b2_rR[8];
   term2 = std::fabs(term1) - ( hl1[0]*b2_rQ[2] + hl1[1]*b2_rQ[5] + hl1[2]*b2_rQ[8] + hl2[2] );

   if( term2 > contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 6 (l = bw) failed!";
      }
      return;
   }
   else if( term2 > maxDepth ) {
      maxDepth      = term2;
      contactNormal = Vec3( R2[2], R2[5], R2[8] );
      contactCase   = 6;
      invertNormal  = ( term1 < real(0) );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Contact test 6 succeeded!\n";
         log << "            maxDepth = " << maxDepth;
      }
   }


   //----- Testing the all nine combinations of the axes of the two boxes -----

   real term3;
   real sum;
   real length;
   Vec3 normal_c;

   // l = au x bu
   term1 = b2_rPos[2] * b2_rR[3] - b2_rPos[1] * b2_rR[6];
   term2 = hl1[1] * b2_rQ[6] + hl1[2] * b2_rQ[3];
   term3 = hl2[2] * b2_rQ[1] + hl2[1] * b2_rQ[2];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 7 (l = au x bu) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[6]) + sq(b2_rR[3]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( 0, -b2_rR[6]/length, b2_rR[3]/length );
            contactCase  = 7;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 7 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = au x bv
   term1 = b2_rPos[2] * b2_rR[4] - b2_rPos[1] * b2_rR[7];
   term2 = hl1[1] * b2_rQ[7] + hl1[2] * b2_rQ[4];
   term3 = hl2[0] * b2_rQ[2] + hl2[2] * b2_rQ[0];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 8 (l = au x bv) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[7]) + sq(b2_rR[4]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( 0, -b2_rR[7]/length, b2_rR[4]/length );
            contactCase  = 8;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 8 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = au x bw
   term1 = b2_rPos[2] * b2_rR[5] - b2_rPos[1] * b2_rR[8];
   term2 = hl1[1] * b2_rQ[8] + hl1[2] * b2_rQ[5];
   term3 = hl2[1] * b2_rQ[0] + hl2[0] * b2_rQ[1];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 9 (l = au x bw) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[8]) + sq(b2_rR[5]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( 0, -b2_rR[8]/length, b2_rR[5]/length );
            contactCase  = 9;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 9 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = av x bu
   term1 = b2_rPos[0] * b2_rR[6] - b2_rPos[2] * b2_rR[0];
   term2 = hl1[2] * b2_rQ[0] + hl1[0] * b2_rQ[6];
   term3 = hl2[2] * b2_rQ[4] + hl2[1] * b2_rQ[5];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 10 (l = av x bu) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[6]) + sq(b2_rR[0]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( b2_rR[6]/length, 0, -b2_rR[0]/length );
            contactCase  = 10;
            invertNormal = ( term1 < real(0) ) ;

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 10 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = av x bv
   term1 = b2_rPos[0] * b2_rR[7] - b2_rPos[2] * b2_rR[1];
   term2 = hl1[2] * b2_rQ[1] + hl1[0] * b2_rQ[7];
   term3 = hl2[0] * b2_rQ[5] + hl2[2] * b2_rQ[3];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 11 (l = av x bv) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[7]) + sq(b2_rR[1]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( b2_rR[7]/length, 0, -b2_rR[1]/length );
            contactCase  = 11;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 11 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = av x bw
   term1 = b2_rPos[0] * b2_rR[8] - b2_rPos[2] * b2_rR[2];
   term2 = hl1[2] * b2_rQ[2] + hl1[0] * b2_rQ[8];
   term3 = hl2[1] * b2_rQ[3] + hl2[0] * b2_rQ[4];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 12 (l = av x bw) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[8]) + sq(b2_rR[2]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( b2_rR[8]/length, 0, -b2_rR[2]/length );
            contactCase  = 12;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 12 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = aw x bu
   term1 = b2_rPos[1] * b2_rR[0] - b2_rPos[0] * b2_rR[3];
   term2 = hl1[0] * b2_rQ[3] + hl1[1] * b2_rQ[0];
   term3 = hl2[2] * b2_rQ[7] + hl2[1] * b2_rQ[8];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 13 (l = aw x bu) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[3]) + sq(b2_rR[0]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( -b2_rR[3]/length, b2_rR[0]/length, 0 );
            contactCase  = 13;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 13 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = aw x bv
   term1 = b2_rPos[1] * b2_rR[1] - b2_rPos[0] * b2_rR[4];
   term2 = hl1[0] * b2_rQ[4] + hl1[1] * b2_rQ[1];
   term3 = hl2[0] * b2_rQ[8] + hl2[2] * b2_rQ[6];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 14 (l = aw x bv) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[4]) + sq(b2_rR[1]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( -b2_rR[4]/length, b2_rR[1]/length, 0 );
            contactCase  = 14;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 14 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }

   // l = aw x bw
   term1 = b2_rPos[1] * b2_rR[2] - b2_rPos[0] * b2_rR[5];
   term2 = hl1[0] * b2_rQ[5] + hl1[1] * b2_rQ[2];
   term3 = hl2[1] * b2_rQ[6] + hl2[0] * b2_rQ[7];
   sum   = std::fabs(term1) - ( term2 + term3 );

   if( std::fabs(term1) > term2 + term3 + contactThreshold ) {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Test 15 (l = aw x bw) failed!";
      }
      return;
   }
   else {
      length = std::sqrt( sq(b2_rR[5]) + sq(b2_rR[2]) );
      if( length > epsilon ) {
         sum /= length;
         if( sum > maxDepth && std::fabs( sum - maxDepth ) > accuracy ) {
            maxDepth     = sum;
            normal_c     = Vec3( -b2_rR[5]/length, b2_rR[2]/length, 0 );
            contactCase  = 15;
            invertNormal = ( term1 < real(0) );

            pe_LOG_DETAIL_SECTION( log ) {
               log << "         Contact test 15 succeeded!\n";
               log << "            maxDepth = " << maxDepth;
            }
         }
      }
   }


   if( contactCase == 0 ) {
      return;
   }

   if( contactCase > 6 ) {
      contactNormal =  R1 * normal_c;
   }

   if( invertNormal ) {
      contactNormal = -contactNormal;
   }


   // TEST
   pe_LOG_DETAIL_SECTION( log ) {
      log << "         Selected contact case = " << contactCase << "\n"
          << "         Contact normal = " << contactNormal << "\n"
          << "         Normal invertion? " << invertNormal;
   }


   //----- Treating edge/edge collisions -----

   if( contactCase > 6 )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "         Treating edge/edge collision between box " << b1->getID()
             << " and " << b2->getID() << "...";
      }

      Vec3 pB1( b1->getPosition() );
      Vec3 sign = trans(R1) * contactNormal;
      //Vec3 sign = normal_c;
      for( unsigned int i=0; i<3; ++i ) {
         sign[i] = ( sign[i]>0 ) ? ( hl1[i] ) : ( -hl1[i] );
      }
      const Vec3 tmp1( sign );
      pB1 += R1 * sign;

      Vec3 pB2 = b2->getPosition();
      sign = trans(R2) * contactNormal;
      for( int i=0; i<3; ++i ) {
         sign[i] = ( sign[i]>0 ) ? ( -hl2[i] ) : ( hl2[i] );
      }
      const Vec3 tmp2( sign );
      pB2 += R2 * sign;

      Vec3 ua, ub;
      for( int i=0; i<3; i++ ) {
         ua[i] = R1[(contactCase-7)/3 + i*3];
         ub[i] = R2[(contactCase-7)%3 + i*3];
      }

      real s, t;
      intersectLines( pB1, ua, pB2, ub, s, t );
      pB1 += s * ua;
      pB2 += t * ub;
      Vec3 gpos = real(0.5) * ( pB1 + pB2 );
      Vec3 normal = ( ua % ub ).getNormalized();

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            box A (b1) = " << b1->getID() << "\n"
             << "            box B (b2) = " << b2->getID() << "\n"
             << "            normal_c   = " << normal_c << "\n"
             << "            tmp1 (for pB1) = " << tmp1 << "\n"
             << "            tmp2 (for pB2) = " << tmp2 << "\n"
             << "            pB1  = " << pB1 << "\n"
             << "            pB2  = " << pB2 << "\n"
             << "            gpos = " << gpos << "\n"
             << "            contactNormal from A to B = " << contactNormal << "\n"
             << "            contactNormal from B to A = " << -contactNormal << "\n"
             << "            ua = " << ua << "\n"
             << "            ub = " << ub << "\n"
             << "            ua x ub = " << ua % ub << "\n"
             << "            normal = " << normal << "\n\n";
      }

      // TEST
      if( trans(normal)*contactNormal < real(0) ) {
         pe_LOG_DETAIL_SECTION( log ) {
            log << "         Inverting ub!\n"
                << "         ua = " << ua << "\n"
                << "         ub = " << ub;
         }
         ub = -ub;
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Edge/edge contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << maxDepth << ")";
      }

      contacts.addEdgeEdgeContact( b2, b1, gpos, contactNormal, ub, ua, maxDepth );
      return;
   }


   //----- Treating vertex/face collisions -----

   pe_LOG_DETAIL_SECTION( log ) {
      log << "         Treating vertex/face collision...";
   }

   const real* hla( hl1 );
   const real* hlb( hl2 );

   if( contactCase > 3 ) {
      std::swap( b1, b2 );
      std::swap( hla, hlb );
      contactNormal = -contactNormal;
   }

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            Box A = " << b1->getID() << "\n"
          << "            Box B = " << b2->getID();
   }

   const Rot3& Ra( b1->getRotation() );
   const Rot3& Rb( b2->getRotation() );


   // Calculating the relative contact normal in the body frame of bx A
   const Vec3 normala( trans(Ra) * contactNormal );

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            Relative contact normal in the body frame of A = " << normala;
   }


   // Calculating the relative contact normal in the body frame of box B
   const Vec3 normalb( trans(Rb) * contactNormal );

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            Relative contact normal in the body frame of B = " << normalb;
   }


   // Estimating the collides face of box B
   unsigned int xb(0), yb(0), zb(0);

   if( std::fabs( normalb[0] ) > std::fabs( normalb[1] ) ) {
      if( std::fabs( normalb[0] ) > std::fabs( normalb[2] ) ) {
         xb = 0; yb = 1; zb = 2;
      }
      else {
         xb = 2; yb = 0; zb = 1;
      }
   }
   else if( std::fabs( normalb[1] ) > std::fabs( normalb[2] ) ) {
      xb = 1; yb = 2; zb = 0;
   }
   else {
      xb = 2; yb = 0; zb = 1;
   }


   // Estimating the colliding face of box A
   unsigned int xa(0), ya(0), za(0);

   if( contactCase < 4 ) {
      xa = contactCase - 1;
   }
   else {
      xa = contactCase - 4;
   }
   if( xa == 0 ) {
      ya = 1; za = 2;
   }
   else if( xa == 1 ) {
      ya = 2; za = 0;
   }
   else {
      ya = 0; za = 1;
   }

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            Colliding face of box A:  xa = " << xa << " , ya = " << ya << " , za = " << za
          << "            Colliding face of box B:  xb = " << xb << " , yb = " << yb << " , zb = " << zb;
   }


   // Calculating the four vertices of the colliding face of box B in the frame of box B
   Vec3 vertexb[4];

   vertexb[0][xb] = ( normalb[xb] > real(0) )?( -hlb[xb] ):( hlb[xb] );
   vertexb[0][yb] = -hlb[yb];
   vertexb[0][zb] = -hlb[zb];

   vertexb[1][xb] = ( normalb[xb] > real(0) )?( -hlb[xb] ):( hlb[xb] );
   vertexb[1][yb] = hlb[yb];
   vertexb[1][zb] = -hlb[zb];

   vertexb[2][xb] = ( normalb[xb] > real(0) )?( -hlb[xb] ):( hlb[xb] );
   vertexb[2][yb] = -hlb[yb];
   vertexb[2][zb] = hlb[zb];

   vertexb[3][xb] = ( normalb[xb] > real(0) )?( -hlb[xb] ):( hlb[xb] );
   vertexb[3][yb] = hlb[yb];
   vertexb[3][zb] = hlb[zb];

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            The four colliding vertices of box B:\n";
      for( unsigned int i=0; i<4; ++i ) {
         log << "               Vertex " << i+1 << ": " << vertexb[i] << "\n";
      }
   }


   // Translating the four vertices to the body frame of box A
   const Vec3 ab( b2->getPosition() - b1->getPosition() );

   Vec3 vertexba[] = { trans(Ra) * ( ab + Rb*vertexb[0] ),
                       trans(Ra) * ( ab + Rb*vertexb[1] ),
                       trans(Ra) * ( ab + Rb*vertexb[2] ),
                       trans(Ra) * ( ab + Rb*vertexb[3] ) };

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            The four colliding vertices of box B relative to the body frame of A:\n";
      for( unsigned int i=0; i<4; ++i ) {
         log << "               Vertex " << i+1 << ": " << vertexba[i] << "\n";
      }
   }


   //----- Calculating the line/line intersections between the two colliding faces -----

   const real offseta( ( normala[xa] > real(0) )?( hla[xa] ):( -hla[xa] ) );
   real s, dist, tmp;

   // Intersection between v0--v1 with hla[ya]
   if( ( vertexba[0][ya] > hla[ya] ) ^ ( vertexba[1][ya] > hla[ya] ) )
   {
      s = ( hla[ya] - vertexba[0][ya] ) / ( vertexba[1][ya] - vertexba[0][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 1\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][za] + s*( vertexba[1][za] - vertexba[0][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[1][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v0--v1 with -hla[ya]
   if( ( vertexba[0][ya] > -hla[ya] ) ^ ( vertexba[1][ya] > -hla[ya] ) )
   {
      s = ( -hla[ya] - vertexba[0][ya] ) / ( vertexba[1][ya] - vertexba[0][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 2\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][za] + s*( vertexba[1][za] - vertexba[0][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[1][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = -hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v0--v1 with hla[za]
   if( ( vertexba[0][za] > hla[za] ) ^ ( vertexba[1][za] > hla[za] ) )
   {
      s = ( hla[za] - vertexba[0][za] ) / ( vertexba[1][za] - vertexba[0][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 3\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][ya] + s*( vertexba[1][ya] - vertexba[0][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[1][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v0--v1 with -hla[za]
   if( ( vertexba[0][za] > -hla[za] ) ^ ( vertexba[1][za] > -hla[za] ) )
   {
      s = ( -hla[za] - vertexba[0][za] ) / ( vertexba[1][za] - vertexba[0][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 4\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][ya] + s*( vertexba[1][ya] - vertexba[0][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[1][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = -hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }


   // Intersection between v0--v2 with hla[ya]
   if( ( vertexba[0][ya] > hla[ya] ) ^ ( vertexba[2][ya] > hla[ya] ) )
   {
      s = ( hla[ya] - vertexba[0][ya] ) / ( vertexba[2][ya] - vertexba[0][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 5\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][za] + s*( vertexba[2][za] - vertexba[0][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[2][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v0--v2 with -hla[ya]
   if( ( vertexba[0][ya] > -hla[ya] ) ^ ( vertexba[2][ya] > -hla[ya] ) )
   {
      s = ( -hla[ya] - vertexba[0][ya] ) / ( vertexba[2][ya] - vertexba[0][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 6\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][za] + s*( vertexba[2][za] - vertexba[0][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[2][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = -hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v0--v2 with hla[za]
   if( ( vertexba[0][za] > hla[za] ) ^ ( vertexba[2][za] > hla[za] ) )
   {
      s = ( hla[za] - vertexba[0][za] ) / ( vertexba[2][za] - vertexba[0][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 7\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][ya] + s*( vertexba[2][ya] - vertexba[0][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[2][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v0--v2 with -hla[za]
   if( ( vertexba[0][za] > -hla[za] ) ^ ( vertexba[2][za] > -hla[za] ) )
   {
      s = ( -hla[za] - vertexba[0][za] ) / ( vertexba[2][za] - vertexba[0][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 8\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[0][ya] + s*( vertexba[2][ya] - vertexba[0][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[0][xa] + s*( vertexba[2][xa] - vertexba[0][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = -hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }


   // Intersection between v3--v1 with hla[ya]
   if( ( vertexba[3][ya] > hla[ya] ) ^ ( vertexba[1][ya] > hla[ya] ) )
   {
      s = ( hla[ya] - vertexba[3][ya] ) / ( vertexba[1][ya] - vertexba[3][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 9\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][za] + s*( vertexba[1][za] - vertexba[3][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[1][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v3--v1 with -hla[ya]
   if( ( vertexba[3][ya] > -hla[ya] ) ^ ( vertexba[1][ya] > -hla[ya] ) )
   {
      s = ( -hla[ya] - vertexba[3][ya] ) / ( vertexba[1][ya] - vertexba[3][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 10\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][za] + s*( vertexba[1][za] - vertexba[3][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[1][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = -hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v3--v1 with hla[za]
   if( ( vertexba[3][za] > hla[za] ) ^ ( vertexba[1][za] > hla[za] ) )
   {
      s = ( hla[za] - vertexba[3][za] ) / ( vertexba[1][za] - vertexba[3][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 11\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][ya] + s*( vertexba[1][ya] - vertexba[3][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[1][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v3--v1 with -hla[za]
   if( ( vertexba[3][za] > -hla[za] ) ^ ( vertexba[1][za] > -hla[za] ) )
   {
      s = ( -hla[za] - vertexba[3][za] ) / ( vertexba[1][za] - vertexba[3][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 12\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][ya] + s*( vertexba[1][ya] - vertexba[3][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[1][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = -hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }


   // Intersection between v3--v2 with hla[ya]
   if( ( vertexba[3][ya] > hla[ya] ) ^ ( vertexba[2][ya] > hla[ya] ) )
   {
      s = ( hla[ya] - vertexba[3][ya] ) / ( vertexba[2][ya] - vertexba[3][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 13\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][za] + s*( vertexba[2][za] - vertexba[3][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[2][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v3--v2 with -hla[ya]
   if( ( vertexba[3][ya] > -hla[ya] ) ^ ( vertexba[2][ya] > -hla[ya] ) )
   {
      s = ( -hla[ya] - vertexba[3][ya] ) / ( vertexba[2][ya] - vertexba[3][ya] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 14\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][za] + s*( vertexba[2][za] - vertexba[3][za] ) ) < hla[za] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[2][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = -hla[ya];
            posa[za] = tmp;

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v3--v2 with hla[za]
   if( ( vertexba[3][za] > hla[za] ) ^ ( vertexba[2][za] > hla[za] ) )
   {
      s = ( hla[za] - vertexba[3][za] ) / ( vertexba[2][za] - vertexba[3][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 15\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][ya] + s*( vertexba[2][ya] - vertexba[3][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[2][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }

   // Intersection between v3--v2 with -hla[za]
   if( ( vertexba[3][za] > -hla[za] ) ^ ( vertexba[2][za] > -hla[za] ) )
   {
      s = ( -hla[za] - vertexba[3][za] ) / ( vertexba[2][za] - vertexba[3][za] );

      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 16\n";
         log << "               s = " << s;
      }

      if( s > real(0) && s < real(1) &&
          std::fabs( tmp = vertexba[3][ya] + s*( vertexba[2][ya] - vertexba[3][ya] ) ) < hla[ya] )
      {
         dist = std::fabs( vertexba[3][xa] + s*( vertexba[2][xa] - vertexba[3][xa] ) ) - hla[xa];
         if( dist < contactThreshold )
         {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "      Vertex/face contact created between box " << b1->getID()
                   << " and box " << b2->getID() << " (dist=" << dist << ")";
            }

            Vec3 posa;
            posa[xa] = offseta;
            posa[ya] = tmp;
            posa[za] = -hla[za];

            const Vec3 gpos( b1->pointFromBFtoWF( posa ) );
            contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
         }
      }
   }


   //----- Calculating contact points for the vertices of box B -----

   if( std::fabs(vertexba[0][ya]) <= hla[ya] && std::fabs(vertexba[0][za]) <= hla[za] &&
       ( dist = std::fabs(vertexba[0][xa]) - hla[xa] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 17";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b1->pointFromBFtoWF( vertexba[0] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }

   if( std::fabs(vertexba[1][ya]) <= hla[ya] && std::fabs(vertexba[1][za]) <= hla[za] &&
       ( dist = std::fabs(vertexba[1][xa]) - hla[xa] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 18";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b1->pointFromBFtoWF( vertexba[1] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }

   if( std::fabs(vertexba[2][ya]) <= hla[ya] && std::fabs(vertexba[2][za]) <= hla[za] &&
       ( dist = std::fabs(vertexba[2][xa]) - hla[xa] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 19";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b1->pointFromBFtoWF( vertexba[2] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }

   if( std::fabs(vertexba[3][ya]) <= hla[ya] && std::fabs(vertexba[3][za]) <= hla[za] &&
       ( dist = std::fabs(vertexba[3][xa]) - hla[xa] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 20";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b1->pointFromBFtoWF( vertexba[3] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }


   //----- Calculating contact points for the vertices of box A -----

   // Calculating the four vertices of the colliding face of box B in the frame of box B
   Vec3 vertexa[4];

   vertexa[0][xa] = ( normala[xa] > real(0) )?( hla[xa] ):( -hla[xa] );
   vertexa[0][ya] = -hla[ya];
   vertexa[0][za] = -hla[za];

   vertexa[1][xa] = ( normala[xa] > real(0) )?( hla[xa] ):( -hla[xa] );
   vertexa[1][ya] = hla[ya];
   vertexa[1][za] = -hla[za];

   vertexa[2][xa] = ( normala[xa] > real(0) )?( hla[xa] ):( -hla[xa] );
   vertexa[2][ya] = -hla[ya];
   vertexa[2][za] = hla[za];

   vertexa[3][xa] = ( normala[xa] > real(0) )?( hla[xa] ):( -hla[xa] );
   vertexa[3][ya] = hla[ya];
   vertexa[3][za] = hla[za];

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            The four colliding vertices of box A:\n";
      for( unsigned int i=0; i<4; ++i ) {
         log << "               Vertex " << i+1 << ": " << vertexa[i] << "\n";
      }
   }


   // Translating the four vertices to the body frame of box B
   const Vec3 ba( b1->getPosition() - b2->getPosition() );

   Vec3 vertexab[] = { trans(Rb) * ( ba + Ra*vertexa[0] ),
                       trans(Rb) * ( ba + Ra*vertexa[1] ),
                       trans(Rb) * ( ba + Ra*vertexa[2] ),
                       trans(Rb) * ( ba + Ra*vertexa[3] ) };

   pe_LOG_DETAIL_SECTION( log ) {
      log << "            The four colliding vertices of box A relative to the body frame of B:\n";
      for( unsigned int i=0; i<4; ++i ) {
         log << "               Vertex " << i+1 << ": " << vertexab[i] << "\n";
      }
   }


   // In order to avoid vertex/vertex-contacts to be generated twice, the evaluation whether
   // a contact point is generated for a vertex of A has the additional requirement, that
   // the vertex of A does not coincide with a vertex of B.

   if( ( ( std::fabs(vertexab[0][yb]) <= hlb[yb] && std::fabs(vertexab[0][zb]) <  hlb[zb] ) ||
         ( std::fabs(vertexab[0][yb]) <  hlb[yb] && std::fabs(vertexab[0][zb]) <= hlb[zb] ) ) &&
       ( dist = std::fabs(vertexab[0][xb]) - hlb[xb] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 21";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b2->pointFromBFtoWF( vertexab[0] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }

   if( ( ( std::fabs(vertexab[1][yb]) <= hlb[yb] && std::fabs(vertexab[1][zb]) <  hlb[zb] ) ||
         ( std::fabs(vertexab[1][yb]) <  hlb[yb] && std::fabs(vertexab[1][zb]) <= hlb[zb] ) ) &&
       ( dist = std::fabs(vertexab[1][xb]) - hlb[xb] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 22";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b2->pointFromBFtoWF( vertexab[1] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }

   if( ( ( std::fabs(vertexab[2][yb]) <= hlb[yb] && std::fabs(vertexab[2][zb]) <  hlb[zb] ) ||
         ( std::fabs(vertexab[2][yb]) <  hlb[yb] && std::fabs(vertexab[2][zb]) <= hlb[zb] ) ) &&
       ( dist = std::fabs(vertexab[2][xb]) - hlb[xb] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 23";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b2->pointFromBFtoWF( vertexab[2] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }

   if( ( ( std::fabs(vertexab[3][yb]) <= hlb[yb] && std::fabs(vertexab[3][zb]) <  hlb[zb] ) ||
         ( std::fabs(vertexab[3][yb]) <  hlb[yb] && std::fabs(vertexab[3][zb]) <= hlb[zb] ) ) &&
       ( dist = std::fabs(vertexab[3][xb]) - hlb[xb] ) < contactThreshold )
   {
      pe_LOG_DETAIL_SECTION( log ) {
         log << "            Treating case 24";
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Vertex/face contact created between box " << b1->getID()
             << " and box " << b2->getID() << " (dist=" << dist << ")";
      }

      const Vec3 gpos( b2->pointFromBFtoWF( vertexab[3] ) );
      contacts.addVertexFaceContact( b2, b1, gpos, contactNormal, dist );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Box and a Capsule.
 * \ingroup contact_generation
 *
 * \param b The colliding box.
 * \param c The colliding capsule.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * During the collision of a box and a capsule, a maximum of two contact points is generated.
 * For most relative orientations between the box and the capsule, a single contact point is
 * generated at the closest point between the two rigid bodies (see pe::getClosestLineBoxPoints).
 * However, in case the capsule is parallel to one of the faces of the box it is possible
 * that two contact points are required.
 *
 * \image html collideBoxCapsule.png
 * \image latex collideBoxCapsule.eps "Collision between a box and a capsule" width=800pt
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideBoxCapsule( BoxID b, CapsuleID c, CC& contacts )
{
   const Rot3& R( c->getRotation() );


   //----- Calculating the first contact point between the capsule and the box -----

   // Computing the displacement of the spherical caps of the capsule in world space coordinates
   const Vec3 c_up( real(0.5)*c->getLength()*R[0],
                    real(0.5)*c->getLength()*R[3],
                    real(0.5)*c->getLength()*R[6] );

   // Computing the centers of the spherical caps in world space coordinates
   const Vec3 up  ( c->getPosition()+c_up );
   const Vec3 down( c->getPosition()-c_up );

   // Computing the closest points on the up-down-axis of the cylinder and the box
   Vec3 cp1, bp1;
   getClosestLineBoxPoints( up, down, b->getPosition(), b->getRotation(), b->getLengths(), cp1, bp1 );

   Vec3 normal1( bp1 - cp1 );
   real dist1( normal1.length() - c->getRadius() );

   // Checking the distance between the capsule and the box
   if( dist1 > contactThreshold ) return;

   // Calculating the contact normal and the position of the closest contact point
   normal1.normalize();
   const Vec3 gpos1( bp1 - real(0.5)*dist1*normal1 );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "      Contact created between box " << b->getID() << " and capsule " << c->getID()
          << " (dist=" << dist1 << ", gpos=" << gpos1 << ", normal=" << normal1 << ")";
   }
   contacts.addVertexFaceContact( b, c, gpos1, normal1, dist1 );


   //----- Calculating the second contact point between the capsule and the box -----

   // Computing the closest points on the down-up-axis of the cylinder and the box
   Vec3 cp2, bp2;
   getClosestLineBoxPoints( down, up, b->getPosition(), b->getRotation(), b->getLengths(), cp2, bp2 );

   // Early exit in case the same contact point is found again
   if( cp1 == cp2 ) return;

   Vec3 normal2( bp2 - cp2 );
   real dist2( normal2.length() - c->getRadius() );

   // Calculating the contact normal and the position of the closest contact point
   normal2.normalize();
   const Vec3 gpos2( bp2 - real(0.5)*dist2*normal2 );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "      Contact created between box " << b->getID() << " and capsule " << c->getID()
          << " (dist=" << dist2 << ", gpos=" << gpos2 << ", normal=" << normal2 << ")";
   }
   contacts.addVertexFaceContact( b, c, gpos2, normal2, dist2 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Box and a Cylinder.
 * \ingroup contact_generation
 *
 * \param b The colliding box.
 * \param c The colliding cylinder.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideBoxCylinder( BoxID b, CylinderID c, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(gjkEPAcollideHybrid< BoxID, CylinderID >(b, c, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (c) to object1 (b)
      contacts.addVertexFaceContact( b, c, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between box " << b->getID()
            << " and cylinder " << c->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Box and a Plane.
 * \ingroup contact_generation
 *
 * \param b The colliding box.
 * \param p The colliding plane.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * For each corner of the box, the distance to the plane is calculated with a projection on
 * the plane's normal. For each corner, which lies on or inside the plane, a contact point is
 * generated.
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideBoxPlane( BoxID b, PlaneID p, CC& contacts )
{
   real dist, k;
   Vec3 pos;
   const Vec3& bpos( b->getPosition() );
   const Vec3& lengths( b->getLengths() );
   const Rot3& R( b->getRotation() );
   const real xrange( static_cast<real>( 0.5 )*lengths[0] );
   const real yrange( static_cast<real>( 0.5 )*lengths[1] );
   const real zrange( static_cast<real>( 0.5 )*lengths[2] );
   const Vec3 nrel( trans( b->getRotation() ) * p->getNormal() );
   const real minlength( -static_cast<real>( 0.99 )*pe::min( xrange, yrange, zrange ) );

   // Test of lower-left-front corner
   if( -xrange*nrel[0] - yrange*nrel[1] - zrange*nrel[2] < minlength )
   {
      pos = bpos + R*Vec3( -xrange, -yrange, -zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }

   // Test of the lower-right-front corner
   if( xrange*nrel[0] - yrange*nrel[1] - zrange*nrel[2] < minlength )
   {
      pos = bpos + R*Vec3( xrange, -yrange, -zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }

   // Test of the lower-right-back corner
   if( xrange*nrel[0] + yrange*nrel[1] - zrange*nrel[2] < minlength )
   {
      pos = bpos + R*Vec3( xrange, yrange, -zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }

   // Test of the lower-left-back corner
   if( -xrange*nrel[0] + yrange*nrel[1] - zrange*nrel[2] < minlength )
   {
      pos = bpos + R*Vec3( -xrange, yrange, -zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }

   // Test of the upper-left-front corner
   if( -xrange*nrel[0] - yrange*nrel[1] + zrange*nrel[2]  < minlength )
   {
      pos = bpos + R*Vec3( -xrange, -yrange, zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }

   // Test of the upper-right-front corner
   if( xrange*nrel[0] - yrange*nrel[1] + zrange*nrel[2] < minlength )
   {
      pos = bpos + R*Vec3( xrange, -yrange, zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }

   // Test of the upper-right-back corner
   if( xrange*nrel[0] + yrange*nrel[1] + zrange*nrel[2] < minlength )
   {
      pos = bpos + R*Vec3( xrange, yrange, zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }

   // Test of the upper-left-back corner
   if( -xrange*nrel[0] + yrange*nrel[1] + zrange*nrel[2] < minlength )
   {
      pos = bpos + R*Vec3( -xrange, yrange, zrange );
      k = trans(pos) * p->getNormal();
      dist = k - p->getDisplacement();
      if( dist < contactThreshold )
      {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Contact created between box " << b->getID()
                << " and plane " << p->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( b, p, pos, p->getNormal(), dist );
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Box and a Triangle Mesh.
 * \ingroup contact_generation
 *
 * \param b The colliding box.
 * \param m The colliding triangle mesh.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideBoxTMesh( BoxID b, TriangleMeshID m, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(gjkEPAcollideHybrid< BoxID, TriangleMeshID >(b, m, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (m) to object1 (b)
      contacts.addVertexFaceContact( b, m, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between box " << b->getID()
            << " and triangle mesh " << m->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Box and a Union.
 * \ingroup contact_generation
 *
 * \param b The colliding box.
 * \param u The colliding union.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between a box and a union is treated as collisions between the box and all
 * the subbodies of the union.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideBoxUnion( BoxID b, UnionID u, CC& contacts )
{
   for( Union::Iterator it=u->begin(); it!=u->end(); ++it ) {
      collide( b, *it, contacts );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between two colliding Capsule primitives.
 * \ingroup contact_generation
 *
 * \param c1 The first colliding capsule.
 * \param c2 The second colliding capsule.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * In case of two colliding capsules one or two contact points may be generated. In order to
 * estimate if the two capsules are colliding, the closest points on both centerlines are
 * calculated (see pe::getClosestLineSegmentPoints). If the distance of these two points is
 * smaller than the sum of their radii, the two capsules are colliding. In case they are
 * lying parallel to each other and touch each other along their cylindrical part, two contact
 * points are generated. In all other cases, a single contact point between the calculated
 * closest points is generated.
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideCapsuleCapsule( CapsuleID c1, CapsuleID c2, CC& contacts )
{
   // Force a defined order of collision detection across processes
   if( c2->getSystemID() < c1->getSystemID() )
      std::swap( c1, c2 );

   const Rot3& R1( c1->getRotation() );
   const Rot3& R2( c2->getRotation() );

   // Calculating the "up" direction for both capsules that points from
   // the center of mass to the center of the upper cap of each capsule
   const Vec3 c1_up_dir( R1[0], R1[3], R1[6] );
   const Vec3 c2_up_dir( R2[0], R2[3], R2[6] );

   // Calculating the displacement of the center of the upper cap-spheres in world space coordinates
   const Vec3 c1_up( real(0.5) * c1->getLength() * c1_up_dir );
   const Vec3 c2_up( real(0.5) * c2->getLength() * c2_up_dir );

   // calculate the closest points of the two center lines
   Vec3 cp1, cp2;
   getClosestLineSegmentPoints( c1->getPosition()+c1_up, c1->getPosition()-c1_up,
                                c2->getPosition()+c2_up, c2->getPosition()-c2_up, cp1, cp2);

   Vec3 normal( cp1 - cp2 );
   const real dist( normal.length() - c1->getRadius() - c2->getRadius() );

   if( dist < contactThreshold )
   {
      normal.normalize();

      // Calculating the relative x-position of the second capsule in the frame of the first capsule
      const real c2x( trans(c1_up_dir) * ( c2->getPosition() - c1->getPosition() ) );

      // Creating two contact points if the capsules are parallel to each other
      if( ( std::fabs( trans(c1_up_dir) * c2_up_dir ) - real(1) ) > -parallelThreshold &&
          c2x > -c1->getLength() && c2x < c1->getLength() )
      {
         const real k( c1->getRadius() + real(0.5) * dist );
         const real hl1( real(0.5) * c1->getLength() );
         const real hl2( real(0.5) * c2->getLength() );

         pe_LOG_DEBUG_SECTION( log ) {
            log << "      First contact created between capsule " << c1->getID()
                << " and capsule " << c2->getID() << " (dist=" << dist << ")";
         }

         // Creating the "upper" contact point in world coordinates
         if( hl1 < c2x + hl2 ) {
            contacts.addVertexFaceContact( c1, c2,
                                           c1->getPosition() + c1_up - k*normal,
                                           normal, dist );
         }
         else {
            contacts.addVertexFaceContact( c1, c2,
                                           c2->getPosition() + hl2 * c1_up_dir + k*normal,
                                           normal, dist );
         }

         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Second contact created between capsule " << c1->getID()
                << " and capsule " << c2->getID() << " (dist=" << dist << ")";
         }

         // Creating the "lower" contact point in world coordinates
         if( -hl1 > c2x - hl2 ) {
            contacts.addVertexFaceContact( c1, c2,
                                           c1->getPosition() - c1_up - k*normal,
                                           normal, dist );
         }
         else {
            contacts.addVertexFaceContact( c1, c2,
                                           c2->getPosition() - hl2 * c1_up_dir + k*normal,
                                           normal, dist );
         }
      }

      // Creating a single contact point
      else
      {
         const real k( c2->getRadius() + real(0.5) * dist );
         const Vec3 gPos( cp2 + normal * k );

         pe_LOG_DEBUG_SECTION( log ) {
            log << "      Single contact created between capsule " << c1->getID()
                << " and capsule " << c2->getID() << " (dist=" << dist << ")";
         }

         contacts.addVertexFaceContact( c1, c2, gPos, normal, dist );
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Capsule and a Cylinder.
 * \ingroup contact_generation
 *
 * \param ca The colliding capsule.
 * \param cy The colliding cylinder.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideCapsuleCylinder( CapsuleID ca, CylinderID cy, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(gjkEPAcollideHybrid< CapsuleID, CylinderID >(ca, cy, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (cy) to object1 (ca)
      contacts.addVertexFaceContact( ca, cy, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between capsule " << ca->getID()
            << " and cylinder " << cy->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Capsule and a Plane.
 * \ingroup contact_generation
 *
 * \param c The colliding capsule.
 * \param p The colliding plane.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between a capsule and a plane is handled as two collisions between the two
 * cap spheres and the plane: the contact points are calculated with a projection of the
 * global positions of the spheres' center of mass on the plane's normal. If the length of
 * the projection is smaller than the radius of the cap sphere, a contact point is generated,
 * which results in a maximum of two contact points.
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideCapsulePlane( CapsuleID c, PlaneID p, CC& contacts )
{
   const Rot3& R( c->getRotation() );

   // Computing the location of the sphere caps of the capsule
   const Vec3 c_up ( real(0.5) * c->getLength() * Vec3( R[0], R[3], R[6] ) );
   const Vec3 posUp( c->getPosition() + c_up );
   const Vec3 posDn( c->getPosition() - c_up );

   // Computing the distance between the sphere caps and the plane
   const real distUp( trans(posUp) * p->getNormal() - p->getDisplacement() - c->getRadius() );
   const real distDn( trans(posDn) * p->getNormal() - p->getDisplacement() - c->getRadius() );

   // Collision of the upper sphere with the plane
   if( distUp < contactThreshold )
   {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between capsule " << c->getID()
             << " and plane " << p->getID() << " (dist=" << distUp << ")";
      }

      contacts.addVertexFaceContact( c, p, posUp - c->getRadius()*p->getNormal(),
                                     p->getNormal(), distUp );
   }

   // Collision of the lower sphere with the plane
   if( distDn < contactThreshold )
   {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between capsule " << c->getID()
             << " and plane " << p->getID() << " (dist=" << distDn << ")";
      }

      contacts.addVertexFaceContact( c, p, posDn - c->getRadius()*p->getNormal(),
                                     p->getNormal(), distDn );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Capsule and a Triangle Mesh.
 * \ingroup contact_generation
 *
 * \param c The colliding capsule.
 * \param m The colliding triangle mesh.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideCapsuleTMesh( CapsuleID c, TriangleMeshID m, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;
   if(gjkEPAcollideHybrid< CapsuleID, TriangleMeshID >(c, m, normal, contactPoint, penetrationDepth)) {
   //if(gjkEPAcollide< CapsuleID >(c, m, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (m) to object1 (c)
      contacts.addVertexFaceContact( c, m, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between capsule " << c->getID()
            << " and triangle mesh " << m->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Capsule and a Union.
 * \ingroup contact_generation
 *
 * \param c The colliding capsule.
 * \param u The colliding union.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between a capsule and a union is treated as collisions between the
 * capsule and all the subbodies of the union.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideCapsuleUnion( CapsuleID c, UnionID u, CC& contacts )
{
   for( Union::Iterator it=u->begin(); it!=u->end(); ++it ) {
      collide( *it, c, contacts );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between two colliding Cylinder primitives.
 * \ingroup contact_generation
 *
 * \param c1 The first colliding cylinder.
 * \param c2 The second colliding cylinder.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideCylinderCylinder( CylinderID c1, CylinderID c2, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(gjkEPAcollideHybrid< CylinderID, CylinderID >(c1, c2, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (c2) to object1 (c1)
      contacts.addVertexFaceContact( c1, c2, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between cylinder " << c1->getID()
            << " and cylinder " << c2->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Cylinder and a Plane.
 * \ingroup contact_generation
 *
 * \param c The colliding cylinder.
 * \param p The colliding plane.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideCylinderPlane( CylinderID /*c*/, PlaneID /*p*/, CC& /*contacts*/ )
{
   // TODO: collide implementation
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Cylinder and a Triangle Mesh.
 * \ingroup contact_generation
 *
 * \param c The colliding cylinder.
 * \param m The colliding triangle mesh.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideCylinderTMesh( CylinderID c, TriangleMeshID m, CC& contacts )
{
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(gjkEPAcollideHybrid< CylinderID >(c, m, normal, contactPoint, penetrationDepth)) {
      //bodys possibly overlap
      //normal points form object2 (m) to object1 (c)
      contacts.addVertexFaceContact( c, m, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between cylinder " << c->getID()
            << " and triangle mesh " << m->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Cylinder and a Union.
 * \ingroup contact_generation
 *
 * \param c The colliding cylinder.
 * \param u The colliding union.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between a cylinder and a union is treated as collisions between the
 * cylinder and all the subbodies of the union.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideCylinderUnion( CylinderID c, UnionID u, CC& contacts )
{
   for( Union::Iterator it=u->begin(); it!=u->end(); ++it ) {
      collide( *it, c, contacts );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Plane and a Triangle Mesh.
 * \ingroup contact_generation
 *
 * \param p The colliding plane.
 * \param m The colliding triangle mesh.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collidePlaneTMesh( PlaneID p, TriangleMeshID m, CC& contacts )
{

#if FALSE
   static timing::WcTimer gesamt;
   static timing::WcTimer total;
   static timing::WcTimer dreieck;
   static timing::WcTimer viereck;
   static timing::WcTimer hinzufuegen;
   static timing::WcTimer sucher;
   static timing::WcTimer sucher2;
   static timing::WcTimer sucher3;
   static timing::WcTimer part1;
   static timing::WcTimer part2;
#endif
   static size_t counter = 0;
   static size_t maxQue = 0;
   static size_t maxRes = 0;




   size_t supportIdx = 0;
   Vec3 support = m->supportContactThreshold(-p->getNormal(),0, &supportIdx);

   real penetraionDepth = p->getDepth(support) - contactThreshold; //the support point is contactThreshold further than it actually is
/*
   std::cerr <<std::endl << "////////////////\npenetraionDepth=" <<  penetraionDepth <<std::endl;
   std::cerr << "support=" << support  <<std::endl;
*/
   if(penetraionDepth <= -contactThreshold) {
      //negative penetraionDepth means we are in normal direction off the plane
      //<= -contactThreshold means that meshe is close to the plane are considered to be in contact
      //do not overlap
      return;
   }

   const bool sympel = false;
   if(sympel) {
      //calac closest point on plane to point(support)
      //Buch 127
      Vec3 pointOnPlane = support - penetraionDepth * p->getNormal();

      Vec3 contactPoint = real(0.5)*(support + pointOnPlane);

      //normal points form object2 (plain) to object1 (mesh)
      //penetration penetraionDepth was positive
      contacts.addVertexFaceContact( m, p, contactPoint, p->getNormal(), -penetraionDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between plane " << p->getID()
             << " and triangle mesh " << m->getID() << " (dist=" << -penetraionDepth << ")";
      }
      return;
   }

   //gesamt.start();
   //sucher2.start();
   real area = 0.0; //total area of all parts of the penetration

   typedef std::pair< real, Vec3> Schwerpunkt; //first area of the face, second the COM of the face

#define VECTORS
//#define MAP_SET

#ifdef MAP_SET
   typedef std::map< size_t, Schwerpunkt> FaceList;

   FaceList faceList;
   std::set< size_t > facesToProcess;

   size_t currentFace = m->vertexEdge_[supportIdx] / 3;
   facesToProcess.insert(currentFace);

   //add the three neighbouring faces if they are not processed or already in the set
   size_t neigbourFace= m->edgeEdge_[(currentFace*3 +0)] / 3;
   facesToProcess.insert(neigbourFace);

   neigbourFace= m->edgeEdge_[(currentFace*3 +1)] / 3;
   facesToProcess.insert(neigbourFace);

   neigbourFace= m->edgeEdge_[(currentFace*3 +2)] / 3;
   facesToProcess.insert(neigbourFace);
#endif

#ifdef VECTORS
   std::vector< size_t > facesToProcess;
   facesToProcess.reserve(10); //maximum encountered was 8
   std::vector< char > workItems(m->faceIndices_.size(), 0);

   typedef std::vector< Schwerpunkt >FaceList;
   FaceList faceList;
   faceList.reserve(20); //maximum encountered was 14


   size_t currentFace = m->vertexEdge_[supportIdx] / 3;
   facesToProcess.push_back(currentFace);
   workItems[currentFace] = 1;

   //add the three neighbouring faces if they are not processed or already in the set
   size_t neigbourFace= m->edgeEdge_[(currentFace*3 +0)] / 3;
   facesToProcess.push_back(neigbourFace);
   workItems[neigbourFace] = 1;

   neigbourFace= m->edgeEdge_[(currentFace*3 +1)] / 3;
   facesToProcess.push_back(neigbourFace);
   workItems[neigbourFace] = 1;

   neigbourFace= m->edgeEdge_[(currentFace*3 +2)] / 3;
   facesToProcess.push_back(neigbourFace);
   workItems[neigbourFace] = 1;
#endif

//   sucher2.end();

   while(!facesToProcess.empty()) {
      if(facesToProcess.size() > maxQue) {
         maxQue = facesToProcess.size();
      }
//      sucher3.start();
#ifdef MAP_SET
      std::set< size_t >::const_iterator citer = facesToProcess.begin();
      currentFace = *citer;
      facesToProcess.erase(citer);
#endif
#ifdef VECTORS
      currentFace = facesToProcess.back();
      facesToProcess.pop_back();
#endif
//      sucher3.end();

      //sucher.start(); TODO
//      part1.start();
      const Vec3 A = m->pointFromBFtoWF(m->verticesOriginal_[m->faceIndices_[currentFace][0]]);
      const Vec3 B = m->pointFromBFtoWF(m->verticesOriginal_[m->faceIndices_[currentFace][1]]);
      const Vec3 C = m->pointFromBFtoWF(m->verticesOriginal_[m->faceIndices_[currentFace][2]]);

      //Positive if penetrating
      //+contactThreshold so close point are conidered to be below the plane surface
      const real depthA = p->getDepth(A) + contactThreshold; //Positive if penetrating
      const real depthB = p->getDepth(B) + contactThreshold;
      const real depthC = p->getDepth(C) + contactThreshold;

      unsigned char bitFieldCBA = 0; // Bits are ordert ?????CBA, 1 means point is below plane

      bitFieldCBA |= (depthA > 0.0)<<0;
      bitFieldCBA |= (depthB > 0.0)<<1;
      bitFieldCBA |= (depthC > 0.0)<<2;

/*
      std::cerr << "currentFace="<<currentFace << std::endl;
      std::cerr << "depthA="<<depthA  << " A="<< A<< std::endl;
      std::cerr << "depthB="<<depthB  << " B="<< B<< std::endl;
      std::cerr << "depthC="<<depthC  << " C="<< C<< std::endl;
      std::cerr << "bitFieldCBA="<< (unsigned int)bitFieldCBA << std::endl;
*/
      real currentArea = 0;
      Vec3 currentCOM;
      unsigned char whatToDoNext = 0;

      //variables for the on point penetrats case
      const Vec3* pentrater = NULL;
      const real* pentratorDepth = NULL;

      //variables for the two points penetrate case
      const Vec3* outsieder = NULL;
      const real* outsiederDepth = NULL;

      //Variables for both cases
      const Vec3* next = NULL;
      const real* nextDepth = NULL;
      const Vec3* last = NULL;
      const real* lastDepth = NULL;

//      part1.end();

//      part2.start();
      switch(bitFieldCBA) {
      case 7:{ //the howl face is penetrating
         whatToDoNext=0;
         break;
      }

      case 0: { //the howl face is NOT penetrating at all
//         part2.end();
         //sucher.end(); TODO
         continue; //do not add the triangles hanging on this on
      }

      case 1:{ //only point A is penetrating
         pentrater = &A;
         pentratorDepth = &depthA;
         next = &B;
         nextDepth = &depthB;
         last = &C;
         lastDepth = &depthC;

         whatToDoNext = 1;
         break;
      }

      case 2: {//only point B is penetrating
         pentrater = &B;
         pentratorDepth = &depthB;
         next = &C;
         nextDepth = &depthC;
         last = &A;
         lastDepth = &depthA;

         whatToDoNext = 1;
         break;
      }

      case 4: {//only point C is penetrating
         pentrater = &C;
         pentratorDepth = &depthC;
         next = &A;
         nextDepth = &depthA;
         last = &B;
         lastDepth = &depthB;

         whatToDoNext = 1;
         break;
      }

      case 3: { //A+B penetrate
         outsieder = &C;
         outsiederDepth = &depthC;
         next = &A;
         nextDepth = &depthA;
         last = &B;
         lastDepth = &depthB;

         whatToDoNext = 2;
         break;
      }

      case 6: { //B+C penetrate
         outsieder = &A;
         outsiederDepth = &depthA;
         next = &B;
         nextDepth = &depthB;
         last = &C;
         lastDepth = &depthC;

         whatToDoNext = 2;
         break;
      }

      case 5: { //C+A penetrate
         outsieder = &B;
         outsiederDepth = &depthB;
         next = &C;
         nextDepth = &depthC;
         last = &A;
         lastDepth = &depthA;

         whatToDoNext = 2;
         break;
      }

      default:
         //never happens
         break;
      }
//      part2.end();
      //sucher.end(); TODO

      static const real oneThird = real(1.0) / real(3.0);
      switch(whatToDoNext) {
      case 0: {//the howl face is penetrating
//         total.start();
         currentArea = ((B-A) % (C-A)).length(); //*0.5
         currentCOM = (A+B+C);// *1/3 at the end
//         total.end();
         break;
      }

      case 1: {//calculate area and all if only one point penetrates
         //Die durchdringunstiefen ist im gleichen verhaeltnis wie die Strecken
         //- bevore nextDepth because it is negativ

//         dreieck.start();
         real penetration1 = (*pentratorDepth) - (*nextDepth);
         Vec3 pointOnAB = -(*nextDepth)/penetration1 * (*pentrater) + (*pentratorDepth)/penetration1 * (*next);
         real penetration2 = (*pentratorDepth) - (*lastDepth);
         Vec3 pointOnAC = -(*lastDepth)/penetration2 * (*pentrater) + (*pentratorDepth)/penetration2 * (*last);

         //std::cerr << "pointOnAB="<<pointOnAB << std::endl;
         //std::cerr << "pointOnAC="<<pointOnAC << std::endl;

         currentArea = ((pointOnAB-(*pentrater)) % (pointOnAC-(*pentrater))).length(); //*0.5
         //http://de.wikipedia.org/wiki/Geometrischer_Schwerpunkt#Dreieck
         currentCOM = ((*pentrater)+pointOnAB+pointOnAC);// *1/3 at the end
//         dreieck.end();
         break;
      }

      case 2: { //calculate are and all if two points penetrate
//         viereck.start();
         real penetration1 = -(*outsiederDepth) + (*nextDepth);
         Vec3 extraPoint1 = (*nextDepth)/penetration1 * (*outsieder) - (*outsiederDepth)/penetration1 * (*next);
         real penetration2 = -(*outsiederDepth) + (*lastDepth);
         Vec3 extraPoint2 = (*lastDepth)/penetration2 * (*outsieder) - (*outsiederDepth)/penetration2 * (*last);

         const Vec3& a = extraPoint1;
         const Vec3& b = (*next);
         const Vec3& c = (*last);
         const Vec3& d = extraPoint2;

         currentArea = (((b - a) % (c - a)).length() + ((c - a) % (d - a)).length()); //*0.5

         //http://de.wikipedia.org/wiki/Viereck#Schwerpunkt
         Vec3 comABC = (a + b + c)*oneThird;
         Vec3 comACD = (a + c + d)*oneThird;
         Vec3 comABD = (a + b + d)*oneThird;
         Vec3 comBCD = (b + c + d)*oneThird;

         //closes points of two lines
         //Buch 146
         Vec3  d1  = comACD - comABC;
         Vec3T d1T = trans(d1);
         Vec3  d2  = comBCD - comABD;
         Vec3T d2T = trans(d2);
         Vec3  r   = comABC - comABD;

         //s = (bf - ce)/ddx ; a=d1*d1, b=d1*d2; f=d2*r; c=d1*r; e=d2*d2; dd=(ae-b*b)
         real dd = (d1T*d1 * d2T*d2 - d1T*d2 * d1T*d2);
         //        (   a   *    e   -    b   *    b  )
         real s = (d1T*d2 * d2T*r - d1T*r * d2T*d2) / dd;
         //       (   b   *    f  -    c  *    e  ) / dd;

         if(dd==0.0) { //quad is very thin so lines become parallel
            s = 0.5;
         }

         currentCOM = (comABC + s* d1)*real(3.0);// *1/3 at the end

//         viereck.end();
         break;
      }
      default:
         //never happens
         break;
      }

      //std::cerr << "currentArea=" << currentArea << std::endl;
      //std::cerr << "currentCOM=" << currentCOM << std::endl;
//      hinzufuegen.start();
#ifdef MAP_SET
      faceList[currentFace] = std::make_pair(currentArea, currentCOM);
      area += currentArea;

      //add the three neighbouring faces if they are not processed or already in the set
      neigbourFace= m->edgeEdge_[(currentFace*3 +0)] / 3;
      if(faceList.find(neigbourFace) == faceList.end()) {
         facesToProcess.insert(neigbourFace);
      }

      neigbourFace= m->edgeEdge_[(currentFace*3 +1)] / 3;
      if(faceList.find(neigbourFace) == faceList.end()) {
         facesToProcess.insert(neigbourFace);
      }

      neigbourFace= m->edgeEdge_[(currentFace*3 +2)] / 3;
      if(faceList.find(neigbourFace) == faceList.end()) {
         facesToProcess.insert(neigbourFace);
      }
#endif
#ifdef VECTORS
      faceList.push_back(std::make_pair(currentArea, currentCOM));
      area += currentArea;

      //add the three neighbouring faces if they are not processed or already in the set
      neigbourFace= m->edgeEdge_[(currentFace*3 +0)] / 3;
      if(workItems[neigbourFace] == 0) {
         facesToProcess.push_back(neigbourFace);
         workItems[neigbourFace] = 1;
      }

      neigbourFace= m->edgeEdge_[(currentFace*3 +1)] / 3;
      if(workItems[neigbourFace] == 0) {
         facesToProcess.push_back(neigbourFace);
         workItems[neigbourFace] = 1;
      }

      neigbourFace= m->edgeEdge_[(currentFace*3 +2)] / 3;
      if(workItems[neigbourFace] == 0) {
         facesToProcess.push_back(neigbourFace);
         workItems[neigbourFace] = 1;
      }
#endif
//      hinzufuegen.end();
      /*
      std::cerr << "facesToProcess:"<<std::endl;
      for (std::set<size_t>::const_iterator it=facesToProcess.begin(); it!=facesToProcess.end(); ++it) {
          std::cerr << ' ' << *it;
      }

      std::cerr << "\nfaceList:"<<std::endl;
      for (FaceList::const_iterator it=faceList.begin(); it!=faceList.end(); ++it) {
          std::cerr << " [" << it->first << "]=(" << it->second.first << ", " << it->second.second << ")" <<std::endl;
      }
      */
   }

   if(area == 0.0) {
      //no intersectoin volumn found, how ever this happened?!
      //std::cerr << "done no volumn found" << std::endl;
      return;
   }

   //calculte weighted COM
   Vec3 contactPoint(0.0, 0.0, 0.0);
#ifdef MAP_SET
   for (FaceList::const_iterator it=faceList.begin(); it!=faceList.end(); ++it) {
       contactPoint += it->second.first * it->second.second ;
   }
#endif
#ifdef VECTORS
   for (FaceList::const_iterator it=faceList.begin(); it!=faceList.end(); ++it) {
       contactPoint += it->first * it->second ;
   }
#endif
   contactPoint /= (3*area); //instead of /3 in every iteration

   if(faceList.size() > maxRes) {
      maxRes = faceList.size();
   }

   //std::cerr<< "done COM="<< contactPoint<< std::endl;


   //normal points form object2 (plain) to object1 (mesh)
   //penetration penetraionDepth was positive
   contacts.addVertexFaceContact( m, p, contactPoint, p->getNormal(), -penetraionDepth );
   pe_LOG_DEBUG_SECTION( log ) {
      log << "      Contact created between plane " << p->getID()
          << " and triangle mesh " << m->getID() << " (dist=" << -penetraionDepth << ")";
   }
//   gesamt.end();

#if FALSE
   if(counter % 1000 == 0) {
      std::cerr << counter << std::endl;
      real totalTime = total.total() + dreieck.total() + viereck.total() + hinzufuegen.total() + part1.total() + part2.total()
               + sucher2.total() + sucher3.total();

      std::cerr << gesamt.getCounter()<<  "\tgesamt: \t" << gesamt.average() << "\tmin:" <<gesamt.min() << "\tmax:" << gesamt.max()
               << "\ttotal:" << gesamt.total() << " totalTime:" << totalTime<< std::endl;
      std::cerr << total.getCounter()<<   "\ttotal:  \t" << total.average() << "\tmin:" <<total.min() << "\tmax:" << total.max()
               << "\t" << total.total()/totalTime*100.0 << "%"<< std::endl;
      std::cerr << dreieck.getCounter()<< "\tdreieck:\t" << dreieck.average() << "\tmin:" <<dreieck.min() << "\tmax:" << dreieck.max()
               << "\t" << dreieck.total()/totalTime*100.0 << "%"<< std::endl;
      std::cerr << viereck.getCounter()<< "\tviereck:\t" << viereck.average() << "\tmin:" <<viereck.min() << "\tmax:" << viereck.max()
               << "\t" << viereck.total()/totalTime*100.0 << "%"<<  std::endl;
      std::cerr << hinzufuegen.getCounter()<< "\thinzufu:\t" << hinzufuegen.average() << "\tmin:" <<hinzufuegen.min() << "\tmax:" << hinzufuegen.max()
               <<  "\t" << hinzufuegen.total()/totalTime*100.0 << "%" << std::endl;

      std::cerr << sucher.getCounter()<<  "\tsucher: \t" << sucher.average() << "\tmin:" <<sucher.min() << "\tmax:" << sucher.max()
               <<  "\t" << sucher.total()/totalTime*100.0 << "%" << std::endl;
      std::cerr << part1.getCounter()<<  "\tpart1:  \t" << part1.average() << "\tmin:" <<part1.min() << "\tmax:" << part1.max()
               <<  "\t" << part1.total()/totalTime*100.0 << "%" << std::endl;
      std::cerr << part2.getCounter()<<  "\tpart2:  \t" << part2.average() << "\tmin:" <<part2.min() << "\tmax:" << part2.max()
               <<  "\t" << part2.total()/totalTime*100.0 << "%" << std::endl;


      std::cerr << sucher2.getCounter()<< "\tsetup:  \t" << sucher2.average() << "\tmin:" <<sucher2.min() << "\tmax:" << sucher2.max()
               <<  "\t" << sucher2.total()/totalTime*100.0 << "%" << std::endl;
      std::cerr << sucher3.getCounter()<< "\tfirstEl:\t" << sucher3.average() << "\tmin:" <<sucher3.min() << "\tmax:" << sucher3.max()
               <<  "\t" << sucher3.total()/totalTime*100.0 << "%" << std::endl;

      std::cerr << "maxQue:" << maxQue << " maxRes:" << maxRes << std::endl;
      std::cerr << "Contact: p-ID="<<p->getID() << " mesh-ID="<<m->getID() << " dist=" << - penetraionDepth << " point=" << contactPoint << std::endl;

   }
#endif
   counter++;

   return;





}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Plane and a Union.
 * \ingroup contact_generation
 *
 * \param p The colliding plane.
 * \param u The colliding union.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between a plane and a union is treated as collisions between the plane
 * and all the subbodies of the union.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collidePlaneUnion( PlaneID p, UnionID u, CC& contacts )
{
   for( Union::Iterator it=u->begin(); it!=u->end(); ++it ) {
      collide( *it, p, contacts );
   }
}
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Contact generation between two colliding Triangle Mesh primitives.
 * \ingroup contact_generation
 *
 * \param m1 The first colliding triangle mesh.
 * \param m2 The second colliding triangle mesh.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * TODO
 */
template< typename CC >  // Type of the contact container
void MaxContacts::collideTMeshTMesh( TriangleMeshID mA, TriangleMeshID mB, CC& contacts )
{
   // Force a defined order of collision detection across processes
   if( mB->getSystemID() < mA->getSystemID() )
      std::swap( mA, mB );


   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(gjkEPAcollideHybrid< TriangleMeshID, TriangleMeshID >(mA, mB, normal, contactPoint, penetrationDepth)) {
      //normal points form object2 (mB) to object1 (mA)
      //as the wittnes points are placed within the other body the normal must be inverted
      //negative penetraionDepth means penetration

      contacts.addVertexFaceContact( mA, mB, contactPoint, normal, penetrationDepth );
      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact created between triangle mesh " << mA->getID()
            << " and triangle mesh " << mB->getID() << " (dist=" << penetrationDepth << ")";
      }
   }
   return;

   //TODO remove this or not?
/*
   GJK gjk;
   if(!gjk.doGJKcontactThreshold(mA, mB)) {
      //not penetration
      return;
   }

   EPA epa;
   Vec3 normal;
   Vec3 contactPoint;
   real penetrationDepth;

   if(!epa.doEPAcontactThreshold(mA, mB, gjk, normal, contactPoint, penetrationDepth)) {
      //touching contact
      //or EPA problem
      return;
   }
   //mA and mB overlap

   //normal points form object2 (mB) to object1 (mA)
   //as the wittnes points are placed within the other body the normal must be inverted
   //negative penetraionDepth means penetration

   contacts.addVertexFaceContact( mA, mB, contactPoint, normal, penetrationDepth );
   pe_LOG_DEBUG_SECTION( log ) {
      log << "      Contact created between triangle mesh " << mA->getID()
         << " and triangle mesh " << mB->getID() << " (dist=" << penetrationDepth << ")";
   }

   return;
*/
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between a Triangle Mesh and a Union.
 * \ingroup contact_generation
 *
 * \param m The colliding triangle mesh.
 * \param p The colliding union.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between a triangle mesh and a union is treated as collisions between the
 * triangle mesh and all the subbodies of the union.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideTMeshUnion( TriangleMeshID m, UnionID u, CC& contacts )
{
   for( Union::Iterator it=u->begin(); it!=u->end(); ++it ) {
      collide( *it, m, contacts );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between two colliding Union primitives.
 * \ingroup contact_generation
 *
 * \param u1 The first colliding union.
 * \param u2 The second colliding union.
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * The collision between two unions is treated as the collision between each of the subbodies of
 * both unions.
 */
template< typename CC >  // Type of the contact container
inline void MaxContacts::collideUnionUnion( UnionID u1, UnionID u2, CC& contacts )
{
   // Force a defined order of collision detection across processes
   if( u2->getSystemID() < u1->getSystemID() )
      std::swap( u1, u2 );

   for( Union::Iterator it1=u1->begin(); it1!=u1->end(); ++it1 ) {
      for( Union::Iterator it2=u2->begin(); it2!=u2->end(); ++it2 ) {
         collide( *it1, *it2, contacts );
      }
   }
}
//*************************************************************************************************


} // namespace fine

} // namespace detection

} // namespace pe

#endif
