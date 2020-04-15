//=================================================================================================
/*!
 *  \file pe/core/Distance.h
 *  \brief Distance calculation between rigid bodies
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

#ifndef _PE_CORE_DISTANCE_H_
#define _PE_CORE_DISTANCE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/Types.h>
#include <pe/math/Functions.h>
#include <pe/math/Infinity.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>


namespace pe {

//=================================================================================================
//
//  DISTANCE CALCULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup distance_calculation Distance calculation functions
 * \ingroup core
 *
 * These functions can be used to calculate the distance between two rigid bodies.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\name Distance calculation functions */
//@{
       real distance                ( ConstBodyID b1    , ConstBodyID b2     );
inline real distanceSphereSphere    ( ConstSphereID s1  , ConstSphereID s2   );
       real distanceSphereBox       ( ConstSphereID s   , ConstBoxID b       );
       real distanceSphereCapsule   ( ConstSphereID s   , ConstCapsuleID c   );
       real distanceSphereCylinder  ( ConstSphereID s   , ConstCylinderID c  );
inline real distanceSpherePlane     ( ConstSphereID s   , ConstPlaneID p     );
inline real distanceSphereUnion     ( ConstSphereID s   , ConstUnionID u     );
       real distanceBoxBox          ( ConstBoxID b1     , ConstBoxID b2      );
       real distanceBoxCapsule      ( ConstBoxID b      , ConstCapsuleID c   );
       real distanceBoxCylinder     ( ConstBoxID b      , ConstCylinderID c  );
       real distanceBoxPlane        ( ConstBoxID b      , ConstPlaneID p     );
inline real distanceBoxUnion        ( ConstBoxID b      , ConstUnionID u     );
       real distanceCapsuleCapsule  ( ConstCapsuleID c1 , ConstCapsuleID c2  );
       real distanceCapsuleCylinder ( ConstCapsuleID ca , ConstCylinderID cy );
       real distanceCapsulePlane    ( ConstCapsuleID c  , ConstPlaneID p     );
inline real distanceCapsuleUnion    ( ConstCapsuleID c  , ConstUnionID u     );
       real distanceCylinderCylinder( ConstCylinderID c1, ConstCylinderID c2 );
       real distanceCylinderPlane   ( ConstCylinderID c , ConstPlaneID p     );
inline real distanceCylinderUnion   ( ConstCylinderID c , ConstUnionID u     );
inline real distancePlanePlane      ( ConstPlaneID p1   , ConstPlaneID p2    );
inline real distancePlaneUnion      ( ConstPlaneID p    , ConstUnionID u     );
inline real distanceUnionUnion      ( ConstUnionID u1   , ConstUnionID u2    );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between two Sphere primitives.
 * \ingroup distance_calculation
 *
 * \param s1 The first sphere.
 * \param s2 The second sphere.
 * \return The minimum distance between the two spheres.
 *
 * This function returns the distance between the two spheres \a s1 and \a s2. Note that a
 * negative distance indicates that the two spheres are overlapping.
 */
inline real distanceSphereSphere( ConstSphereID s1, ConstSphereID s2 )
{
   const Vec3 normal( s1->getPosition() - s2->getPosition() );
   return normal.length() - s1->getRadius() - s2->getRadius();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Sphere and a Plane.
 * \ingroup distance_calculation
 *
 * \param s The sphere.
 * \param p The plane.
 * \return The minimum distance between the sphere and the plane.
 *
 * This function returns the distance between the sphere \a s and the plane \a p. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
inline real distanceSpherePlane( ConstSphereID s, ConstPlaneID p )
{
   return ( trans( s->getPosition() ) * p->getNormal() ) - s->getRadius() - p->getDisplacement();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Sphere and a Union.
 * \ingroup distance_calculation
 *
 * \param s The sphere.
 * \param u The union.
 * \return The minimum distance between the sphere and the union.
 *
 * This function returns the distance between the sphere \a s and the union \a u. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
inline real distanceSphereUnion( ConstSphereID s, ConstUnionID u )
{
   real d( inf );
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      d = min( d, distance( s, *it ) );
   }
   return d;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Box and a Union.
 * \ingroup distance_calculation
 *
 * \param b The box.
 * \param u The union.
 * \return The minimum distance between the box and the union.
 *
 * This function returns the distance between the box \a b and the union \a u. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
inline real distanceBoxUnion( ConstBoxID b, ConstUnionID u )
{
   real d( inf );
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      d = min( d, distance( b, *it ) );
   }
   return d;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Capsule and a Union.
 * \ingroup distance_calculation
 *
 * \param c The capsule.
 * \param u The union.
 * \return The minimum distance between the capsule and the union.
 *
 * This function returns the distance between the capsule \a c and the union \a u. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
inline real distanceCapsuleUnion( ConstCapsuleID c, ConstUnionID u )
{
   real d( inf );
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      d = min( d, distance( c, *it ) );
   }
   return d;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Cylinder and a Union.
 * \ingroup distance_calculation
 *
 * \param c The cylinder.
 * \param u The union.
 * \return The minimum distance between the cylinder and the union.
 *
 * This function returns the distance between the cylinder \a c and the union \a u. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
inline real distanceCylinderUnion( ConstCylinderID c, ConstUnionID u )
{
   real d( inf );
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      d = min( d, distance( c, *it ) );
   }
   return d;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between two Plane primitives.
 * \ingroup distance_calculation
 *
 * \param p1 The first plane.
 * \param p2 The second plane.
 * \return The minimum distance between the two planes.
 *
 * This function returns the distance between the two planes \a p1 and \a p2. Note that in case
 * the two (infinite) planes overlap the returned distance is - pe::inf.
 */
inline real distancePlanePlane( ConstPlaneID p1, ConstPlaneID p2 )
{
   if( ( trans( p1->getNormal() ) * p2->getNormal() ) <= real(1E-12-1.0) )
      return std::fabs( p1->getDisplacement() - p2->getDisplacement() );
   else return -inf;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Plane and a Union.
 * \ingroup distance_calculation
 *
 * \param p The plane.
 * \param u The union.
 * \return The minimum distance between the plane and the union.
 *
 * This function returns the distance between the plane \a c and the union \a u. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
inline real distancePlaneUnion( ConstPlaneID p, ConstUnionID u )
{
   real d( inf );
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      d = min( d, distance( p, *it ) );
   }
   return d;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between two Union primitives.
 * \ingroup distance_calculation
 *
 * \param u1 The first union.
 * \param u2 The second union.
 * \return The minimum distance between the two unions.
 *
 * This function returns the distance between the two unions \a u1 and \a u2. Note that
 * a negative distance indicates that the two unions are overlapping.
 */
inline real distanceUnionUnion( ConstUnionID u1, ConstUnionID u2 )
{
   real d( inf );
   for( Union::ConstIterator it1=u1->begin(); it1!=u1->end(); ++it1 ) {
      for( Union::ConstIterator it2=u2->begin(); it2!=u2->end(); ++it2 ) {
         d = min( d, distance( *it1, *it2 ) );
      }
   }
   return d;
}
//*************************************************************************************************

} // namespace pe

#endif
