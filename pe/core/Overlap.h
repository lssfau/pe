//=================================================================================================
/*!
 *  \file pe/core/Overlap.h
 *  \brief Overlap tests between rigid bodies
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

#ifndef _PE_CORE_OVERLAP_H_
#define _PE_CORE_OVERLAP_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Types.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>


namespace pe {

//=================================================================================================
//
//  OVERLAP TEST FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup overlap_tests Overlap test functions
 * \ingroup core
 *
 * These functions can be used to detect overlaps/interpenetrations between two rigid bodies.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\name Overlap test functions */
//@{
inline bool overlap                ( ConstBodyID b1    , ConstBodyID b2     );
inline bool overlapSphereSphere    ( ConstSphereID s1  , ConstSphereID s2   );
       bool overlapSphereBox       ( ConstSphereID s   , ConstBoxID b       );
       bool overlapSphereCapsule   ( ConstSphereID s   , ConstCapsuleID c   );
       bool overlapSphereCylinder  ( ConstSphereID s   , ConstCylinderID c  );
inline bool overlapSpherePlane     ( ConstSphereID s   , ConstPlaneID p     );
inline bool overlapSphereUnion     ( ConstSphereID s   , ConstUnionID u     );
       bool overlapBoxBox          ( ConstBoxID b1     , ConstBoxID b2      );
       bool overlapBoxCapsule      ( ConstBoxID b      , ConstCapsuleID c   );
       bool overlapBoxCylinder     ( ConstBoxID b      , ConstCylinderID c  );
       bool overlapBoxPlane        ( ConstBoxID b      , ConstPlaneID p     );
inline bool overlapBoxUnion        ( ConstBoxID b      , ConstUnionID u     );
       bool overlapCapsuleCapsule  ( ConstCapsuleID c1 , ConstCapsuleID c2  );
       bool overlapCapsuleCylinder ( ConstCapsuleID ca , ConstCylinderID cy );
       bool overlapCapsulePlane    ( ConstCapsuleID c  , ConstPlaneID p     );
inline bool overlapCapsuleUnion    ( ConstCapsuleID c  , ConstUnionID u     );
       bool overlapCylinderCylinder( ConstCylinderID c1, ConstCylinderID c2 );
       bool overlapCylinderPlane   ( ConstCylinderID c , ConstPlaneID p     );
inline bool overlapCylinderUnion   ( ConstCylinderID c , ConstUnionID u     );
inline bool overlapPlanePlane      ( ConstPlaneID p1   , ConstPlaneID p2    );
inline bool overlapPlaneUnion      ( ConstPlaneID p    , ConstUnionID u     );
inline bool overlapUnionUnion      ( ConstUnionID u1   , ConstUnionID u2    );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
bool overlap_backend( ConstBodyID b1, ConstBodyID b2 );
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two rigid bodies.
 * \ingroup overlap_tests
 *
 * \param b1 The first rigid body.
 * \param b2 The second rigid body.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 * \exception std::runtime_error Unknown body type.
 *
 * Overlap test between the bodies \a b1 and \a b2. If the maximum distance between the surfaces
 * of the two bodies is smaller than the threshold value pe::contactThreshold, then the two bodies
 * are overlapping and the overlap test returns \a true.
 */
inline bool overlap( ConstBodyID b1, ConstBodyID b2 )
{
   // Testing for overlapping bounding boxes
   if( !( b1->getAABB().overlaps( b2->getAABB() ) ) )  // Testing for overlapping bounding boxes
      return false;
   else return overlap_backend( b1, b2 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two Sphere primitives.
 * \ingroup overlap_tests
 *
 * \param s1 The first sphere.
 * \param s2 The second sphere.
 * \return \a true if the two spheres are overlapping, \a false if no overlap is detected.
 *
 * To test whether two spheres are overlapping, the distance between the centers of mass is
 * calculated and compared to the radii of the spheres.
 */
inline bool overlapSphereSphere( ConstSphereID s1, ConstSphereID s2 )
{
   const Vec3 normal( s1->getPosition() - s2->getPosition() );
   const real dist( normal.length() - s1->getRadius() - s2->getRadius() );

   if( dist < contactThreshold ) return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Sphere and a Plane.
 * \ingroup overlap_tests
 *
 * \param s The sphere.
 * \param p The plane.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * To test whether a sphere and a plane overlap, the projection of the sphere's global position
 * on the plane's normal is compared to the radius of the sphere and the displacement of the
 * plane.
 */
inline bool overlapSpherePlane( ConstSphereID s, ConstPlaneID p )
{
   const real dist = ( trans( p->getNormal() ) * s->getPosition() ) - s->getRadius() - p->getDisplacement();

   if( dist < contactThreshold ) return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Sphere and a Union.
 * \ingroup overlap_tests
 *
 * \param s The sphere.
 * \param u The union.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between a sphere and a union is treated as overlap tests between the sphere
 * and all subbodies of the union.
 */
inline bool overlapSphereUnion( ConstSphereID s, ConstUnionID u )
{
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      if( overlap( s, *it ) ) return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Box and a Union.
 * \ingroup overlap_tests
 *
 * \param b The box.
 * \param u The union.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between a box and a union is treated as overlap tests between the box and
 * all subbodies of the union.
 */
inline bool overlapBoxUnion( ConstBoxID b, ConstUnionID u )
{
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      if( overlap( *it, b ) ) return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Capsule and a Union.
 * \ingroup overlap_tests
 *
 * \param c The capsule.
 * \param u The union.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between a capsule and a union is treated as overlap tests between
 * the capsule and all subbodies of the union.
 */
inline bool overlapCapsuleUnion( ConstCapsuleID c, ConstUnionID u )
{
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      if( overlap( *it, c ) ) return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Cylinder and a Union.
 * \ingroup overlap_tests
 *
 * \param c The cylinder.
 * \param u The union.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between a cylinder and a union is treated as overlap tests between
 * the cylinder and all subbodies of the union.
 */
inline bool overlapCylinderUnion( ConstCylinderID c, ConstUnionID u )
{
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      if( overlap( *it, c ) ) return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two Plane primitives.
 * \ingroup overlap_tests
 *
 * \param p1 The first plane.
 * \param p2 The second plane.
 * \return \a true if the two planes are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between two planes is performed by a comparison of the planes' normals.
 */
inline bool overlapPlanePlane( ConstPlaneID p1, ConstPlaneID p2 )
{
   if( std::fabs( trans( p1->getNormal() ) * p2->getNormal() ) >= real(1.0-1E-12) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Plane and a Union.
 * \ingroup overlap_tests
 *
 * \param p The plane.
 * \param u The union.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between a plane and a union is treated as overlap tests between the
 * plane and all subbodies of the union.
 */
inline bool overlapPlaneUnion( ConstPlaneID p, ConstUnionID u )
{
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      if( overlap( *it, p ) ) return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two Union primitives.
 * \ingroup overlap_tests
 *
 * \param u1 The first union.
 * \param u2 The second union.
 * \return \a true if the two unions are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between two unions is treated as overlap tests between all subbodies of
 * both unions.
 */
inline bool overlapUnionUnion( ConstUnionID u1, ConstUnionID u2 )
{
   for( Union::ConstIterator it1=u1->begin(); it1!=u1->end(); ++it1 ) {
      for( Union::ConstIterator it2=u2->begin(); it2!=u2->end(); ++it2 ) {
         if( overlap( *it1, *it2 ) ) return true;
      }
   }

   return false;
}
//*************************************************************************************************

} // namespace pe

#endif
