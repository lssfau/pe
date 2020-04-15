//=================================================================================================
/*!
 *  \file pe/core/GeomTools.h
 *  \brief Header file for geometrical utility functions
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

#ifndef _PE_CORE_GEOMTOOLS_H_
#define _PE_CORE_GEOMTOOLS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/RotationMatrix.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>


namespace pe {

//=================================================================================================
//
//  GEOMETRY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup geometry Geometry functions
 * \ingroup core
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\name Geometry functions */
//@{
void getClosestLineBoxPoints( const Vec3& p1, const Vec3& p2, const Vec3& c, const Rot3& R,
                              const Vec3& side, Vec3& lret, Vec3& bret);

void getClosestLineSegmentPoints( const Vec3& a1, const Vec3& a2, const Vec3& b1, const Vec3& b2,
                                  Vec3& cp1, Vec3& cp2 );

void intersectLines( const Vec3& o1, const Vec3& d1, const Vec3& o2, const Vec3& d2,
                     real& s, real& t );

inline bool originInTetrahedron( const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D );
inline bool pointInTetrahedron ( const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D,
                                 const Vec3& point );

inline bool pointInFrontOfPlane( const Vec3& normal, const Vec3& pointOnPlane, const Vec3& point );

Vec3 closestPointToTriangle( const Vec3& point, const Vec3& A, const Vec3& B, const Vec3& C );
Vec3 closestPointToTriangle( const Vec3& point, const Vec3& A, const Vec3& B, const Vec3& C,
                             real& a, real& b, real& c );
//@}
//*************************************************************************************************



//=================================================================================================
//
//  GEOMETRY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Estimates whether or not the origin lies within the given tetrahedron.
 * \ingroup geometry
 *
 * \param A Vertex that sees the triangle BCD in counterclockwise order.
 * \param B Vertex that sees the triangle ADC in counterclockwise order.
 * \param C Vertex that sees the triangle ABD in counterclockwise order.
 * \param D Vertex that sees the triangle ACB in counterclockwise order.
 * \return \a true if the origin lies within the tetrahedron, otherwise \a false.
 *
 * \note Points on the surface of the tetrahedron are considered inside.
 * 
 * \todo Review documentation
 */
inline bool originInTetrahedron( const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D ) {
   Vec3T aoT = trans(A);

   //using fpuAccuracy instead of real(0.0) to avoid numeric problems
   if((aoT * (B % C)) < -Limits<real>::fpuAccuracy()) {
      //if volume of ABC and Origin <0.0 than the origin is on the wrong side of ABC
      //http://mathworld.wolfram.com/Tetrahedron.html volume formula
      return false;
   }
   if((aoT * (C % D)) < -Limits<real>::fpuAccuracy()) {
      return false;
   }
   if((aoT * (D % B)) < -Limits<real>::fpuAccuracy()) {
      return false;
   }
   if((trans(B) * (D % C)) < -Limits<real>::fpuAccuracy()) {
      return false;
   }
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates whether or not a given point lies within the given tetrahedron.
 * \ingroup geometry
 *
 * \param A Vertex that sees the triangle BCD in counterclockwise order.
 * \param B Vertex that sees the triangle ADC in counterclockwise order.
 * \param C Vertex that sees the triangle ABD in counterclockwise order.
 * \param D Vertex that sees the triangle ACB in counterclockwise order.
 * \param point The point whose position is check for being inside the tetrahedron.
 * \return \a true if the origin lies within the tetrahedron, otherwise \a false.
 *
 * \note Points on the surface of the tetrahedron are considered inside.
 * \todo Review documentation
 */
inline bool pointInTetrahedron( const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D, const Vec3& point ) {
   return originInTetrahedron( A-point, B-point, C-point, D-point );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates whether a given point is in front of a plane.
 * \ingroup geometry
 *
 * \param normal The normal of the plane, does not have to be normalised.
 * \param pointOnPlane Any point on the Plane.
 * \param point The point whose position is check for being in front of the plane.
 * \return \a true if the origin lies in front of the plane, otherwise \a false.
 *
 * \note Points on the surface of the plane are considered not in front of the plane.
 * \todo Review documentation
 */
inline bool pointInFrontOfPlane( const Vec3& normal, const Vec3& pointOnPlane, const Vec3& point ) {
   return (trans(normal) * (point - pointOnPlane)) > 0;
}
//*************************************************************************************************


} // namespace pe

#endif
