//=================================================================================================
/*!
 *  \file src/core/GeomTools.cpp
 *  \brief Source file for geometrical utility functions
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <limits>
#include <pe/core/GeomTools.h>
#include <pe/core/Thresholds.h>
#include <pe/math/Accuracy.h>
#include <pe/math/Epsilon.h>
#include <pe/math/shims/Square.h>


namespace pe {

//=================================================================================================
//
//  GEOMETRY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Find the closest points on a box and a line segment.
 * \ingroup geometry
 *
 * \param p1 The first end point of the line segment.
 * \param p2 The second end point of the line segment.
 * \param c The center position of the box.
 * \param R The rotation of the box.
 * \param side The box's sides lengths.
 * \param [out] lret The closest point on the line.
 * \param [out] bret The closest point on the box.
 * \return void
 *
 * This function calculates the closest points between a box and a line segment. The variable
 * \a lret will be set to the closest point on the line, the variable \a bret to the closest
 * point on the box. In case \a p1 lies outside the box and the line intersects the box, the
 * intersection point is returned (in both variables). In case \a p1 lies inside the box, both
 * \a lret and \a bret will refer to \a p1.
 *
 * \image html lineBox.png
 * \image latex lineBox.eps "Calculation of the closest points between a line and a box" width=455pt
 */
void getClosestLineBoxPoints( const Vec3& p1, const Vec3& p2, const Vec3& c, const Rot3& R,
                              const Vec3& side, Vec3& lret, Vec3& bret )
{
   //----- Note: All computations will be performed in the box-relative coordinate-system -----


   // Calculating the global and relative direction of the line p1--p2
   const Vec3 l( p2 - p1 );
   Vec3 tmp( trans(R) * l );

   // Saving the sign of the direction p1--p2
   const real sign[] = { pe::sign(tmp[0]), pe::sign(tmp[1]), pe::sign(tmp[2]) };

   // Calculating the absolute values of the direction direction
   const real v [] = { sign[0]*tmp[0], sign[1]*tmp[1], sign[2]*tmp[2] };
   const real v2[] = { sq( v[0] )   , sq( v[1] )   , sq( v[2] )    };

   // Calculating the starting point of the line p1--p2 in box-relative coordinates
   tmp = p1 - c;
   const real s[] = { sign[0]*( R[0]*tmp[0] + R[3]*tmp[1] + R[6]*tmp[2] ),
                      sign[1]*( R[1]*tmp[0] + R[4]*tmp[1] + R[7]*tmp[2] ),
                      sign[2]*( R[2]*tmp[0] + R[5]*tmp[1] + R[8]*tmp[2] ) };

   // Calculating the half lengths of the box
   const real h[] = { real(0.5)*side[0], real(0.5)*side[1], real(0.5)*side[2] };


   // Estimating the region of the starting point depending on which side of the
   // box planes the coordinates are (-1,0,1). 'tanchor' stores the next t value
   // causing a transition from one to another region, or the last one if there
   // are no more. Additionally, d|d|^2/dt is computed for t=0. If it is >= 0
   // then p1 is the closest point since the line points away from the box.
   int  region [] = { 0, 0, 0 };
   real tanchor[] = { 2, 2, 2 };  // Invalid t values; t cannot be greater than 1
   real dd2dt( 0 );

   if( v[0] > epsilon )
   {
      if( s[0] < -h[0] ) {
         region[0]  = -1;
         tanchor[0] = ( -h[0]-s[0] ) / v[0];
         dd2dt -= v2[0] * tanchor[0];
      }
      else if( s[0] > h[0] ) {
         region[0] = 1;
         tanchor[0] = ( h[0]-s[0] ) / v[0];
         dd2dt -= v2[0] * tanchor[0];
      }
      else {
         tanchor[0] = ( h[0]-s[0] ) / v[0];
      }
   }

   if( v[1] > epsilon )
   {
      if( s[1] < -h[1] ) {
         region[1]  = -1;
         tanchor[1] = ( -h[1]-s[1] ) / v[1];
         dd2dt -= v2[1] * tanchor[1];
      }
      else if( s[1] > h[1] ) {
         region[1] = 1;
         tanchor[1] = ( h[1]-s[1] ) / v[1];
         dd2dt -= v2[1] * tanchor[1];
      }
      else {
         tanchor[1] = ( h[1]-s[1] ) / v[1];
      }
   }

   if( v[2] > epsilon )
   {
      if( s[2] < -h[2] ) {
         region[2]  = -1;
         tanchor[2] = ( -h[2]-s[2] ) / v[2];
         dd2dt -= v2[2] * tanchor[2];
      }
      else if( s[2] > h[2] ) {
         region[2] = 1;
         tanchor[2] = ( h[2]-s[2] ) / v[2];
         dd2dt -= v2[2] * tanchor[2];
      }
      else {
         tanchor[2] = ( h[2]-s[2] ) / v[2];
      }
   }


   // Calculating the value t for the closest point on the line
   real t( 0 );

   if( dd2dt < -accuracy )
   {
      do {
         // Finding the t value for the next clip plane/line intersection
         real next_t( 1 );

         if( ( tanchor[0] > t ) && ( tanchor[0] < real(1) ) && ( tanchor[0] < next_t ) ) {
            next_t = tanchor[0];
         }
         if( ( tanchor[1] > t ) && ( tanchor[1] < real(1) ) && ( tanchor[1] < next_t ) ) {
            next_t = tanchor[1];
         }
         if( ( tanchor[2] > t ) && ( tanchor[2] < real(1) ) && ( tanchor[2] < next_t ) ) {
            next_t = tanchor[2];
         }

         // Computing d|d|^2/dt for the next t
         real next_dd2dt( 0 );

         if( region[0] != 0 ) {
            next_dd2dt += v2[0] * ( next_t - tanchor[0] );
         }
         if( region[1] != 0 ) {
            next_dd2dt += v2[1] * ( next_t - tanchor[1] );
         }
         if( region[2] != 0 ) {
            next_dd2dt += v2[2] * ( next_t - tanchor[2] );
         }

         // if the sign of d|d|^2/dt has changed, solution = the crossover point
         if( next_dd2dt >= 0 ) {
            t -= dd2dt / ( ( next_dd2dt - dd2dt ) / ( next_t - t ) );
            break;
         }

         // Advancing to the next anchor point / region
         if( tanchor[0] == next_t ) {
            tanchor[0] = ( h[0] - s[0] ) / v[0];
            ++region[0];
         }
         if( tanchor[1] == next_t ) {
            tanchor[1] = ( h[1] - s[1] ) / v[1];
            ++region[1];
         }
         if( tanchor[2] == next_t ) {
            tanchor[2] = ( h[2] - s[2] ) / v[2];
            ++region[2];
         }

         t = next_t;
         dd2dt = next_dd2dt;
      }
      while( t < real(1) );
   }

   pe_INTERNAL_ASSERT( t >= real(0) && t <= real(1), "Invalid line point" );

   // Computing the closest point on the line
   lret = p1 + t * l;

   // Computing the closest point on the box
   tmp[0] = sign[0] * ( s[0] + t * v[0] );
   if     ( tmp[0] < -h[0] ) tmp[0] = -h[0];
   else if( tmp[0] >  h[0] ) tmp[0] =  h[0];

   tmp[1] = sign[1] * ( s[1] + t * v[1] );
   if     ( tmp[1] < -h[1] ) tmp[1] = -h[1];
   else if( tmp[1] >  h[1] ) tmp[1] =  h[1];

   tmp[2] = sign[2] * ( s[2] + t * v[2] );
   if     ( tmp[2] < -h[2] ) tmp[2] = -h[2];
   else if( tmp[2] >  h[2] ) tmp[2] =  h[2];

   bret = ( R * tmp ) + c;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Find the closest points on two line segments.
 * \ingroup geometry
 *
 * \param a1 The first end point of the first line segment.
 * \param a2 The second end point of the first line segment.
 * \param b1 The first end point of the second line segment.
 * \param b2 The second end point of the second line segment.
 * \param [out] cp1 The closest point on line one.
 * \param [out] cp2 The closest point on line two.
 * \return void
 *
 * Given the two line segments A and B, this function returns the points that are closest to
 * each other. In the case of parallel lines, where multiple solutions are possible, a solution
 * involving the endpoint of at least one line will be returned. This will also work correctly
 * for zero length lines, e.g. if \a a1 = \a a2 and/or \a b1 = \a b2.
 */
void getClosestLineSegmentPoints( const Vec3& a1, const Vec3& a2, const Vec3& b1, const Vec3& b2,
                                  Vec3& cp1, Vec3& cp2 )
{
   //----- Checking the vertex-vertex features -----

   const Vec3 a1a2( a2 - a1 );
   const Vec3 b1b2( b2 - b1 );
   const Vec3 a1b1( b1 - a1 );
   const real da1 ( trans(a1a2) * a1b1 );
   const real db1 ( trans(b1b2) * a1b1 );
   if( da1 <= real(0) && db1 >= real(0) ) {
      cp1 = a1;
      cp2 = b1;
      return;
   }

   const Vec3 a1b2( b2 - a1 );
   const real da2 ( trans(a1a2) * a1b2 );
   const real db2 ( trans(b1b2) * a1b2 );
   if( da2 <= real(0) && db2 <= real(0) ) {
      cp1 = a1;
      cp2 = b2;
      return;
   }

   const Vec3 a2b1( b1 - a2 );
   const real da3 ( trans(a1a2) * a2b1 );
   const real db3 ( trans(b1b2) * a2b1 );
   if( da3 >= real(0) && db3 >= real(0) ) {
      cp1 = a2;
      cp2 = b1;
      return;
   }

   const Vec3 a2b2( b2 - a2 );
   const real da4 ( trans(a1a2) * a2b2 );
   const real db4 ( trans(b1b2) * a2b2 );
   if( da4 >= real(0) && db4 <= real(0) ) {
      cp1 = a2;
      cp2 = b2;
      return;
   }


   //----- Checking the edge-vertex features -----
   // If one or both of the line segments have zero length, we will never get here. Therefore
   // we don't have to worry about possible divisions by zero in the following calculations.

   Vec3 n, k;

   const real la( trans(a1a2) * a1a2 );
   if( da1 >= real(0) && da3 <= real(0) ) {
      k = (da1 / la) * a1a2;
      n = a1b1 - k;
      if( trans(b1b2) * n >= real(0) ) {
         cp1 = a1 + k;
         cp2 = b1;
         return;
      }
   }

   if( da2 >= real(0) && da4 <= real(0) ) {
      k = (da2 / la) * a1a2;
      n = a1b2 - k;
      if( trans(b1b2) * n <= real(0) ) {
         cp1 = a1 + k;
         cp2 = b2;
         return;
      }
   }

   const real lb( trans(b1b2) * b1b2 );
   if( db1 <= real(0) && db2 >= real(0) ) {
      k = (-db1 / lb) * b1b2;
      n = -a1b1 - k;
      if( trans(a1a2) * n >= real(0) ) {
         cp1 = a1;
         cp2= b1 + k;
         return;
      }
   }

   if( db3 <= real(0) && db4 >= real(0) ) {
      k = (-db3 / lb) * b1b2;
      n = -a2b1 - k;
      if( trans(a1a2) * n <= real(0) ) {
         cp1 = a2;
         cp2 = b1 + k;
         return;
      }
   }


   //----- Checking the edge-edge features -----

   const real scale( trans(a1a2) * b1b2 );
   real div( la * lb - sq(scale) );

   pe_INTERNAL_ASSERT( div > real(0), "Invalid division" );
   div = real(1) / div;

   const real s( ( lb    * da1 - scale * db1 ) * div );
   const real t( ( scale * da1 - la    * db1 ) * div );

   cp1 = a1 + s * a1a2;
   cp2 = b1 + t * b1b2;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Find the intersection point or the point of the closest approach between two straight
 * \brief lines \f$ l_1(s) = o_1+sd_1 \f$ and \f$ l_2(t) = o_2+td_2 \f$.
 * \ingroup geometry
 *
 * \param o1 A point on line \f$ l_1 \f$.
 * \param d1 The direction of line \f$ l_1 \f$.
 * \param o2 A point on line \f$ l_2 \f$.
 * \param d2 The direction of line \f$ l_2 \f$.
 * \param [out] s The resolved parameter for line \f$ l_1 \f$.
 * \param [out] t The resolved parameter for line \f$ l_2 \f$.
 * \return void

   \f[  o_1+sd_1 = o_2+td_2  \f]
   \f[  \Longleftrightarrow  \f]
   \f[  s = \frac{\det(o_2-o_1,d_2,d_1 \times d_2)}{\left \Vert d_1 \times d_2 \right \| ^2 }  \f]
   \f[  t = \frac{\det(o_2-o_1,d_1,d_1 \times d_2)}{\left \Vert d_1 \times d_2 \right \| ^2 }  \f]
 */
void intersectLines( const Vec3& o1, const Vec3& d1, const Vec3& o2, const Vec3& d2,
                     real& s, real& t )
{
   const real sqrlen( ( d1 % d2 ).sqrLength() );

   if( sqrlen < parallelThreshold )
   {
      s = real(0),
      t = real(0);
   }
   else
   {
      const real isqrlen( real(1) / sqrlen );
      const Vec3 p( o2 - o1 );
      const real dot(  trans(d1) * d2 );
      const real a  (  trans(d1) * p  );
      const real b  ( -trans(d2) * p  );

      s = ( a * ( trans(d2) * d2 ) + dot*b ) * isqrlen;
      t = ( b * ( trans(d1) * d1 ) + dot*a ) * isqrlen;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the closest point on triangle to point.
 * \ingroup geometry
 *
 * \param point The point whose closest point to ABC is calculated.
 * \param A One Point of the triangle.
 * \param B One Point of the triangle.
 * \param C One Point of the triangle.
 * \return Closest point on triangle ABC to \a point.
 *
 * \todo Review documentation
 */
Vec3 closestPointToTriangle( const Vec3& point, const Vec3& A, const Vec3& B, const Vec3& C )
{
   /* ADDITIONAL LICENSE TERMS:
    *
    * The code from this function is copied and modified from Real-Time Collision Detection by
    * Christer Ericson, published by Morgan Kaufmann Publishers, (C) 2005 Elsevier Inc. The
    * software license agreement in the book grants a nonexclusive license to use the software for
    * any purpose, commercial or non-commercial, as long as the above credit is included
    * identifying the original source of the software. This license supplements the GPLv3 license
    * in agreement with its section 7.
    */

   // Check if P in vertex region outside A
   const Vec3 ab = B - A;
   const Vec3 ac = C - A;
   const Vec3 ap = point - A;

   const Vec3T abT = trans(ab);
   const Vec3T acT = trans(ac);

   const real d1 = abT * ap;
   const real d2 = acT * ap;
   if (d1 <= real(0.0) && d2 <= real(0.0)) return A; // barycentric coordinates (1,0,0)

   // Check if P in vertex region outside B
   const Vec3 bp = point - B;
   const real d3 = abT * bp;
   const real d4 = acT * bp;
   if (d3 >= real(0.0) && d4 <= d3) return B; // barycentric coordinates (0,1,0)

   // Check if P in edge region of AB, if so return projection of P onto AB
   const real vc = d1*d4 - d3*d2;
   if (vc <= real(0.0) && d1 >= real(0.0) && d3 <= real(0.0)) {
      const real v = d1 / (d1 - d3);
      return A + v * ab; // barycentric coordinates (1-v,v,0)
   }

   // Check if P in vertex region outside C
   const Vec3 cp = point - C;
   const real d5 = abT * cp;
   const real d6 = acT * cp;
   if (d6 >= real(0.0) && d5 <= d6) return C; // barycentric coordinates (0,0,1)

   // Check if P in edge region of AC, if so return projection of P onto AC
   const real vb = d5*d2 - d1*d6;
   if (vb <= real(0.0) && d2 >= real(0.0) && d6 <= real(0.0)) {
      const real w = d2 / (d2 - d6);
      return A + w * ac; // barycentric coordinates (1-w,0,w)
   }

   // Check if P in edge region of BC, if so return projection of P onto BC
   const real va = d3*d6 - d5*d4;
   if (va <= real(0.0) && (d4 - d3) >= real(0.0) && (d5 - d6) >= real(0.0)) {
      real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      return B + w * (C - B); // barycentric coordinates (0,1-w,w)
   }

   // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
   real denom = real(1.0) / (va + vb + vc);
   real v = vb * denom;
   real w = vc * denom;
   return A + ab * v + ac * w; // = u*A + v*B + w*C, u = va * denom = 1.0 - v - w
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the closest point on triangle to point.
 * \ingroup geometry
 *
 * \param point The point whose closest point to ABC is calculated.
 * \param A One Point of the triangle.
 * \param B One Point of the triangle.
 * \param C One Point of the triangle.
 * \param a Barycentric coordinate belonging to point \a A
 * \param b Barycentric coordinate belonging to point \a B
 * \param c Barycentric coordinate belonging to point \a C
 * \return Closest point on triangle ABC to \a point.
 *
 * The closest point Calculated can be reconstructed as:
 * closestPoint = a*A + b*B + c*C
 *
 * a + b + c == 1 is always true.
 *
 * Barycentric coordinates not needed are set  to real(0.0).
 *
 * \todo Review documentation
 */
Vec3 closestPointToTriangle( const Vec3& point, const Vec3& A, const Vec3& B, const Vec3& C,
                             real& a, real& b, real& c )
{
   /* ADDITIONAL LICENSE TERMS:
    *
    * The code from this function is copied and modified from Real-Time Collision Detection by
    * Christer Ericson, published by Morgan Kaufmann Publishers, (C) 2005 Elsevier Inc. The
    * software license agreement in the book grants a nonexclusive license to use the software for
    * any purpose, commercial or non-commercial, as long as the above credit is included
    * identifying the original source of the software. This license supplements the GPLv3 license
    * in agreement with its section 7.
    */

   //reset all barycentric coordinates
   a = real(0.0);
   b = real(0.0);
   c = real(0.0);

   // Check if P in vertex region outside A
   const Vec3 ab = B - A;
   const Vec3 ac = C - A;
   const Vec3 ap = point - A;

   const Vec3T abT = trans(ab);
   const Vec3T acT = trans(ac);

   const real d1 = abT * ap;
   const real d2 = acT * ap;
   if (d1 <= real(0.0) && d2 <= real(0.0)) {
      a = real(1.0);
      return A; // barycentric coordinates (1,0,0)
   }

   // Check if P in vertex region outside B
   const Vec3 bp = point - B;
   const real d3 = abT * bp;
   const real d4 = acT * bp;
   if (d3 >= real(0.0) && d4 <= d3) {
      b = real(1.0);
      return B; // barycentric coordinates (0,1,0)
   }

   // Check if P in edge region of AB, if so return projection of P onto AB
   const real vc = d1*d4 - d3*d2;
   if (vc <= real(0.0) && d1 >= real(0.0) && d3 <= real(0.0)) {
      b = d1 / (d1 - d3);
      a = real(1.0) - b;
      return A + b * ab; // barycentric coordinates (1-b,b,0)
   }

   // Check if P in vertex region outside C
   const Vec3 cp = point - C;
   const real d5 = abT * cp;
   const real d6 = acT * cp;
   if (d6 >= real(0.0) && d5 <= d6) {
      c = real(1.0);
      return C; // barycentric coordinates (0,0,1)
   }

   // Check if P in edge region of AC, if so return projection of P onto AC
   const real vb = d5*d2 - d1*d6;
   if (vb <= real(0.0) && d2 >= real(0.0) && d6 <= real(0.0)) {
      c = d2 / (d2 - d6);
      a = real(1.0) - c;
      return A + c * ac; // barycentric coordinates (1-c,0,c)
   }

   // Check if P in edge region of BC, if so return projection of P onto BC
   const real va = d3*d6 - d5*d4;
   if (va <= real(0.0) && (d4 - d3) >= real(0.0) && (d5 - d6) >= real(0.0)) {
      c = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      b = real(1.0) - c;
      return B + c * (C - B); // barycentric coordinates (0,1-c,c)
   }

   // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
   const real denom = real(1.0) / (va + vb + vc);
   b = vb * denom;
   c = vc * denom;
   a = real(1.0) - b - c;
   return A + ab * b + ac * c; // = u*A + b*B + c*C, u = va * denom = 1.0 - b - c
}
//*************************************************************************************************

} // namespace pe
