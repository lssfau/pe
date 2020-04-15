//=================================================================================================
/*!
 *  \file src/core/Distance.cpp
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <sstream>
#include <stdexcept>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/Distance.h>
#include <pe/core/GeomTools.h>
#include <pe/core/MPI.h>
#include <pe/math/Accuracy.h>
#include <pe/math/Epsilon.h>
#include <pe/math/shims/Square.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  DISTANCE CALCULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Distance calculation between two rigid bodies.
 * \ingroup distance_calculation
 *
 * \param b1 The first rigid body.
 * \param b2 The second rigid body.
 * \return The minimum distance between the two rigid bodies.
 * \exception std::runtime_error Unknown body type.
 *
 * This function returns the minimum distance between the two rigid bodies \a b1 and \a b2.
 * Note that a negative distance indicates that the two bodies are overlapping.
 */
real distance( ConstBodyID b1, ConstBodyID b2 )
{
   // Performing a distance calculation between the two rigid bodies
   switch( b1->getType() )
   {
      // Performing a distance calculation between a sphere and the second rigid body
      case sphereType:
         switch( b2->getType() ) {
            case sphereType:
               return distanceSphereSphere( static_body_cast<const Sphere>( b1 ),
                                            static_body_cast<const Sphere>( b2 ) );
               break;
            case boxType:
               return distanceSphereBox( static_body_cast<const Sphere>( b1 ),
                                         static_body_cast<const Box>( b2 ) );
               break;
            case capsuleType:
               return distanceSphereCapsule( static_body_cast<const Sphere>( b1 ),
                                             static_body_cast<const Capsule>( b2 ) );
               break;
            case cylinderType:
               return distanceSphereCylinder( static_body_cast<const Sphere>( b1 ),
                                              static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return distanceSpherePlane( static_body_cast<const Sphere>( b1 ),
                                           static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return real(0);
               break;
            case unionType:
               return distanceSphereUnion( static_body_cast<const Sphere>( b1 ),
                                           static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a distance calculation between a box and the second rigid body
      case boxType:
         switch( b2->getType() ) {
            case sphereType:
               return distanceSphereBox( static_body_cast<const Sphere>( b2 ),
                                         static_body_cast<const Box>( b1 ) );
               break;
            case boxType:
               return distanceBoxBox( static_body_cast<const Box>( b1 ),
                                      static_body_cast<const Box>( b2 ) );
               break;
            case capsuleType:
               return distanceBoxCapsule( static_body_cast<const Box>( b1 ),
                                          static_body_cast<const Capsule>( b2 ) );
               break;
            case cylinderType:
               return distanceBoxCylinder( static_body_cast<const Box>( b1 ),
                                           static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return distanceBoxPlane( static_body_cast<const Box>( b1 ),
                                        static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return distanceBoxUnion( static_body_cast<const Box>( b1 ),
                                        static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a distance calculation between a capsule and the second rigid body
      case capsuleType:
         switch( b2->getType() ) {
            case sphereType:
               return distanceSphereCapsule( static_body_cast<const Sphere>( b2 ),
                                             static_body_cast<const Capsule>( b1 ) );
               break;
            case boxType:
               return distanceBoxCapsule( static_body_cast<const Box>( b2 ),
                                          static_body_cast<const Capsule>( b1 ) );
               break;
            case capsuleType:
               return distanceCapsuleCapsule( static_body_cast<const Capsule>( b1 ),
                                              static_body_cast<const Capsule>( b2 ) );
               break;
            case cylinderType:
               return distanceCapsuleCylinder( static_body_cast<const Capsule>( b1 ),
                                               static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return distanceCapsulePlane( static_body_cast<const Capsule>( b1 ),
                                            static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return real(0);
               break;
            case unionType:
               return distanceCapsuleUnion( static_body_cast<const Capsule>( b1 ),
                                            static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a distance calculation between a cylinder and the second rigid body
      case cylinderType:
         switch( b2->getType() ) {
            case sphereType:
               return distanceSphereCylinder( static_body_cast<const Sphere>( b2 ),
                                              static_body_cast<const Cylinder>( b1 ) );
               break;
            case boxType:
               return distanceBoxCylinder( static_body_cast<const Box>( b2 ),
                                           static_body_cast<const Cylinder>( b1 ) );
               break;
            case capsuleType:
               return distanceCapsuleCylinder( static_body_cast<const Capsule>( b2 ),
                                               static_body_cast<const Cylinder>( b1 ) );
               break;
            case cylinderType:
               return distanceCylinderCylinder( static_body_cast<const Cylinder>( b1 ),
                                                static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return distanceCylinderPlane( static_body_cast<const Cylinder>( b1 ),
                                             static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return real(0);
               break;
            case unionType:
               return distanceCylinderUnion( static_body_cast<const Cylinder>( b1 ),
                                             static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing a distance calculation between a plane and the second rigid body
      case planeType:
         switch( b2->getType() ) {
            case sphereType:
               return distanceSpherePlane( static_body_cast<const Sphere>( b2 ),
                                           static_body_cast<const Plane>( b1 ) );
               break;
            case boxType:
               return distanceBoxPlane( static_body_cast<const Box>( b2 ),
                                        static_body_cast<const Plane>( b1 ) );
               break;
            case capsuleType:
               return distanceCapsulePlane( static_body_cast<const Capsule>( b2 ),
                                            static_body_cast<const Plane>( b1 ) );
               break;
            case cylinderType:
               return distanceCylinderPlane( static_body_cast<const Cylinder>( b2 ),
                                             static_body_cast<const Plane>( b1 ) );
               break;
            case planeType:
               return distancePlanePlane( static_body_cast<const Plane>( b1 ),
                                          static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return real(0);
               break;
            case unionType:
               return distancePlaneUnion( static_body_cast<const Plane>( b1 ),
                                          static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performaing a distance calculation between a triangle mesh and the second rigid body
      case triangleMeshType:
         switch( b2->getType() ) {
            case sphereType:
               return real(0);
               break;
            case boxType:
               return real(0);
               break;
            case capsuleType:
               return real(0);
               break;
            case cylinderType:
               return real(0);
               break;
            case planeType:
               return real(0);
               break;
            case triangleMeshType:
               return real(0);
               break;
            case unionType:
               return real(0);
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }

      // Performing a distance calculation between a union and the second rigid body
      case unionType:
         switch( b2->getType() ) {
            case sphereType:
               return distanceSphereUnion( static_body_cast<const Sphere>( b2 ),
                                           static_body_cast<const Union>( b1 ) );
               break;
            case boxType:
               return distanceBoxUnion( static_body_cast<const Box>( b2 ),
                                        static_body_cast<const Union>( b1 ) );
               break;
            case capsuleType:
               return distanceCapsuleUnion( static_body_cast<const Capsule>( b2 ),
                                            static_body_cast<const Union>( b1 ) );
               break;
            case cylinderType:
               return distanceCylinderUnion( static_body_cast<const Cylinder>( b2 ),
                                             static_body_cast<const Union>( b1 ) );
               break;
            case planeType:
               return distancePlaneUnion( static_body_cast<const Plane>( b2 ),
                                          static_body_cast<const Union>( b1 ) );
               break;
            case triangleMeshType:
               return real(0);
               break;
            case unionType:
               return distanceUnionUnion( static_body_cast<const Union>( b1 ),
                                          static_body_cast<const Union>( b2 ) );
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
/*!\brief Distance calculation between a Sphere and a Box.
 * \ingroup distance_calculation
 *
 * \param s The sphere.
 * \param b The box.
 * \return The minimum distance between the sphere and the box.
 *
 * This function returns the distance between the sphere \a s and the box \a b. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceSphereBox( ConstSphereID s, ConstBoxID b )
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

      for( size_t i=1; i<3; ++i ) {
         const real tmp( std::fabs(p[i]) - l[i] );
         if( dist < tmp ) {
            dist = tmp;
         }
      }

      return dist - s->getRadius();
   }

   const Vec3 q( R * p );  // Transformation from the projection to the global world frame
   const Vec3 n( d - q );  // Normal direction of the contact (pointing from the box to the sphere)

   return ( n.length() - s->getRadius() );  // Distance between the sphere and the box
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Sphere and a Capsule.
 * \ingroup distance_calculation
 *
 * \param s The sphere.
 * \param c The capsule.
 * \return The minimum distance between the sphere and the capsule.
 *
 * This function returns the distance between the sphere \a s and the capsule \a c. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceSphereCapsule( ConstSphereID s, ConstCapsuleID c )
{
   const Vec3 rpos( c->pointFromWFtoBF( s->getPosition() ) );
   const real length( real(0.5) * c->getLength() );

   if( std::fabs(rpos[0]) > length ) {
      const Vec3 rpos_abs( std::fabs(rpos[0]) - length, rpos[1], rpos[2] );
      return rpos_abs.length() - s->getRadius() - c->getRadius();
   }
   else return std::sqrt( sq(rpos[1]) + sq(rpos[2]) ) - s->getRadius() - c->getRadius();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Sphere and a Cylinder.
 * \ingroup distance_calculation
 *
 * \param s The sphere.
 * \param c The cylinder.
 * \return The minimum distance between the sphere and the cylinder.
 *
 * This function returns the distance between the sphere \a s and the cylinder \a c. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceSphereCylinder( ConstSphereID /*s*/, ConstCylinderID /*c*/ )
{
   // TODO: distance implementation

   return real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between two Box primitives.
 * \ingroup distance_calculation
 *
 * \param b1 The first box.
 * \param b2 The second box.
 * \return The minimum distance between the two boxes.
 *
 * This function returns the distance between the two boxes \a b1 and \a b2. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceBoxBox( ConstBoxID b1, ConstBoxID b2 )
{
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Calculating the distance between box " << b1->getID() << " and box " << b2->getID();
   }

   real dist( -inf );

   const Rot3& R1( b1->getRotation() );
   const Rot3& R2( b2->getRotation() );

   // Calculating the rotation of box 2 relative to the orientation of box 1
   const Rot3 b2_rR( trans(R1) * R2 );

   // Calculating the absolute values of the relative rotation
   const Mat3 b2_rQ( fabs( b2_rR ) );

   // Calculating the position of box 2 relative to the orientation of box 1
   const Vec3 b2_rPos( trans(R1) * ( b2->getPosition() - b1->getPosition() ) );

   // Calculating the half Lengths of both boxes
   const real hl1[] = { real(0.5) * b1->getLengths()[0],
                        real(0.5) * b1->getLengths()[1],
                        real(0.5) * b1->getLengths()[2] };
   const real hl2[] = { real(0.5) * b2->getLengths()[0],
                        real(0.5) * b2->getLengths()[1],
                        real(0.5) * b2->getLengths()[2] };


   //----- Testing the three axes of box 1 -----

   real term1, term2, term3, sum, length;

   // l = au
   term1 = std::fabs(b2_rPos[0]) - ( hl1[0] + hl2[0]*b2_rQ[0] + hl2[1]*b2_rQ[1] + hl2[2]*b2_rQ[2] );

   if( term1 > dist ) {
      dist = term1;

      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Test 1 succeeded!\n"
                "      dist = " << dist;
      }
   }

   // l = av
   term1 = std::fabs(b2_rPos[1]) - ( hl1[1] + hl2[0]*b2_rQ[3] + hl2[1]*b2_rQ[4] + hl2[2]*b2_rQ[5] );

   if( term1 > dist ) {
      dist = term1;

      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Test 2 succeeded!\n"
                "      dist = " << dist;
      }
   }

   // l = aw
   term1 = std::fabs(b2_rPos[2]) - ( hl1[2] + hl2[0]*b2_rQ[6] + hl2[1]*b2_rQ[7] + hl2[2]*b2_rQ[8] );

   if( term1 > dist ) {
      dist = term1;

      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Test 3 succeeded!\n"
                "      dist = " << dist;
      }
   }


   //----- Testing the three axes of box 2 -----

   // l = bu
   term1 = b2_rPos[0]*b2_rR[0] + b2_rPos[1]*b2_rR[3] + b2_rPos[2]*b2_rR[6];
   term2 = std::fabs(term1) - ( hl1[0]*b2_rQ[0] + hl1[1]*b2_rQ[3] + hl1[2]*b2_rQ[6] + hl2[0] );

   if( term2 > dist ) {
      dist = term2;

      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Test 4 succeeded!\n"
                "      dist = " << dist;
      }
   }

   // l = bv
   term1 = b2_rPos[0]*b2_rR[1] + b2_rPos[1]*b2_rR[4] + b2_rPos[2]*b2_rR[7];
   term2 = std::fabs(term1) - ( hl1[0]*b2_rQ[1] + hl1[1]*b2_rQ[4] + hl1[2]*b2_rQ[7] + hl2[1] );

   if( term2 > dist ) {
      dist = term2;

      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Test 5 succeeded!\n"
                "      dist = " << dist;
      }
   }

   // l = bw
   term1 = b2_rPos[0]*b2_rR[2] + b2_rPos[1]*b2_rR[5] + b2_rPos[2]*b2_rR[8];
   term2 = std::fabs(term1) - ( hl1[0]*b2_rQ[2] + hl1[1]*b2_rQ[5] + hl1[2]*b2_rQ[8] + hl2[2] );

   if( term2 > dist ) {
      dist = term2;

      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Test 6 succeeded!\n"
                "      dist = " << dist;
      }
   }


   //----- Testing the all nine combinations of the axes of the two boxes -----

   // l = au x bu
   length = std::sqrt( sq(b2_rR[6]) + sq(b2_rR[3]) );

   if( length > epsilon ) {
      term1 = b2_rPos[2] * b2_rR[3] - b2_rPos[1] * b2_rR[6];
      term2 = hl1[1] * b2_rQ[6] + hl1[2] * b2_rQ[3];
      term3 = hl2[2] * b2_rQ[1] + hl2[1] * b2_rQ[2];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 7 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = au x bv
   length = std::sqrt( sq(b2_rR[7]) + sq(b2_rR[4]) );

   if( length > epsilon ) {
      term1 = b2_rPos[2] * b2_rR[4] - b2_rPos[1] * b2_rR[7];
      term2 = hl1[1] * b2_rQ[7] + hl1[2] * b2_rQ[4];
      term3 = hl2[0] * b2_rQ[2] + hl2[2] * b2_rQ[0];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 8 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = au x bw
   length = std::sqrt( sq(b2_rR[8]) + sq(b2_rR[5]) );

   if( length > epsilon ) {
      term1 = b2_rPos[2] * b2_rR[5] - b2_rPos[1] * b2_rR[8];
      term2 = hl1[1] * b2_rQ[8] + hl1[2] * b2_rQ[5];
      term3 = hl2[1] * b2_rQ[0] + hl2[0] * b2_rQ[1];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 9 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = av x bu
   length = std::sqrt( sq(b2_rR[6]) + sq(b2_rR[0]) );

   if( length > epsilon ) {
      term1 = b2_rPos[0] * b2_rR[6] - b2_rPos[2] * b2_rR[0];
      term2 = hl1[2] * b2_rQ[0] + hl1[0] * b2_rQ[6];
      term3 = hl2[2] * b2_rQ[4] + hl2[1] * b2_rQ[5];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 10 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = av x bv
   length = std::sqrt( sq(b2_rR[7]) + sq(b2_rR[1]) );

   if( length > epsilon ) {
      term1 = b2_rPos[0] * b2_rR[7] - b2_rPos[2] * b2_rR[1];
      term2 = hl1[2] * b2_rQ[1] + hl1[0] * b2_rQ[7];
      term3 = hl2[0] * b2_rQ[5] + hl2[2] * b2_rQ[3];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 11 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = av x bw
   length = std::sqrt( sq(b2_rR[8]) + sq(b2_rR[2]) );

   if( length > epsilon ) {
      term1 = b2_rPos[0] * b2_rR[8] - b2_rPos[2] * b2_rR[2];
      term2 = hl1[2] * b2_rQ[2] + hl1[0] * b2_rQ[8];
      term3 = hl2[1] * b2_rQ[3] + hl2[0] * b2_rQ[4];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 12 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = aw x bu
   length = std::sqrt( sq(b2_rR[3]) + sq(b2_rR[0]) );

   if( length > epsilon ) {
      term1 = b2_rPos[1] * b2_rR[0] - b2_rPos[0] * b2_rR[3];
      term2 = hl1[0] * b2_rQ[3] + hl1[1] * b2_rQ[0];
      term3 = hl2[2] * b2_rQ[7] + hl2[1] * b2_rQ[8];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 13 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = aw x bv
   length = std::sqrt( sq(b2_rR[4]) + sq(b2_rR[1]) );

   if( length > epsilon ) {
      term1 = b2_rPos[1] * b2_rR[1] - b2_rPos[0] * b2_rR[4];
      term2 = hl1[0] * b2_rQ[4] + hl1[1] * b2_rQ[1];
      term3 = hl2[0] * b2_rQ[8] + hl2[2] * b2_rQ[6];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 14 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // l = aw x bw
   length = std::sqrt( sq(b2_rR[5]) + sq(b2_rR[2]) );

   if( length > epsilon ) {
      term1 = b2_rPos[1] * b2_rR[2] - b2_rPos[0] * b2_rR[5];
      term2 = hl1[0] * b2_rQ[5] + hl1[1] * b2_rQ[2];
      term3 = hl2[1] * b2_rQ[6] + hl2[0] * b2_rQ[7];
      sum   = std::fabs(term1) - ( term2 + term3 );
      sum  /= length;
      if( sum > dist && std::fabs( sum - dist ) > accuracy ) {
         dist = sum;

         pe_LOG_DEBUG_SECTION( log ) {
            log << "   Test 15 succeeded!\n"
                   "      dist = " << dist;
         }
      }
   }

   // Returning the calculated distance
   return dist;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Box and a Capsule.
 * \ingroup distance_calculation
 *
 * \param b The box.
 * \param c The capsule.
 * \return The minimum distance between the box and the capsule.
 *
 * This function returns the distance between the box \a b and the capsule \a c. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceBoxCapsule( ConstBoxID b, ConstCapsuleID c )
{
   const Rot3& R( c->getRotation() );

   // Computing the displacement of the spherical caps of the capsule in world space coordinates
   const Vec3 c_up( real(0.5)*c->getLength()*R[0],
                    real(0.5)*c->getLength()*R[3],
                    real(0.5)*c->getLength()*R[6] );

   // Computing the closest points on the axis of the cylinder and the box
   Vec3 cp, bp;
   getClosestLineBoxPoints( c->getPosition()-c_up, c->getPosition()+c_up,
                            b->getPosition(), b->getRotation(), b->getLengths(), cp, bp );

   const Vec3 normal( bp - cp );
   return ( normal.length() - c->getRadius() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Box and a Cylinder.
 * \ingroup distance_calculation
 *
 * \param b The box.
 * \param c The cylinder.
 * \return The minimum distance between the box and the cylinder.
 *
 * This function returns the distance between the box \a b and the cylinder \a c. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceBoxCylinder( ConstBoxID /*b*/, ConstCylinderID /*c*/ )
{
   // TODO: distance implementation

   return real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Box and a Plane.
 * \ingroup distance_calculation
 *
 * \param b The box.
 * \param p The plane.
 * \return The minimum distance between the box and the plane.
 *
 * This function returns the distance between the box \a b and the plane \a p. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceBoxPlane( ConstBoxID b, ConstPlaneID p )
{
   const Vec3& n  ( p->getNormal()   );
   const Vec3& pos( b->getPosition() );
   const Vec3& l  ( b->getLengths()  );
   const Rot3& R  ( b->getRotation() );

   // Projection of the box side lengths on the plane's normal vector.
   const real px( std::fabs( n[0]*R[0]+n[1]*R[3]+n[2]*R[6] )*l[0] );
   const real py( std::fabs( n[0]*R[1]+n[1]*R[4]+n[2]*R[7] )*l[1] );
   const real pz( std::fabs( n[0]*R[2]+n[1]*R[5]+n[2]*R[8] )*l[2] );

   return ( trans(n)*pos - p->getDisplacement() - real(0.5)*( px+py+pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between two Capsule primitives.
 * \ingroup distance_calculation
 *
 * \param c1 The first capsule.
 * \param c2 The second capsule.
 * \return The minimum distance between the two capsules.
 *
 * This function returns the distance between the two capsules \a c1 and \a c2. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceCapsuleCapsule( ConstCapsuleID c1, ConstCapsuleID c2 )
{
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

   // Calculating the normal vector between the two closest points
   const Vec3 normal( cp1 - cp2 );

   return ( normal.length() - c1->getRadius() - c2->getRadius() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Capsule and a Cylinder.
 * \ingroup distance_calculation
 *
 * \param ca The capsule.
 * \param cy The cylinder.
 * \return The minimum distance between the capsule and the cylinder.
 *
 * This function returns the distance between the capsule \a ca and the cylinder \a cy. Note
 * that a negative distance indicates that the two bodies are overlapping.
 */
real distanceCapsuleCylinder( ConstCapsuleID /*ca*/, ConstCylinderID /*cy*/ )
{
   // TODO: distance implementation

   return real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Capsule and a Plane.
 * \ingroup distance_calculation
 *
 * \param c The capsule.
 * \param p The plane.
 * \return The minimum distance between the capsule and the plane.
 *
 * This function returns the distance between the capsule \a c and the plane \a p. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceCapsulePlane( ConstCapsuleID c, ConstPlaneID p )
{
   // Computing the displacement of the upper cap sphere of the capsule in world frame coordinates
   const Vec3 c_up( real(0.5) * c->getLength() * c->getRotation()[0],
                    real(0.5) * c->getLength() * c->getRotation()[3],
                    real(0.5) * c->getLength() * c->getRotation()[6] );

   const real dist1( trans( c->getPosition() + c_up ) * p->getNormal() - p->getDisplacement() - c->getRadius() );
   const real dist2( trans( c->getPosition() - c_up ) * p->getNormal() - p->getDisplacement() - c->getRadius() );

   return min( dist1, dist2 );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between two Cylinder primitives.
 * \ingroup distance_calculation
 *
 * \param c1 The first cylinder.
 * \param c2 The second cylinder.
 * \return The minimum distance between the two cylinders.
 *
 * This function returns the distance between the two cylinders \a c1 and \a c2. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceCylinderCylinder( ConstCylinderID /*c1*/, ConstCylinderID /*c2*/ )
{
   // TODO: distance implementation

   return real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Distance calculation between a Cylinder and a Plane.
 * \ingroup distance_calculation
 *
 * \param c The cylinder.
 * \param p The plane.
 * \return The minimum distance between the cylinder and the plane.
 *
 * This function returns the distance between the cylinder \a c and the plane \a p. Note that
 * a negative distance indicates that the two bodies are overlapping.
 */
real distanceCylinderPlane( ConstCylinderID /*c*/, ConstPlaneID /*p*/ )
{
   // TODO: distance implementation

   return real(0);
}
//*************************************************************************************************

} // namespace pe
