//=================================================================================================
/*!
 *  \file src/core/Overlap.cpp
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
#include <pe/core/GeomTools.h>
#include <pe/core/MPI.h>
#include <pe/core/Overlap.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/shims/Square.h>


namespace pe {

//=================================================================================================
//
//  OVERLAP TEST FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
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
 *
 * \b Note: This function assumes that the two given rigid bodies are close to each other and
 * therefore potentially overlapping. No further coarse collision detection test (like checking
 * the axis-aligned bounding boxes of the two bodies) is applied!
 */
bool overlap_backend( ConstBodyID b1, ConstBodyID b2 )
{
   // Performing an overlap test between the two rigid bodies
   switch( b1->getType() )
   {
      // Performing an overlap test between a sphere and the second rigid body
      case sphereType:
         switch( b2->getType() ) {
            case sphereType:
               return overlapSphereSphere( static_body_cast<const Sphere>( b1 ),
                                           static_body_cast<const Sphere>( b2 ) );
               break;
            case boxType:
               return overlapSphereBox( static_body_cast<const Sphere>( b1 ),
                                        static_body_cast<const Box>( b2 ) );
               break;
            case capsuleType:
               return overlapSphereCapsule( static_body_cast<const Sphere>( b1 ),
                                            static_body_cast<const Capsule>( b2 ) );
               break;
            case cylinderType:
               return overlapSphereCylinder( static_body_cast<const Sphere>( b1 ),
                                             static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return overlapSpherePlane( static_body_cast<const Sphere>( b1 ),
                                          static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return overlapSphereUnion( static_body_cast<const Sphere>( b1 ),
                                          static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing an overlap test between a box and the second rigid body
      case boxType:
         switch( b2->getType() ) {
            case sphereType:
               return overlapSphereBox( static_body_cast<const Sphere>( b2 ),
                                        static_body_cast<const Box>( b1 ) );
               break;
            case boxType:
               return overlapBoxBox( static_body_cast<const Box>( b1 ),
                                     static_body_cast<const Box>( b2 ) );
               break;
            case capsuleType:
               return overlapBoxCapsule( static_body_cast<const Box>( b1 ),
                                         static_body_cast<const Capsule>( b2 ) );
               break;
            case cylinderType:
               return overlapBoxCylinder( static_body_cast<const Box>( b1 ),
                                          static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return overlapBoxPlane( static_body_cast<const Box>( b1 ),
                                       static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return overlapBoxUnion( static_body_cast<const Box>( b1 ),
                                       static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing an overlap test between a capsule and the second rigid body
      case capsuleType:
         switch( b2->getType() ) {
            case sphereType:
               return overlapSphereCapsule( static_body_cast<const Sphere>( b2 ),
                                            static_body_cast<const Capsule>( b1 ) );
               break;
            case boxType:
               return overlapBoxCapsule( static_body_cast<const Box>( b2 ),
                                         static_body_cast<const Capsule>( b1 ) );
               break;
            case capsuleType:
               return overlapCapsuleCapsule( static_body_cast<const Capsule>( b1 ),
                                             static_body_cast<const Capsule>( b2 ) );
               break;
            case cylinderType:
               return overlapCapsuleCylinder( static_body_cast<const Capsule>( b1 ),
                                              static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return overlapCapsulePlane( static_body_cast<const Capsule>( b1 ),
                                           static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return overlapCapsuleUnion( static_body_cast<const Capsule>( b1 ),
                                           static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing an overlap test between a cylinder and the second rigid body
      case cylinderType:
         switch( b2->getType() ) {
            case sphereType:
               return overlapSphereCylinder( static_body_cast<const Sphere>( b2 ),
                                             static_body_cast<const Cylinder>( b1 ) );
               break;
            case boxType:
               return overlapBoxCylinder( static_body_cast<const Box>( b2 ),
                                          static_body_cast<const Cylinder>( b1 ) );
               break;
            case capsuleType:
               return overlapCapsuleCylinder( static_body_cast<const Capsule>( b2 ),
                                              static_body_cast<const Cylinder>( b1 ) );
               break;
            case cylinderType:
               return overlapCylinderCylinder( static_body_cast<const Cylinder>( b1 ),
                                               static_body_cast<const Cylinder>( b2 ) );
               break;
            case planeType:
               return overlapCylinderPlane( static_body_cast<const Cylinder>( b1 ),
                                            static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return overlapCylinderUnion( static_body_cast<const Cylinder>( b1 ),
                                            static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performing an overlap test between a plane and the second rigid body
      case planeType:
         switch( b2->getType() ) {
            case sphereType:
               return overlapSpherePlane( static_body_cast<const Sphere>( b2 ),
                                          static_body_cast<const Plane>( b1 ) );
               break;
            case boxType:
               return overlapBoxPlane( static_body_cast<const Box>( b2 ),
                                       static_body_cast<const Plane>( b1 ) );
               break;
            case capsuleType:
               return overlapCapsulePlane( static_body_cast<const Capsule>( b2 ),
                                           static_body_cast<const Plane>( b1 ) );
               break;
            case cylinderType:
               return overlapCylinderPlane( static_body_cast<const Cylinder>( b2 ),
                                            static_body_cast<const Plane>( b1 ) );
               break;
            case planeType:
               return overlapPlanePlane( static_body_cast<const Plane>( b1 ),
                                         static_body_cast<const Plane>( b2 ) );
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return overlapPlaneUnion( static_body_cast<const Plane>( b1 ),
                                         static_body_cast<const Union>( b2 ) );
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }
         break;

      // Performaing an overlap test between a triangle mesh and the second rigid body
      case triangleMeshType:
         switch( b2->getType() ) {
            case sphereType:
               return false;
               break;
            case boxType:
               return false;
               break;
            case capsuleType:
               return false;
               break;
            case cylinderType:
               return false;
               break;
            case planeType:
               return false;
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return false;
               break;
            default:
               std::ostringstream oss;
               oss << "Unknown body type (" << b2->getType() << ")!";
               throw std::runtime_error( oss.str() );
               break;
         }

      // Performing an overlap test between a union and the second rigid body
      case unionType:
         switch( b2->getType() ) {
            case sphereType:
               return overlapSphereUnion( static_body_cast<const Sphere>( b2 ),
                                          static_body_cast<const Union>( b1 ) );
               break;
            case boxType:
               return overlapBoxUnion( static_body_cast<const Box>( b2 ),
                                       static_body_cast<const Union>( b1 ) );
               break;
            case capsuleType:
               return overlapCapsuleUnion( static_body_cast<const Capsule>( b2 ),
                                           static_body_cast<const Union>( b1 ) );
               break;
            case cylinderType:
               return overlapCylinderUnion( static_body_cast<const Cylinder>( b2 ),
                                            static_body_cast<const Union>( b1 ) );
               break;
            case planeType:
               return overlapPlaneUnion( static_body_cast<const Plane>( b2 ),
                                         static_body_cast<const Union>( b1 ) );
               break;
            case triangleMeshType:
               return false;
               break;
            case unionType:
               return overlapUnionUnion( static_body_cast<const Union>( b1 ),
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
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Sphere and a Box.
 * \ingroup overlap_tests
 *
 * \param s The sphere.
 * \param b The box.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The test whether a sphere and a box are overlapping is calculated by estimating the distance
 * of the sphere's center from the surface of the box. If the distance is smaller than the
 * sphere's radius (within the threshold value pe::contactThreshold), the two rigid bodies are
 * overlapping.
 */
bool overlapSphereBox( ConstSphereID s, ConstBoxID b )
{
   const Vec3 rPos( fabs( b->pointFromWFtoBF( s->getPosition() ) ) );
   const real hl[] = { real(0.5)*b->getLengths()[0],
                       real(0.5)*b->getLengths()[1],
                       real(0.5)*b->getLengths()[2] };
   real sqrDist( 0 );

   if( rPos[0] > hl[0] )
      sqrDist += sq( rPos[0] - hl[0] );
   if( std::fabs( rPos[1] ) > hl[1] )
      sqrDist += sq( rPos[1] - hl[1] );
   if( std::fabs( rPos[2] ) > hl[2] )
      sqrDist += sq( rPos[2] - hl[2] );

   return ( sqrDist - sq( s->getRadius() ) ) < contactThreshold;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Sphere and a Capsule.
 * \ingroup overlap_tests
 *
 * \param s The sphere.
 * \param c The capsule.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * A sphere and a capsule overlap if the sphere overlaps either with one of the caps of the
 * capsule or with the cylindrical part of the capsule. For the first case, a sphere/sphere
 * overlap test is performed. For the second case, the center of the sphere is projected
 * on the plane defined by the centerline of the capsule. In this plane, the distance between
 * the centers of the sphere and the capsule is calculated and compared to the radii of both
 * rigid bodies.
 */
bool overlapSphereCapsule( ConstSphereID s, ConstCapsuleID c )
{
   const real length( real(0.5) * c->getLength() );
   const Vec3 rPos( c->pointFromWFtoBF( s->getPosition() ) );

   if( std::fabs(rPos[0]) > length ) {
      const Vec3 rPos_abs( std::fabs(rPos[0]) - length, rPos[1], rPos[2] );
      return rPos_abs.sqrLength() < sq( s->getRadius() + c->getRadius() + contactThreshold );
   }
   else return ( sq(rPos[1]) + sq(rPos[2]) ) < sq( s->getRadius() + c->getRadius() + contactThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Sphere and a Cylinder.
 * \ingroup overlap_tests
 *
 * \param s The sphere.
 * \param c The cylinder.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * TODO
 */
bool overlapSphereCylinder( ConstSphereID /*s*/, ConstCylinderID /*c*/ )
{
   // TODO: Overlap implementation

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two Box primitives.
 * \ingroup overlap_tests
 *
 * \param b1 The first box.
 * \param b2 The second box.
 * \return \a true if the two boxes are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between two boxes is performed by using the separating axes theorem:
 * If the projection of two bodies A and B on a straight line is disjoint, then the two
 * bodies are not overlapping. In case of two boxes, the test includes a total of 15
 * possible separating axes:
 *
 *  - the x-, y- and z-axis of the body frame of box A
 *  - the x-, y- and z-axis of the body frame of box B
 *  - the nine combinations of the body frame axes of box A and B
 *
 * \image html overlapBoxBox.png
 * \image latex overlapBoxBox.eps "Box-Box overlap test" width=536pt
 */
bool overlapBoxBox( ConstBoxID b1, ConstBoxID b2 )
{
   // Calculating the rotation of box 2 relative to the orientation of box 1
   const Rot3 b2_rR( trans( b1->getRotation() ) * b2->getRotation() );

   // Calculating the absolute values of the relative rotation
   const Mat3 b2_rQ( fabs( b2_rR ) );

   // Calculating the relative position of box 2
   const Vec3 b2_rPos( b1->pointFromWFtoBF( b2->getPosition() ) );

   // Calculating the half lengths of both boxes
   const real b1_hl[] = { real(0.5) * b1->getLengths()[0],
                          real(0.5) * b1->getLengths()[1],
                          real(0.5) * b1->getLengths()[2] };
   const real b2_hl[] = { real(0.5) * b2->getLengths()[0],
                          real(0.5) * b2->getLengths()[1],
                          real(0.5) * b2->getLengths()[2] };


   //----- Testing the three axes of box 1 -----

   // l = au
   if( std::fabs(b2_rPos[0]) > ( b1_hl[0] + b2_hl[0]*b2_rQ[0] + b2_hl[1]*b2_rQ[1] + b2_hl[2]*b2_rQ[2] ) ) {
      return false;
   }

   // l = av
   if( std::fabs(b2_rPos[1]) > ( b1_hl[1] + b2_hl[0]*b2_rQ[3] + b2_hl[1]*b2_rQ[4] + b2_hl[2]*b2_rQ[5] ) ) {
      return false;
   }

   // l = aw
   if( std::fabs(b2_rPos[2]) > ( b1_hl[2] + b2_hl[0]*b2_rQ[6] + b2_hl[1]*b2_rQ[7] + b2_hl[2]*b2_rQ[8] ) ) {
      return false;
   }


   //----- Testing the three axes of box 2 -----

   // l = bu
   if( std::fabs( b2_rPos[0]*b2_rR[0] + b2_rPos[1]*b2_rR[3] + b2_rPos[2]*b2_rR[6] ) >
       ( b1_hl[0]*b2_rQ[0] + b1_hl[1]*b2_rQ[3] + b1_hl[2]*b2_rQ[6] + b2_hl[0] ) ) {
      return false;
   }

   // l = bv
   if( std::fabs( b2_rPos[0]*b2_rR[1] + b2_rPos[1]*b2_rR[4] + b2_rPos[2]*b2_rR[7] ) >
       ( b1_hl[0]*b2_rQ[1] + b1_hl[1]*b2_rQ[4] + b1_hl[2]*b2_rQ[7] + b2_hl[1] ) ) {
      return false;
   }

   // l = bw
   if( std::fabs( b2_rPos[0]*b2_rR[2] + b2_rPos[1]*b2_rR[5] + b2_rPos[2]*b2_rR[8] ) >
       ( b1_hl[0]*b2_rQ[2] + b1_hl[1]*b2_rQ[5] + b1_hl[2]*b2_rQ[8] + b2_hl[2] ) ) {
      return false;
   }


   //----- Testing the all nine combinations of the axes of the two boxes -----

   real tl, dA, dB;

   // l = au x bu
   tl = std::fabs( b2_rPos[2] * b2_rR[3] - b2_rPos[1] * b2_rR[6] );
   dA = b1_hl[1] * b2_rQ[6] + b1_hl[2] * b2_rQ[3];
   dB = b2_hl[2] * b2_rQ[1] + b2_hl[1] * b2_rQ[2];
   if( tl > dA + dB ) {
      return false;
   }

   // l = au x bv
   tl = std::fabs( b2_rPos[2] * b2_rR[4] - b2_rPos[1] * b2_rR[7] );
   dA = b1_hl[1] * b2_rQ[7] + b1_hl[2] * b2_rQ[4];
   dB = b2_hl[0] * b2_rQ[2] + b2_hl[2] * b2_rQ[0];
   if( tl > dA + dB ) {
      return false;
   }

   // l = au x bw
   tl = std::fabs( b2_rPos[2] * b2_rR[5] - b2_rPos[1] * b2_rR[8] );
   dA = b1_hl[1] * b2_rQ[8] + b1_hl[2] * b2_rQ[5];
   dB = b2_hl[1] * b2_rQ[0] + b2_hl[0] * b2_rQ[1];
   if( tl > dA + dB ) {
      return false;
   }

   // l = av x bu
   tl = std::fabs( b2_rPos[0] * b2_rR[6] - b2_rPos[2] * b2_rR[0] );
   dA = b1_hl[2] * b2_rQ[0] + b1_hl[0] * b2_rQ[6];
   dB = b2_hl[2] * b2_rQ[4] + b2_hl[1] * b2_rQ[5];
   if( tl > dA + dB ) {
      return false;
   }

   // l = av x bv
   tl = std::fabs( b2_rPos[0] * b2_rR[7] - b2_rPos[2] * b2_rR[1] );
   dA = b1_hl[2] * b2_rQ[1] + b1_hl[0] * b2_rQ[7];
   dB = b2_hl[0] * b2_rQ[5] + b2_hl[2] * b2_rQ[3];
   if( tl > dA + dB ) {
      return false;
   }

   // l = av x bw
   tl = std::fabs( b2_rPos[0] * b2_rR[8] - b2_rPos[2] * b2_rR[2] );
   dA = b1_hl[2] * b2_rQ[2] + b1_hl[0] * b2_rQ[8];
   dB = b2_hl[1] * b2_rQ[3] + b2_hl[0] * b2_rQ[4];
   if( tl > dA + dB ) {
      return false;
   }

   // l = aw x bu
   tl = std::fabs( b2_rPos[1] * b2_rR[0] - b2_rPos[0] * b2_rR[3] );
   dA = b1_hl[0] * b2_rQ[3] + b1_hl[1] * b2_rQ[0];
   dB = b2_hl[2] * b2_rQ[7] + b2_hl[1] * b2_rQ[8];
   if( tl > dA + dB ) {
      return false;
   }

   // l = aw x bv
   tl = std::fabs( b2_rPos[1] * b2_rR[1] - b2_rPos[0] * b2_rR[4] );
   dA = b1_hl[0] * b2_rQ[4] + b1_hl[1] * b2_rQ[1];
   dB = b2_hl[0] * b2_rQ[8] + b2_hl[2] * b2_rQ[6];
   if( tl > dA + dB ) {
      return false;
   }

   // l = aw x bw
   tl = std::fabs( b2_rPos[1] * b2_rR[2] - b2_rPos[0] * b2_rR[5] );
   dA = b1_hl[0] * b2_rQ[5] + b1_hl[1] * b2_rQ[2];
   dB = b2_hl[1] * b2_rQ[6] + b2_hl[0] * b2_rQ[7];
   if( tl > dA + dB ) {
      return false;
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Box and a Capsule.
 * \ingroup overlap_tests
 *
 * \param b The box.
 * \param c The capsule.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * For the overlap test between a box and a capsule, the shortest distance between the centerline
 * of the capsule and the box is calculated (see pe::getClosestLineBoxPoints) and compared to the
 * radius of the capsule.
 */
bool overlapBoxCapsule( ConstBoxID b, ConstCapsuleID c )
{
   const Rot3& R( c->getRotation() );

   // Computing the displacement of the upper cap spheres of the capsule in world frame coordinates
   const Vec3 c_up( real(0.5)*c->getLength()*R[0],
                    real(0.5)*c->getLength()*R[3],
                    real(0.5)*c->getLength()*R[6] );
   Vec3 p1, p2;

   getClosestLineBoxPoints( c->getPosition()+c_up, c->getPosition()-c_up,
                            b->getPosition(), b->getRotation(), b->getLengths(), p1, p2 );

   return ( ( p1 - p2 ).sqrLength() < sq( c->getRadius() + contactThreshold ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Box and a Cylinder.
 * \ingroup overlap_tests
 *
 * \param b The box.
 * \param c The cylinder.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * TODO
 */
bool overlapBoxCylinder( ConstBoxID /*b*/, ConstCylinderID /*c*/ )
{
   // TODO: Overlap implementation

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Box and a Plane.
 * \ingroup overlap_tests
 *
 * \param b The box.
 * \param p The plane.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between a box and a plane is calculated by comparing the projection of
 * the side lengths of the box on the plane's normal vector with the distance of the box's
 * center of mass from the plane.
 */
bool overlapBoxPlane( ConstBoxID b, ConstPlaneID p )
{
   const Vec3& n  ( p->getNormal()   );
   const Vec3& pos( b->getPosition() );
   const Vec3& l  ( b->getLengths()  );
   const Rot3& R  ( b->getRotation() );

   // Projection of the box side lengths on the plane's normal vector.
   const real px( std::fabs( n[0]*R[0]+n[1]*R[3]+n[2]*R[6] )*l[0] );
   const real py( std::fabs( n[0]*R[1]+n[1]*R[4]+n[2]*R[7] )*l[1] );
   const real pz( std::fabs( n[0]*R[2]+n[1]*R[5]+n[2]*R[8] )*l[2] );

   if( trans(n)*pos - p->getDisplacement() - real(0.5)*( px+py+pz ) > contactThreshold )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two Capsule primitives.
 * \ingroup overlap_tests
 *
 * \param c1 The first capsule.
 * \param c2 The second capsule.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * In order to test whether two capsules \a c1 and \a c2 are overlapping, the closest points
 * on both center lines are calculated (see pe::getClosestLineSegmentPoints). If the distance
 * between these two points is smaller than the sum of the radii of both capsules, an overlap
 * is detected.
 */
bool overlapCapsuleCapsule( ConstCapsuleID c1, ConstCapsuleID c2 )
{
   // Computing the displacement of the upper cap spheres of both capsules in world frame coordinates
   const Vec3 c1_up( real(0.5) * c1->getLength() * c1->getRotation()[0],
                     real(0.5) * c1->getLength() * c1->getRotation()[3],
                     real(0.5) * c1->getLength() * c1->getRotation()[6] );
   const Vec3 c2_up( real(0.5) * c2->getLength() * c2->getRotation()[0],
                     real(0.5) * c2->getLength() * c2->getRotation()[3],
                     real(0.5) * c2->getLength() * c2->getRotation()[6] );

   // Calculating the closest points of the two center lines
   Vec3 cp1, cp2;
   getClosestLineSegmentPoints( c1->getPosition()+c1_up, c1->getPosition()-c1_up,
                                c2->getPosition()+c2_up, c2->getPosition()-c2_up, cp1, cp2 );

   return ( ( cp1 - cp2 ).sqrLength() < sq( c1->getRadius() + c2->getRadius() + contactThreshold ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Capsule and a Cylinder.
 * \ingroup overlap_tests
 *
 * \param ca The capsule.
 * \param cy The cylinder.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * TODO
 */
bool overlapCapsuleCylinder( ConstCapsuleID /*ca*/, ConstCylinderID /*cy*/ )
{
   // TODO: Overlap implementation

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Capsule and a Plane.
 * \ingroup overlap_tests
 *
 * \param c The capsule.
 * \param p The plane.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * The overlap test between a capsule and a plane is performed by testing both spherical end
 * caps of the capsule. If for one of these spheres an overlap is detected, the test returns
 * \a true.
 */
bool overlapCapsulePlane( ConstCapsuleID c, ConstPlaneID p )
{
   // Computing the displacement of the upper cap sphere of the capsule in world frame coordinates
   const Vec3 c_up( real(0.5) * c->getLength() * c->getRotation()[0],
                    real(0.5) * c->getLength() * c->getRotation()[3],
                    real(0.5) * c->getLength() * c->getRotation()[6] );

   const real dist1( trans( c->getPosition() + c_up ) * p->getNormal() - p->getDisplacement() );
   const real dist2( trans( c->getPosition() - c_up ) * p->getNormal() - p->getDisplacement() );

   if( ( dist1 - c->getRadius() ) < contactThreshold || ( dist2 - c->getRadius() ) < contactThreshold )
      return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between two Cylinder primitives.
 * \ingroup overlap_tests
 *
 * \param c1 The first cylinder.
 * \param c2 The second cylinder.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * TODO
 */
bool overlapCylinderCylinder( ConstCylinderID /*c1*/, ConstCylinderID /*c2*/ )
{
   // TODO: Overlap implementation

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Overlap test between a Cylinder and a Plane.
 * \ingroup overlap_tests
 *
 * \param c The cylinder.
 * \param p The plane.
 * \return \a true if the two rigid bodies are overlapping, \a false if no overlap is detected.
 *
 * TODO
 */
bool overlapCylinderPlane( ConstCylinderID /*c*/, ConstPlaneID /*p*/ )
{
   // TODO: Overlap implementation

   return false;
}
//*************************************************************************************************

} // namespace pe
