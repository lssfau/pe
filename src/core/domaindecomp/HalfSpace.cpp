//=================================================================================================
/*!
 *  \file src/core/domaindecomp/Process.cpp
 *  \brief Source file for the Process class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#include <cmath>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/Thresholds.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/math/Functions.h>
#include <pe/math/shims/Square.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the HalfSpace class.
 *
 * \param a The x-component of the boundary plane's normal vector.
 * \param b The y-component of the boundary plane's normal vector.
 * \param c The z-component of the boundary plane's normal vector.
 * \param d The boundary plane's displacement from the global origin.
 * \param dx The size of the process overlap \f$ [0..\infty) \f$.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid overlap size.
 *
 * This constructor creates a half space. This half space is defined
 * by a plane shaped boundary. The boundary
 * is specified by the given plane normal \a (a,b,c) and the displacement from the global
 * origin \a d. Note that the plane normal always points inside the half space and that the
 * displacement can be characterized as follows:
 *
 *  - > 0: The global origin is outside the half space\n
 *  - < 0: The global origin is inside the half space\n
 *  - = 0: The global origin is on the surface of the half space
 *
 * The local process and the remote process occupying the half space are slightly overlapping.
 * The size of this overlap is defined by \a dx, which has to be in the range \f$ [0..\infty] \f$.
 * In case a negative overlap size is specified, a \a std::invalid_argument exception is thrown.
 */
HalfSpace::HalfSpace( real a, real b, real c, real d, real dx )
   : normal_( a, b, c )                      // Normal of the boundary plane
   , d_     ( d )                            // Boundary displacement from the origin
   , dx_    ( max( contactThreshold, dx ) )  // Size of the process overlap
{
   // Checking the normal of the half space boundary
   if( normal_.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid half space boundary normal" );

   // Checking the size of the overlap
   if( dx < real(0) )
      throw std::invalid_argument( "Invalid overlap size" );

   // Normalizing the half space boundary normal
   normal_.normalize();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the HalfSpace class.
 *
 * \param normal The normal vector of the boundary plane, \f$ |n| > 0 \f$.
 * \param d The boundary plane's displacement from the global origin.
 * \param dx The size of the process overlap \f$ [0..\infty) \f$.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid overlap size.
 *
 * This constructor creates a half space. This half space is defined
 * by a plane shaped boundary. The boundary
 * is specified by the given plane normal \a normal and the displacement from the global
 * origin \a d. Note that the plane normal always points inside the half space and that the
 * displacement can be characterized as follows:
 *
 *  - > 0: The global origin is outside the half space\n
 *  - < 0: The global origin is inside the half space\n
 *  - = 0: The global origin is on the surface of the half space
 *
 * The local process and the remote process occupying the half space are slightly overlapping.
 * The size of this overlap is defined by \a dx, which has to be in the range \f$ [0..\infty] \f$.
 * In case a negative overlap size is specified, a \a std::invalid_argument exception is thrown.
 */
HalfSpace::HalfSpace( const Vec3& normal, real d, real dx )
   : normal_( normal )                       // Normal of the boundary plane
   , d_     ( d )                            // Boundary displacement from the origin
   , dx_    ( max( contactThreshold, dx ) )  // Size of the process overlap
{
   // Checking the normal of the half space boundary
   if( normal_.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid half space boundary normal" );

   // Checking the size of the overlap
   if( dx < real(0) )
      throw std::invalid_argument( "Invalid overlap size" );

   // Normalizing the half space boundary normal
   normal_.normalize();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the HalfSpace class.
 *
 * \param a The x-component of the boundary plane's normal vector.
 * \param b The y-component of the boundary plane's normal vector.
 * \param c The z-component of the boundary plane's normal vector.
 * \param x The global x-position of the boundary point.
 * \param y The global y-position of the boundary point.
 * \param z The global z-position of the boundary point.
 * \param dx The size of the process overlap \f$ [0..\infty) \f$.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid overlap size.
 *
 * This constructor creates a half space. This half space is defined
 * by a plane shaped boundary. The boundary is specified by the given plane normal
 * \a (a,b,c) and the global coordinate \a (x,y,z) that is contained in the boundary plane.
 * Note that the plane normal always points inside the half space.
 *
 * The local process and the remote process occupying the half space are slightly overlapping.
 * The size of this overlap is defined by \a dx, which has to be in the range \f$ [0..\infty] \f$.
 * In case a negative overlap size is specified, a \a std::invalid_argument exception is thrown.
 */
HalfSpace::HalfSpace( real a, real b, real c, real x, real y, real z, real dx )
   : normal_( a, b, c )                      // Normal of the boundary plane
   , d_     ( 0 )                            // Boundary displacement from the origin
   , dx_    ( max( contactThreshold, dx ) )  // Size of the process overlap
{
   // Checking the normal of the half space boundary
   if( normal_.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid half space boundary normal" );

   // Checking the size of the overlap
   if( dx < real(0) )
      throw std::invalid_argument( "Invalid overlap size" );

   // Normalizing the half space boundary normal
   normal_.normalize();

   // Calculating the displacement from the global origin
   d_ = trans(normal_) * Vec3(x,y,z);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the HalfSpace class.
 *
 * \param normal The normal vector of the boundary plane, \f$ |n| > 0 \f$.
 * \param gpos The global position of the boundary point.
 * \param dx The size of the process overlap \f$ [0..\infty) \f$.
 * \exception std::invalid_argument Invalid half space boundary normal.
 * \exception std::invalid_argument Invalid overlap size.
 *
 * This constructor creates a half space. This half space is defined
 * by a plane shaped boundary. The boundary is specified by the given plane normal
 * \a (a,b,c) and the global coordinate \a gpos that is contained in the boundary plane.
 * Note that the plane normal always points inside the half space.
 *
 * The local process and the remote process occupying the half space are slightly overlapping.
 * The size of this overlap is defined by \a dx, which has to be in the range \f$ [0..\infty] \f$.
 * In case a negative overlap size is specified, a \a std::invalid_argument exception is thrown.
 */
HalfSpace::HalfSpace( const Vec3& normal, const Vec3& gpos, real dx )
   : normal_( normal )                       // Normal of the boundary plane
   , d_     ( 0 )                            // Boundary displacement from the origin
   , dx_    ( max( contactThreshold, dx ) )  // Size of the process overlap
{
   // Checking the normal of the half space boundary
   if( normal_.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid half space boundary normal" );

   // Checking the size of the overlap
   if( dx < real(0) )
      throw std::invalid_argument( "Invalid overlap size" );

   // Normalizing the half space boundary normal
   normal_.normalize();

   // Calculating the displacement from the global origin
   d_ = trans(normal_) * gpos;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the given rigid body intersects with the half space.
 *
 * \param b The rigid body to be tested.
 * \return \a true if the rigid body intersects with the half space, \a false if not.
 * \exception std::invalid_argument Invalid infinite rigid body detected.
 *
 * This function tests whether the given rigid body is partially contained in the half space.
 * In case the body is partially contained in the half space the function returns \a true,
 * otherwise it returns \a false. Note that it is not
 * possible to test infinite rigid bodies (as for instance planes). The attempt to test an
 * infinite rigid body results in a \a std::invalid_argument exception.
 */
bool HalfSpace::intersectsWith( ConstBodyID b ) const
{
   // Checking if the rigid body is finite
   // Sending an infinite rigid body is not allowed.
   if( !b->isFinite() )
      throw std::invalid_argument( "Invalid infinite rigid body detected" );

   // Performing an overlap test between the rigid body and the half space
   switch( b->getType() ) {
      case sphereType:
         return HalfSpace::intersectsWith( static_body_cast<const Sphere>( b ) );
         break;
      case boxType:
         return HalfSpace::intersectsWith( static_body_cast<const Box>( b ) );
         break;
      case capsuleType:
         return HalfSpace::intersectsWith( static_body_cast<const Capsule>( b ) );
         break;
      case cylinderType:
         return HalfSpace::intersectsWith( static_body_cast<const Cylinder>( b ) );
         break;
      case triangleMeshType:
         return HalfSpace::intersectsWith( static_body_cast<const TriangleMesh>( b ) );
         break;
      case unionType:
         return HalfSpace::intersectsWith( static_body_cast<const Union>( b ) );
         break;
      default:
         std::ostringstream oss;
         oss << "Unknown body type (" << b->getType() << ")!";
         throw std::runtime_error( oss.str() );
         break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given sphere intersects with the half space.
 *
 * \param s The sphere to be tested.
 * \return \a true if the sphere intersects with the half space, \a false if not.
 *
 * This function tests whether the given sphere is partially contained in the half space.
 * In case the sphere is partially contained in the half space the function returns \a true,
 * otherwise it returns \a false.
 */
bool HalfSpace::intersectsWith( ConstSphereID s ) const
{
   if( trans(normal_) * s->getPosition() - d_ < -( s->getRadius() + dx_ ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given box intersects with the half space.
 *
 * \param b The box to be tested.
 * \return \a true if the box intersects with the half space, \a false if not.
 *
 * This function tests whether the given box is partially contained in the half space.
 * In case the box is partially contained in the half space the function returns \a true,
 * otherwise it returns \a false.
 */
bool HalfSpace::intersectsWith( ConstBoxID b ) const
{
   const Vec3& pos( b->getPosition() );
   const Vec3& l  ( b->getLengths()  );
   const Rot3& R  ( b->getRotation() );

   // Projection of the box side lengths on the plane's normal vector.
   const real px( std::fabs( normal_[0]*R[0]+normal_[1]*R[3]+normal_[2]*R[6] )*l[0] );
   const real py( std::fabs( normal_[0]*R[1]+normal_[1]*R[4]+normal_[2]*R[7] )*l[1] );
   const real pz( std::fabs( normal_[0]*R[2]+normal_[1]*R[5]+normal_[2]*R[8] )*l[2] );

   if( trans(normal_)*pos - d_ < -( real(0.5)*( px+py+pz ) + dx_ ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given capsule intersects with the half space.
 *
 * \param c The capsule to be tested.
 * \return \a true if the capsule intersects with the half space, \a false if not.
 *
 * This function tests whether the given capsule is partially contained in the half space.
 * In case the capsule is partially contained in the half space the function returns \a true,
 * otherwise it returns \a false.
 */
bool HalfSpace::intersectsWith( ConstCapsuleID c ) const
{
   const real  l( c->getLength()   );
   const Rot3& R( c->getRotation() );

   // Computing the displacement of the upper cap sphere of the capsule in world frame coordinates
   const Vec3 c_up( real(0.5)*l*R[0], real(0.5)*l*R[3], real(0.5)*l*R[6] );

   const real dist1( std::fabs( trans(normal_) * c_up ) );
   const real dist2( trans(normal_) * c->getPosition() );

   if( dist2 - d_ < -( dist1 + c->getRadius() + dx_ ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given cylinder intersects with the half space.
 *
 * \param c The cylinder to be tested.
 * \return \a true if the cylinder intersects with the half space, \a false if not.
 *
 * This function tests whether the given cylinder is partially contained in the half space.
 * In case the cylinder is partially contained in the half space the function returns \a true,
 * otherwise it returns \a false.
 */
bool HalfSpace::intersectsWith( ConstCylinderID c ) const
{
   using std::sqrt;

   const real  r ( c->getRadius() );
   const real  hl( real(0.5) * c->getLength() );
   const Rot3& R ( c->getRotation() );

   // Computing the displacement of the upper cylinder end point
   const Vec3 c_up( hl*R[0], hl*R[3], hl*R[6] );

   const real dist1( trans(normal_) * c->getPosition() );    // Projection of the position
   const real dist2( std::fabs( trans(normal_) * c_up ) );   // Projection of the cylinder length
   const real dist3( (r/hl) * sqrt( sq(hl) - sq(dist2) ) );  // Projection of the cylinder radius

   if( dist1 - d_ < -( dist2 - dist3 + dx_ ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given triangle mesh intersects with the half space.
 *
 * \param obj The triangle mesh to be tested.
 * \return \a true if the triangle mesh intersects with the half space, \a false if not.
 *
 * This function tests whether the given triangle mesh is contained in the half space. If it is,
 * then the function returns \a true, otherwise it returns \a false.
 */
bool HalfSpace::intersectsWith( ConstTriangleMeshID obj ) const
{
   // Determine support point sp of axis-aligned bounding box in direction of the half space normal
   //Vec3 sp( normal_[0] >= 0 ? obj->getAABB()[3] : obj->getAABB()[0], normal_[1] >= 0 ? obj->getAABB()[4] : obj->getAABB()[1], normal_[2] >= 0 ? obj->getAABB()[5] : obj->getAABB()[2] );

   // Determine support point p of triangle mesh in direction of the half space normal
   Vec3 sp( obj->supportContactThreshold( normal_ ) );

   // Check whether support point is inside half space expanded by boundary layer dx_
   if( trans(normal_) * sp - d_ < -dx_ )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given union intersects with the half space.
 *
 * \param u The union to be tested.
 * \return \a true if the union intersects with the half space, \a false if not.
 *
 * This function tests whether the given union is partially contained in the half space.
 * In case the union is partially contained in the half space the function returns \a true,
 * otherwise it returns \a false.
 */
bool HalfSpace::intersectsWith( ConstUnionID u ) const
{
   for( Union::ConstIterator it=u->begin(); it!=u->end(); ++it ) {
      if( HalfSpace::intersectsWith( *it ) ) return true;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained in the half space.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained in the half space, \a false if not.
 *
 * This function tests whether the given global coordinate is contained in the half space.
 * If the point is located on the surface of the half space then it is considered to be
 * part of the half space.
 */
bool HalfSpace::containsPoint( const Vec3& gpos ) const
{
   return ( trans(normal_) * gpos ) - d_ >=  real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is contained strictly in the half space.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is contained strictly in the half space, \a false if not.
 *
 * This function tests whether the given global coordinate is contained strictly in the half space.
 * If the point is located on the surface of the half space then it is considered to be \em not
 * part of the half space.
 */
bool HalfSpace::containsPointStrictly( const Vec3& gpos ) const
{
   return ( trans(normal_) * gpos ) - d_ >  real(0);
}
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Extracts all half spaces in the process geometry.
 *
 * \param halfspaces Descriptions of the half spaces in the process geometry are appended to the list on return.
 * \return void
 *
 * The description of each half space is a pair of the half space normal and its signed distance from the origin. If the process geometry does not contain any half spaces no descriptions are appended.
 */
void HalfSpace::extractHalfSpaces( std::list< std::pair< Vec3, real > >& halfspaces ) const
{
   halfspaces.push_back( std::make_pair( normal_, d_ ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the half space properties.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void HalfSpace::print( std::ostream& os ) const
{
   os << " Half space: normal=" << normal_ << ", displacement=" << d_ << ", dx=" << dx_ << "\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the half space properties.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the half space output.
 * \return void
 */
void HalfSpace::print( std::ostream& os, const char* tab ) const
{
   os << tab << "Half space: normal=" << normal_ << ", displacement=" << d_ << ", dx=" << dx_ << "\n";
}
//*************************************************************************************************

} // namespace
