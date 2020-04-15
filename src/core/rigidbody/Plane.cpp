//=================================================================================================
/*!
 *  \file src/core/rigidbody/Plane.cpp
 *  \brief Source file for the Plane class
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

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <pe/core/BodyManager.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/Visualization.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DetailSection.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Plane class.
 *
 * \param sid Unique system-specific ID for the plane.
 * \param uid User-specific ID for the plane.
 * \param gpos The global position (anchor point) of the plane.
 * \param normal The plane's normal in reference to the global world frame, \f$ |n| = 1 \f$.
 * \param d The displacement of the plane.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 *
 * The plane equation is: \f$ ax + by + cz = d \f$.\n
 * \a a, \a b and \a c are the x, y and z coordinate of the normal vector and \a d is the distance
 * from the origin to the plane.
 */
Plane::Plane( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal,
              real d, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, normal, d, material, visible )  // Initialization of the parent class
{
   // Checking the mass properties of the plane
   pe_INTERNAL_ASSERT( mass_    == real(0), "Mass of plane is not 0" );
   pe_INTERNAL_ASSERT( invMass_ == real(0), "Inverse mass of plane is not 0" );
   pe_INTERNAL_ASSERT( I_       == Mat3(0), "Moment of inertia of plane is not 0" );
   pe_INTERNAL_ASSERT( Iinv_    == Mat3(0), "Inverse moment of inertia of plane is not 0" );

   // Calculating the orientation and rotation
   // The default normal of a plane is <0,0,1>. The rotation of the plane is calculated
   // as the rotation of this default normal to the specified normal.
   if( normal[0]*normal[0] + normal[1]*normal[1] < accuracy )
      q_ = Quat( Vec3( 1, 0, 0 ), std::acos( normal[2] ) );
   else
      q_ = Quat( Vec3( -normal[1], normal[0], real(0) ), std::acos( normal[2] ) );

   R_ = q_.toRotationMatrix();

   // Setting the plane fixed
   fixed_  = true;

   // Registering the plane for visualization
   Visualization::add( this );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiation constructor for the Plane class.
 *
 * \param sid Unique system-specific ID for the plane.
 * \param uid User-specific ID for the plane.
 * \param gpos The global position (anchor point) of the plane.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the plane's body frame in the global world frame.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 *
 * The plane equation is: \f$ ax + by + cz = d \f$.\n
 * \a a, \a b and \a c are the x, y and z coordinate of the normal vector and \a d is the distance
 * from the origin to the plane.
 */
Plane::Plane( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
              MaterialID material, bool visible )
   : Parent( sid, uid, gpos, q.rotate( Vec3( 0, 0, 1 ) ), trans( q.rotate( Vec3( 0, 0, 1 ) ) ) * gpos, material, visible )  // Initialization of the parent class
{
   // Checking the mass properties of the plane
   pe_INTERNAL_ASSERT( mass_    == real(0), "Mass of plane is not 0" );
   pe_INTERNAL_ASSERT( invMass_ == real(0), "Inverse mass of plane is not 0" );
   pe_INTERNAL_ASSERT( I_       == Mat3(0), "Moment of inertia of plane is not 0" );
   pe_INTERNAL_ASSERT( Iinv_    == Mat3(0), "Inverse moment of inertia of plane is not 0" );

   // Initializing the instantiated cylinder
   remote_ = true;                   // Setting the remote flag
   rpos_   = rpos;                   // Setting the relative position
   q_      = q;                      // Setting the orientation
   R_      = q_.toRotationMatrix();  // Setting the rotation matrix

   // Setting the plane fixed
   fixed_  = true;

   // Registering the plane for visualization
   Visualization::add( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Plane class.
 */
Plane::~Plane()
{
   // Deregistering the capsule from visualization
   Visualization::remove( this );

   // Logging the destruction of the plane
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed plane " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the plane visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the plane visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the plane visible/invisible in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) plane on one
 * process may invalidate the settings of the plane on another process. In order to synchronize
 * all rigid bodies after local changes, the World::synchronize() function should be used to
 * update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 * neglected and overwritten by the settings of the rigid body on its local process!
 */
void Plane::setVisible( bool visible )
{
   if( visible ^ visible_ ) {
      visible_ = visible;
      Visualization::changeVisibility( this );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the plane.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 * \exception std::logic_error Invalid translation of a plane inside an exclusive section.
 *
 * This function sets the global position (anchor point) of the plane.
 *
 * \b Note:
 * - Setting the position of a plane contained in a union changes the geometry of the union.
 *   Therefore this may cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the position change must be applied on all processes.
 *   It is not allowed to change the position from within an pe_EXCLUSIVE_SECTION. The attempt
 *   to do this results in a \a std::logic_error.
 */
void Plane::setPosition( real px, real py, real pz )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a plane inside an exclusive section" );

   gpos_ = Vec3( px, py, pz );
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();    // Updating the axis-aligned bounding box of the plane
#if MOBILE_INFINITE
   wake();               // Waking the box from sleep mode
#else
   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
#endif
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the plane.
 *
 * \param gpos The global position.
 * \return void
 * \exception std::logic_error Invalid translation of a plane inside an exclusive section.
 *
 * This function sets the global position (anchor point) of the plane.
 *
 * \b Note: Translating a plane contained in a union changes the geometry of the union.
 * Therefore this may cause an invalidation of links contained in the union.
 *
 * \b Note:
 * - Setting the position of a plane contained in a union changes the geometry of the union.
 *   Therefore this may cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the position change must be applied on all processes.
 *   It is not allowed to change the position from within an pe_EXCLUSIVE_SECTION. The attempt
 *   to do this results in a \a std::logic_error.
 */
void Plane::setPosition( const Vec3& gpos )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a plane inside an exclusive section" );

   gpos_ = gpos;
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();    // Updating the axis-aligned bounding box of the plane
#if MOBILE_INFINITE
   wake();               // Waking the box from sleep mode
#else
   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
#endif
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the plane.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * This function sets the normal of the plane corresponding to the given global orientation,
 * where the initial orientation of the plane's normal is (0,0,1). This change of orientation
 * corresponds to a rotation around the anchor point of the plane, which is consequently not
 * changed. However, this rotation changes the distance (displacement) to the origin of the
 * global world frame.
 *
 * \b Note:
 * - Setting the orientation of a plane contained in a union changes the geometry of the union.
 *   Therefore this may cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the orientation change must be applied on all processes.
 *   It is not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt
 *   to do this results in a \a std::logic_error.
 */
void Plane::setOrientation( real r, real i, real j, real k )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   q_ = Quat( r, i, j, k );     // Updating the orientation of the plane
   R_ = q_.toRotationMatrix();  // Updating the rotation of the plane

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the displacement from the origin
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
#if MOBILE_INFINITE
   wake();               // Waking the box from sleep mode
#else
   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
#endif
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the plane.
 *
 * \param q The global orientation.
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * This function sets the normal of the plane corresponding to the given global orientation,
 * where the initial orientation of the plane's normal is (0,0,1). This change of orientation
 * corresponds to a rotation around the anchor point of the plane, which is consequently not
 * changed. However, this rotation changes the distance (displacement) to the origin of the
 * global world frame.
 *
 * \b Note:
 * - Setting the orientation of a plane contained in a union changes the geometry of the union.
 *   Therefore this may cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the orientation change must be applied on all processes.
 *   It is not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt
 *   to do this results in a \a std::logic_error.
 */
void Plane::setOrientation( const Quat& q )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   q_ = q;                      // Updating the orientation of the plane
   R_ = q_.toRotationMatrix();  // Updating the rotation of the plane

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the displacement from the origin
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   wake();             // Waking the box from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the global position of the plane by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 * \exception std::logic_error Invalid translation of a plane inside an exclusive section.
 *
 * \b Note:
 * - Translating a plane contained in a union changes the geometry of the union. Therefore this
 *   may cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the translation must be applied on all processes. It
 *   is not allowed to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to
 *   do this results in a \a std::logic_error.
 */
void Plane::translate( real dx, real dy, real dz )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a plane inside an exclusive section" );

   gpos_[0] += dx;
   gpos_[1] += dy;
   gpos_[2] += dz;
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();    // Updating the axis-aligned bounding box
   signalTranslation();  // Signaling the position change to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the global position of the plane by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 * \exception std::logic_error Invalid translation of a plane inside an exclusive section.
 *
 * \b Note:
 * - Translating a plane contained in a union changes the geometry of the union. Therefore this
 *   may cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the translation must be applied on all processes. It
 *   is not allowed to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to
 *   do this results in a \a std::logic_error.
 */
void Plane::translate( const Vec3& dp )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a plane inside an exclusive section" );

   gpos_ += dp;
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();    // Updating the axis-aligned bounding box
   signalTranslation();  // Signaling the position change to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the plane around the global rotation axis (x,y,z) by the rotation
 * \brief angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around its anchor point
 * (its current global position) around the given axis \a axis by \a angle degrees (in radian
 * measure). This rotation changes the normal of the plane and its distance (displacement) to
 * the origin of the global world frame.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotate( real x, real y, real z, real angle )
{
   rotate( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around its anchor point
 * (its current global position) around the given axis \a axis by \a angle degrees (in radian
 * measure). This rotation changes the normal of the plane and its distance (displacement) to
 * the origin of the global world frame.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotate( const Vec3& axis, real angle )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   q_ = Quat( axis, angle ) * q_;  // Updating the orientation of the plane
   R_ = q_.toRotationMatrix();     // Updating the rotation of the plane

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the displacement from the origin
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   signalRotation();   // Signaling the change of orientation to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around its anchor point
 * (its current global position) by the Euler angles \a xangle, \a yangle, and \a zangle (all in
 * radian measure). This rotation changes the normal of the plane and its distance (displacement)
 * to the origin of the global world frame. The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotate( real xangle, real yangle, real zangle )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   // Updating the orientation of the plane
   q_.rotateX( xangle );  // Rotation around the x-axis
   q_.rotateY( yangle );  // Rotation around the y-axis
   q_.rotateZ( zangle );  // Rotation around the z-axis

   // Updating the rotation of the plane
   R_ = q_.toRotationMatrix();

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the displacement from the origin
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   signalRotation();   // Signaling the change of orientation to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around its anchor point
 * (its current global position) by the Euler angles \a xangle, \a yangle, and \a zangle (all in
 * radian measure). This rotation changes the normal of the plane and its distance (displacement)
 * to the origin of the global world frame. The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotate( const Vec3& euler )
{
   rotate( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around its anchor point
 * (its current global position) by the quaternion \a dq. This rotation changes the normal of
 * the plane and its distance (displacement) to the origin of the global world frame.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotate( const Quat& dq )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   q_ = dq * q_;                // Updating the orientation of the plane
   R_ = q_.toRotationMatrix();  // Updating the rotation of the plane

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the displacement from the origin
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   signalRotation();   // Signaling the change of orientation to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around the origin of
 * the global frame around the given axis \a (x,y,z) by \a angle degrees (radian measure).
 * Therefore the anchor point (global position) and the normal of the plane are changed, not
 * its distance (displacement) to the origin.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around the origin of
 * the global frame around the given axis \a (x,y,z) by \a angle degrees (radian measure).
 * Therefore the anchor point (global position) and the normal of the plane are changed, not
 * its distance (displacement) to the origin.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotateAroundOrigin( const Vec3& axis, real angle )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   const Quat dq( axis, angle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the plane
   q_    = dq * q_;                // Updating the orientation of the plane
   R_    = q_.toRotationMatrix();  // Updating the rotation of the plane

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   signalRotation();   // Signaling the change of orientation to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around the origin of
 * the global frame by the Euler angles \a xangle, \a yangle and \a zangle (all in radian
 * measure). Therefore the anchor point (global position) and the normal of the plane are
 * changed, not its distance (displacement) to the origin. The rotations are applied in the
 * order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the plane
   q_    = dq * q_;                // Updating the orientation of the plane
   R_    = q_.toRotationMatrix();  // Updating the rotation of the plane

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   signalRotation();   // Signaling the change of orientation to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * Changing the orientation/rotation of the plane. The plane is rotated around the origin of
 * the global frame by the Euler angles \a xangle, \a yangle and \a zangle (all in radian
 * measure). Therefore the anchor point (global position) and the normal of the plane are
 * changed, not its distance (displacement) to the origin. The rotations are applied in the
 * order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * This function rotates the plane around the given global coordinate \a point and changes
 * the global position, the displacement from the origin, and the orientation/rotation of the
 * plane. The plane is rotated around the given axis \a axis by \a angle degrees (radian
 * measure).\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   const Quat dq( axis, angle );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the box
   q_    = dq * q_;                  // Updating the orientation of the box
   R_    = q_.toRotationMatrix();    // Updating the rotation of the box

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the displacement from the origin
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   signalRotation();   // Signaling the change of orientation to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the plane around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a plane inside an exclusive section.
 *
 * This function rotates the plane around the given global coordinate \a point and changes
 * the global position, the displacement from the origin and the orientation/rotation of the
 * plane. The plane is rotated by the Euler angles \a euler (all components in radian measure).
 * The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a plane contained in a union changes the geometry of the union. Therefore this may
 *   cause an invalidation of links contained in the union.
 * - In a <b>MPI parallel simulation</b>, the rotation must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
void Plane::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a plane inside an exclusive section" );

   const Quat dq( euler );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the box
   q_    = dq * q_;                  // Updating the orientation of the box
   R_    = q_.toRotationMatrix();    // Updating the rotation of the box

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the displacement from the origin
   d_ = trans(normal_) * gpos_;

   Plane::calcBoundingBox();  // Updating the axis-aligned bounding box of the plane
   signalRotation();   // Signaling the change of orientation to the superordinate body

   // As an immobile, infinite body a plane doesn't have to be awakened from sleep mode!
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the plane.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the plane, \a false if not.
 */
bool Plane::containsRelPoint( real /*px*/, real /*py*/, real pz ) const
{
   return pz <= real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the plane.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the plane, \a false if not.
 */
bool Plane::containsRelPoint( const Vec3& rpos ) const
{
   return rpos[2] <= real(0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the plane.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the plane, \a false if not.
 */
bool Plane::containsPoint( real px, real py, real pz ) const
{
   const Vec3 gpos( px, py, pz );
   return ( ( trans(normal_) * gpos ) - d_ <= real(0) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the plane.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the plane, \a false if not.
 */
bool Plane::containsPoint( const Vec3& gpos ) const
{
   return ( ( trans(normal_) * gpos ) - d_ <= real(0) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the plane.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the plane, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Plane::isSurfaceRelPoint( real /*px*/, real /*py*/, real pz ) const
{
   return std::fabs( pz ) <= surfaceThreshold;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the plane.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the plane, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Plane::isSurfaceRelPoint( const Vec3& rpos ) const
{
   return std::fabs( rpos[2] ) <= surfaceThreshold;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the plane.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the plane, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Plane::isSurfacePoint( real px, real py, real pz ) const
{
   const Vec3 gpos( px, py, pz );
   return ( std::fabs( ( trans(normal_) * gpos ) - d_ ) <= surfaceThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the plane
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the plane, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Plane::isSurfacePoint( const Vec3& gpos ) const
{
   return ( std::fabs( ( trans(normal_) * gpos ) - d_ ) <= surfaceThreshold );
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate plane.
 *
 * \param dp Change in the global position of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. This movement involves a change of the global position, the displacement from
 * the origin and the axis-aligned bounding box.
 */
void Plane::update( const Vec3& dp )
{
   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Updating the global position and the displacement
   gpos_ += dp;
   d_ = trans(normal_) * gpos_;

   // Setting the axis-aligned bounding box
   Plane::calcBoundingBox();

   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate plane.
 *
 * \param dq Change in the orientation of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational
 * movement. This movement involves a change in the global position, the displacement from
 * the origin, the orientation/rotation, the normal and the axis-aligned bounding box of
 * the plane.
 */
void Plane::update( const Quat& dq )
{
   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Updating the normal of the plane ( R * <0,0,1> )
   normal_[0] = R_[2];
   normal_[1] = R_[5];
   normal_[2] = R_[8];

   // Updating the global position and the displacement
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );
   d_    = trans(normal_) * gpos_;

   // Setting the axis-aligned bounding box
   Plane::calcBoundingBox();

   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a plane.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the plane output.
 * \return void
 */
void Plane::print( std::ostream& os, const char* tab ) const
{
   using std::setw;

   os << tab << " Plane " << uid_ << " with normal " << normal_ << " and displacement " << d_ << "\n"
      << tab << "   System ID         = " << sid_ << "\n"
      << tab << "   Material          = " << Material::getName( material_ ) << "\n"
      << tab << "   Global position   = " << gpos_ << "\n";

   if( verboseMode )
   {
      os << tab << "   Relative position = " << rpos_ << "\n"
         << tab << "   Bounding box      = " << aabb_ << "\n"
         << tab << "   Quaternion        = " << q_ << "\n"
         << tab << "   Rotation matrix   = ( " << setw(9) << R_[0] << " , " << setw(9) << R_[1] << " , " << setw(9) << R_[2] << " )\n"
         << tab << "                       ( " << setw(9) << R_[3] << " , " << setw(9) << R_[4] << " , " << setw(9) << R_[5] << " )\n"
         << tab << "                       ( " << setw(9) << R_[6] << " , " << setw(9) << R_[7] << " , " << setw(9) << R_[8] << " )\n";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  PLANE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new plane.
 * \ingroup plane
 *
 * \param uid The user-specific ID of the plane.
 * \param normal The normal vector of the plane, \f$ |n| > 0 \f$.
 * \param d The plane's displacement from the global origin.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 * \return Handle for the new plane.
 * \exception std::invalid_argument Invalid plane parameters.
 * \exception std::logic_error Invalid creation of a plane inside an exclusive section.
 * \exception std::logic_error Invalid creation of a plane outside a global section.
 *
 * This function creates a new plane with the user-specific ID \a uid, the normal \a normal,
 * the distance/displacement from the origin of the global world frame \a d, and the material
 * \a material. The global position of the plane is per default set to the closest surface
 * point to the origin of the global world frame. The \a visible flag sets the plane
 * (in-)visible in all visualizations.
 *
 * \image html plane.png
 * \image latex plane.eps "Plane geometry" width=520pt
 *
 * The following code example illustrates the setup of a plane:

   \code
   // Creating the iron plane 1 with the normal (1,0,2) and the displacement 1.0. Per default
   // the plane is visible in all visualizations. Note that the plane is automatically added
   // to the simulation world and is immediately part of the entire simulation. The function
   // returns a handle to the newly created plane, which can be used to for instance rotate
   // the plane around the global y-axis.
   PlaneID plane = createPlane( 1, Vec3( 1.0, 0.0, 2.0 ), 1.0, iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the plane is created inside a pe::pe_CREATE_UNION section, the plane is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron plane 2 with the normal (-1,0,-2) and the displacement 1.0.
      // Since the plane is created inside a pe_CREATE_UNION section, the plane is directly
      // added to the union 'newunion' and is henceforth considered to be part of the union.
      createPlane( 2, Vec3( -1.0, 0.0, -2.0 ), 1.0, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, planes may only be created inside a pe_GLOBAL_SECTION
 * environment:

   \code
   pe_GLOBAL_SECTION {
      // Creating the global plane 1 with normal (0,0,1) and displacement 2.0. Since the plane
      // is created inside a pe_GLOBAL_SECTION it is globally known on all MPI processes.
      // Therefore any function call that might modify the plane (translations, rotations, ...)
      // has to be executed on all processes (i.e., it is invalid to modify the plane inside
      // a pe_EXCLUSIVE_SECTION)!
      createPlane( 1, Vec3( 0.0, 0.0, 1.0 ), 2.0, iron );
   }
   \endcode
 */
PE_PUBLIC PlaneID createPlane( id_t uid, Vec3 normal, real d, MaterialID material, bool visible )
{
   const bool global( GlobalSection::isActive() );

   // Checking the normal of the plane
   if( normal.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid plane normal!" );

   // Checking whether the plane is created inside an exclusive section
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid creation of a plane inside an exclusive section" );

   // Checking whether the plane is created inside a global section (in a MPI parallel simulation)
   if( MPISettings::isParallel() && !global )
      throw std::logic_error( "Invalid creation of a plane outside a global section" );

   // Normalizing the plane normal
   normal.normalize();

   // Creating a new plane
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   PlaneID plane = new Plane( sid, uid, normal*d, normal, d, material, visible );

   // Registering the new plane with the default body manager
   try {
      theDefaultManager()->add( plane );
   }
   catch( std::exception& ) {
      delete plane;
      throw;
   }

   // Logging the successful creation of the plane
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created plane " << sid << "\n"
          << "   User-ID      = " << uid << "\n"
          << "   Anchor point = " << plane->getPosition() << "\n"
          << "   Normal       = " << normal << "\n"
          << "   Displacement = " << d << "\n"
          << "   Material     = " << Material::getName( material );
   }

   return plane;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new plane containing the surface point \a gpos.
 * \ingroup plane
 *
 * \param uid The user-specific ID of the plane.
 * \param normal The normal vector of the plane, \f$ |n| > 0 \f$.
 * \param gpos The global position of the surface point.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 * \return Handle for the new plane.
 * \exception std::invalid_argument Invalid plane parameters.
 * \exception std::logic_error Invalid creation of a plane inside an exclusive section.
 * \exception std::logic_error Invalid creation of a plane outside a global section.
 *
 * This function creates a new plane with the user-specific ID \a uid, the normal \a normal, the
 * surface point \a gpos, and the material \a material. The given coordinate \a gpos additionally
 * specifies the anchor point of the plane. The \a visible flag sets the plane (in-)visible in
 * all visualizations.
 *
 * \image html plane.png
 * \image latex plane.eps "Plane geometry" width=520pt
 *
 * The following code example illustrates the setup of a plane:

   \code
   // Creating the iron plane 1 with the normal (1,0,2) and the anchor point (1,0,0). Per
   // default the plane is visible in all visualizations. Note that the plane is automatically
   // added to the simulation world and is immediately part of the entire simulation. The
   // function returns a handle to the newly created plane, which can be used to for instance
   // rotate the plane around the global y-axis.
   PlaneID plane = createPlane( 1, Vec3( 1.0, 0.0, 2.0 ), Vec3( 1.0, 0.0, 0.0 ), iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the plane is created inside a pe::pe_CREATE_UNION section, the plane is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron plane 2 with the normal (-1,0,-2) and the anchor point (-1,0,0).
      // Since the plane is created inside a pe_CREATE_UNION section, the plane is directly
      // added to the union 'newunion' and is henceforth considered to be part of the union.
      createPlane( 2, Vec3( -1.0, 0.0, -2.0 ), Vec3( -1.0, 0.0, 0.0 ), iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, planes may only be created inside a pe_GLOBAL_SECTION
 * environment:

   \code
   pe_GLOBAL_SECTION {
      // Creating the global plane 1 with normal (0,0,1) and anchor point (2,1,0). Since the
      // plane is created inside a pe_GLOBAL_SECTION it is globally known on all MPI processes.
      // Therefore any function call that might modify the plane (translations, rotations, ...)
      // has to be executed on all processes (i.e., it is invalid to modify the plane inside
      // a pe_EXCLUSIVE_SECTION)!
      createPlane( 1, Vec3( 0.0, 0.0, 1.0 ), Vec3( 2.0, 1.0, 0.0 ), iron );
   }
   \endcode
 */
PlaneID createPlane( id_t uid, Vec3 normal, const Vec3& gpos,
                     MaterialID material, bool visible )
{
   const bool global( GlobalSection::isActive() );

   // Checking the normal of the plane
   if( normal.sqrLength() == real(0) )
      throw std::invalid_argument( "Invalid plane normal!" );

   // Checking whether the plane is created inside an exclusive section
   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid creation of a plane inside an exclusive section" );

   // Checking whether the plane is created inside a global section (in a MPI parallel simulation)
   if( MPISettings::isParallel() && !global )
      throw std::logic_error( "Invalid creation of a plane outside a global section" );

   // Normalizing the plane normal
   normal.normalize();

   // Creating a new plane
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   PlaneID plane = new Plane( sid, uid, gpos, normal, trans(normal)*gpos, material, visible );

   // Registering the new plane with the default body manager
   try {
      theDefaultManager()->add( plane );
   }
   catch( std::exception& ) {
      delete plane;
      throw;
   }

   // Logging the successful creation of the plane
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created plane " << sid << "\n"
          << "   User-ID      = " << uid << "\n"
          << "   Anchor point = " << gpos << "\n"
          << "   Normal       = " << normal << "\n"
          << "   Displacement = " << plane->getDisplacement() << "\n"
          << "   Material     = " << Material::getName( material );
   }

   return plane;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote plane.
 * \ingroup cylinder
 *
 * \param sid The unique system-specific ID of the plane.
 * \param uid The user-specific ID of the plane.
 * \param gpos The global position of the surface point.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the plane's body frame in the global world frame.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 * \param reg \a true to register the object in the default body manager.
 * \return Handle for the new plane.
 *
 * This function instantiates a copy of a plane with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of a plane residing on a remote
 * MPI process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
PlaneID instantiatePlane( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                          const Quat& q, MaterialID material,
                          bool visible, bool reg ) {
   // Instantiating the plane
   PlaneID plane = new Plane( sid, uid, gpos, rpos, q, material, visible );

   // Registering the plane with the default body manager
   if( reg ) {
      try {
         theDefaultManager()->add( plane );
      }
      catch( std::exception& ) {
         delete plane;
         throw;
      }
   }

   // Logging the successful instantiation of the cylinder
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated plane " << sid << "\n"
          << "   User-ID      = " << uid << "\n"
          << "   Anchor point = " << gpos << "\n"
          << "   Normal       = " << plane->getNormal() << "\n"
          << "   Displacement = " << plane->getDisplacement() << "\n"
          << "   Material     = " << Material::getName( material );
   }

   return plane;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for planes.
 * \ingroup plane
 *
 * \param os Reference to the output stream.
 * \param p Reference to a constant plane object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Plane& p )
{
   os << "--" << pe_BROWN << "PLANE PARAMETERS" << pe_OLDCOLOR
      << "--------------------------------------------------------------\n";
   p.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for plane handles.
 * \ingroup plane
 *
 * \param os Reference to the output stream.
 * \param p Constant plane handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstPlaneID p )
{
   os << "--" << pe_BROWN << "PLANE PARAMETERS" << pe_OLDCOLOR
      << "--------------------------------------------------------------\n";
   p->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
