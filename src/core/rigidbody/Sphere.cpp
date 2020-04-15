//=================================================================================================
/*!
 *  \file src/core/rigidbody/Sphere.cpp
 *  \brief Source file for the Sphere class
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
#include <pe/core/CollisionSystem.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/Settings.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/core/Visualization.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DetailSection.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Sphere class.
 *
 * \param sid Unique system-specific ID for the sphere.
 * \param uid User-specific ID for the sphere.
 * \param gpos Global geometric center of the sphere.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 */
Sphere::Sphere( id_t sid, id_t uid, const Vec3& gpos, real radius,
                MaterialID material, bool visible )
   : Parent( sid, uid, gpos, radius, material, visible )  // Initialization of the parent class
{
   // Checking the radius
   // Since the sphere constructor is never directly called but only used in a small number
   // of functions that already check the sphere arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid sphere radius" );

   // Registering the sphere for visualization
   Visualization::add( this );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiation constructor for the Sphere class.
 *
 * \param sid Unique system-specific ID for the sphere.
 * \param uid User-specific ID for the sphere.
 * \param gpos Global geometric center of the sphere.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the sphere's body frame in the global world frame.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 * \param fixed \a true to fix the sphere, \a false to unfix it.
 */
Sphere::Sphere( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                real radius, MaterialID material, bool visible, bool fixed )
   : Parent( sid, uid, gpos, radius, material, visible )  // Initialization of the parent class
{
   // Checking the radius
   // Since the sphere constructor is never directly called but only used in a small number
   // of functions that already check the sphere arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid sphere radius" );

   // Initializing the instantiated sphere
   remote_ = true;                   // Setting the remote flag
   rpos_   = rpos;                   // Setting the relative position
   q_      = q;                      // Setting the orientation
   R_      = q_.toRotationMatrix();  // Setting the rotation matrix

   if( fixed ) {
      fixed_   = true;               // Setting the fixed flag
      invMass_ = real(0);            // Setting the inverse total mass
      Iinv_    = real(0);            // Setting the inverse moment of inertia
   }

   // Registering the sphere for visualization
   Visualization::add( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Sphere class.
 */
Sphere::~Sphere()
{
   // Deregistering the sphere from visualization
   Visualization::remove( this );

   // Logging the destruction of the sphere
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed sphere " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the sphere visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the sphere visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the sphere visible/invisible in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere on one
 * process may invalidate the settings of the sphere on another process. In order to synchronize
 * all rigid bodies after local changes, the World::synchronize() function should be used to
 * update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 * neglected and overwritten by the settings of the rigid body on its local process!
 */
void Sphere::setVisible( bool visible )
{
   if( visible ^ visible_ ) {
      visible_ = visible;
      Visualization::changeVisibility( this );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the sphere.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global sphere inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a sphere contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::setPosition( real px, real py, real pz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global sphere inside an exclusive section" );

   gpos_ = Vec3( px, py, pz );

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box of the sphere
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the sphere.
 *
 * \param gpos The global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global sphere inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a sphere contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::setPosition( const Vec3& gpos )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global sphere inside an exclusive section" );

   gpos_ = gpos;

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box of the sphere
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the sphere.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * Changing the orientation of a sphere contained in a union causes no apperent changes due
 * to the shape of the sphere. The moment of inertia and the axis-aligned bounding box remain
 * unchanged.
 *
 * \b Note:
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not
 *   allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this
 *   results in a \a std::logic_error.
 */
void Sphere::setOrientation( real r, real i, real j, real k )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   // No updates of the moment of inertia and the axis-aligned bounding boxes
   // of the sphere are needed. Therefore the superordinate body doesn't have to
   // be notified of the rotation!

   q_ = Quat( r, i, j, k );     // Setting the orientation of the sphere
   R_ = q_.toRotationMatrix();  // Updating the rotation of the sphere

   wake();  // Waking the sphere from sleep mode
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the sphere.
 *
 * \param q The global orientation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * Changing the orientation of a sphere contained in a union causes no apperent changes due
 * to the shape of the sphere. The moment of inertia and the axis-aligned bounding box remain
 * unchanged.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere on one
 * process may invalidate the settings of the sphere on another process. In order to synchronize
 * all rigid bodies after local changes, the World::synchronize() function should be used to
 * update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 * neglected and overwritten by the settings of the rigid body on its local process!
 */
void Sphere::setOrientation( const Quat& q )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   // No updates of the moment of inertia and the axis-aligned bounding boxes
   // of the sphere are needed. Therefore the superordinate body doesn't have to
   // be notified of the rotation!

   q_ = q;                      // Setting the orientation of the sphere
   R_ = q_.toRotationMatrix();  // Updating the rotation of the sphere

   wake();  // Waking the sphere from sleep mode
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the center of mass of the sphere by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 * \exception std::logic_error Invalid translation of a global sphere inside an exclusive section.
 *
 * \b Note:
 * - Translating a sphere contained in a union changes the mass distribution and geometry of
 *   the union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Sphere::translate( real dx, real dy, real dz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global sphere inside an exclusive section" );

   gpos_[0] += dx;
   gpos_[1] += dy;
   gpos_[2] += dz;

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the center of mass of the sphere by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 * \exception std::logic_error Invalid translation of a global sphere inside an exclusive section.
 *
 * \b Note:
 * - Translating a sphere contained in a union changes the mass distribution and geometry of
 *   the union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Sphere::translate( const Vec3& dp )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global sphere inside an exclusive section" );

   gpos_ += dp;

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the sphere around the global rotation axis (x,y,z) by the rotation
 * \brief angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * Changing the orientation/rotation of the sphere. The sphere is rotated around its center of
 * mass around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 * If the sphere is rotated within a superordinate body, the rotation causes no apparent changes
 * due to the shape of the sphere. The moment of inertia and the axis-aligned bounding box remain
 * unchanged.
 *
 * \b Note:
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotate( real x, real y, real z, real angle )
{
   rotate( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * Changing the orientation/rotation of the sphere. The sphere is rotated around its center of
 * mass around the given axis \a axis by \a angle degrees (radian measure).\n
 * If the sphere is rotated within a superordinate body, the rotation causes no apparent changes
 * due to the shape of the sphere. The moment of inertia and the axis-aligned bounding box remain
 * unchanged.
 *
 * \b Note:
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotate( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   // No updates of the moment of inertia and the axis-aligned bounding boxes
   // of the sphere are needed. Therefore the superordinate body doesn't have to
   // be notified of the rotation!

   q_ = Quat( axis, angle ) * q_;  // Updating the orientation of the sphere
   R_ = q_.toRotationMatrix();     // Updating the rotation of the sphere

   wake();  // Waking the sphere from sleep mode
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * Changing the orientation/rotation of the sphere. The sphere is rotated around its center of
 * mass by the Euler angles \a xangle, \a yangle and \a zangle (all in radian measure). The
 * rotations are applied in the order x, y, and z.\n
 * If the sphere is rotated within a superordinate body, the rotation causes no apparent changes
 * due to the shape of the sphere. The moment of inertia and the axis-aligned bounding box remain
 * unchanged.
 *
 * \b Note:
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotate( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   // No updates of the moment of inertia and the axis-aligned bounding boxes
   // of the sphere are needed. Therefore the superordinate body doesn't have to
   // be notified of the rotation!

   // Updating the orientation of the sphere
   q_.rotateX( xangle );  // Rotation around the x-axis
   q_.rotateY( yangle );  // Rotation around the y-axis
   q_.rotateZ( zangle );  // Rotation around the z-axis

   R_ = q_.toRotationMatrix();  // Updating the rotation of the sphere

   wake();  // Waking the sphere from sleep mode
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * Changing the orientation/rotation of the sphere. The sphere is rotated around its center of
 * mass by the Euler angles \a euler (all components in radian measure). The rotations are
 * applied in the order x, y, and z.\n
 * If the sphere is rotated within a superordinate body, the rotation causes no apparent changes
 * due to the shape of the sphere. The moment of inertia and the axis-aligned bounding box remain
 * unchanged.
 *
 * \b Note:
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotate( const Vec3& euler )
{
   rotate( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * Changing the orientation/rotation of the sphere. The sphere is rotated around its center of
 * mass by the quaternion \a dq. \n
 * If the sphere is rotated within a superordinate body, the rotation causes no apparent changes
 * due to the shape of the sphere. The moment of inertia and the axis-aligned bounding box remain
 * unchanged.
 *
 * \b Note:
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotate( const Quat& dq )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   // No updates of the moment of inertia and the axis-aligned bounding boxes
   // of the sphere are needed. Therefore the superordinate body doesn't have to
   // be notified of the rotation!

   q_ = dq * q_;                // Updating the orientation of the sphere
   R_ = q_.toRotationMatrix();  // Updating the rotation of the sphere

   wake();  // Waking the sphere from sleep mode
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * This function rotates the sphere around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the sphere. The sphere is rotated
 * around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - In case the sphere is contained in a union, this rotation may change the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * This function rotates the sphere around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the sphere. The sphere is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - In case the sphere is contained in a union, this rotation may change the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotateAroundOrigin( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   const Quat dq( axis, angle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the sphere
   q_    = dq * q_;                // Updating the orientation of the sphere
   R_    = q_.toRotationMatrix();  // Updating the rotation of the sphere

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box of the sphere
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * This function rotates the sphere around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the sphere. The sphere is rotated
 * by the Euler angles \a xangle, \a yangle and \a zangle (all components in radian measure).
 * The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - In case the sphere is contained in a union, this rotation may change the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the sphere
   q_    = dq * q_;                // Updating the orientation of the sphere
   R_    = q_.toRotationMatrix();  // Updating the rotation of the sphere

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box of the sphere
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * This function rotates the sphere around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the sphere. The sphere is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - In case the sphere is contained in a union, this rotation may change the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * This function rotates the sphere around the given global coordiante \a point and changes
 * both the global position and the orientation/rotation of the sphere. The sphere is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - In case the sphere is contained in a union, this rotation may change the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   const Quat dq( axis, angle );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the sphere
   q_    = dq * q_;                  // Updating the orientation of the sphere
   R_    = q_.toRotationMatrix();    // Updating the rotation of the sphere

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box of the sphere
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the sphere around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global sphere inside an exclusive section.
 *
 * This function rotates the sphere around the given global coordinate \a point and changes
 * both the global position and the orientation/rotation of the sphere. The sphere is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - In case the sphere is contained in a union, this rotation may change the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) sphere
 *   on one process may invalidate the settings of the sphere on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the sphere is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Sphere::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global sphere inside an exclusive section" );

   const Quat dq( euler );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the sphere
   q_    = dq * q_;                  // Updating the orientation of the sphere
   R_    = q_.toRotationMatrix();    // Updating the rotation of the sphere

   Sphere::calcBoundingBox();    // Updating the axis-aligned bounding box of the sphere
   wake();               // Waking the sphere from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the sphere.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the sphere, \a false if not.
 */
bool Sphere::containsRelPoint( real px, real py, real pz ) const
{
   const Vec3 rpos( px, py, pz );
   return ( rpos.sqrLength() <= ( radius_*radius_ ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the sphere.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the sphere, \a false if not.
 */
bool Sphere::containsRelPoint( const Vec3& rpos ) const
{
   return ( rpos.sqrLength() <= ( radius_*radius_ ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the sphere.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the sphere, \a false if not.
 */
bool Sphere::containsPoint( real px, real py, real pz ) const
{
   const Vec3 gpos( px, py, pz );
   return ( ( gpos - gpos_ ).sqrLength() <= ( radius_*radius_ ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the sphere.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the sphere, \a false if not.
 */
bool Sphere::containsPoint( const Vec3& gpos ) const
{
   return ( ( gpos - gpos_ ).sqrLength() <= ( radius_*radius_ ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the sphere.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the surface of the sphere, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Sphere::isSurfaceRelPoint( real px, real py, real pz ) const
{
   const Vec3 rpos( px, py, pz );
   return ( std::fabs( rpos.sqrLength() - ( radius_*radius_ ) ) <= surfaceThreshold*surfaceThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the sphere.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the surface of the sphere, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Sphere::isSurfaceRelPoint( const Vec3& rpos ) const
{
   return ( std::fabs( rpos.sqrLength() - ( radius_*radius_ ) ) <= surfaceThreshold*surfaceThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the sphere.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the surface of the sphere, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Sphere::isSurfacePoint( real px, real py, real pz ) const
{
   const Vec3 gpos( px, py, pz );
   return ( std::fabs( ( gpos - gpos_ ).sqrLength() - ( radius_*radius_ ) ) <= surfaceThreshold*surfaceThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the sphere.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the surface of the sphere, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Sphere::isSurfacePoint( const Vec3& gpos ) const
{
   return ( std::fabs( ( gpos - gpos_ ).sqrLength() - ( radius_*radius_ ) ) <= surfaceThreshold*surfaceThreshold );
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate sphere.
 *
 * \param dp Change in the global position of the superordinate rigid body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. This movement involves a change in the global position and the axis-aligned
 * bounding box.
 */
void Sphere::update( const Vec3& dp )
{
   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Updating the global position
   gpos_ += dp;

   // Setting the axis-aligned bounding box
   Sphere::calcBoundingBox();

   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate sphere.
 *
 * \param dq Change in the orientation of the superordinate rigid body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational movement.
 * This movement involves a change in the global position, the orientation/rotation and the
 * axis-aligned bounding box of the sphere.
 */
void Sphere::update( const Quat& dq )
{
   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Calculating the new global position
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Setting the axis-aligned bounding box
   Sphere::calcBoundingBox();

   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a sphere.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the sphere output.
 * \return void
 */
void Sphere::print( std::ostream& os, const char* tab ) const
{
   using std::setw;

   os << tab << " Sphere " << uid_ << " with radius " << radius_ << "\n";

   if( verboseMode ) {
      os << tab << "   Fixed: " << fixed_ << " , sleeping: " << !awake_ << "\n";
   }

   os << tab << "   System ID         = " << sid_ << "\n"
      << tab << "   Total mass        = " << mass_ << "\n"
      << tab << "   Material          = " << Material::getName( material_ ) << "\n"
      << tab << "   Global position   = " << gpos_ << "\n"
      << tab << "   Relative position = " << rpos_ << "\n"
      << tab << "   Linear velocity   = " << getLinearVel() << "\n"
      << tab << "   Angular velocity  = " << getAngularVel() << "\n";

   if( verboseMode )
   {
      os << tab << "   Bounding box      = " << aabb_ << "\n"
         << tab << "   Quaternion        = " << q_ << "\n"
         << tab << "   Rotation matrix   = ( " << setw(9) << R_[0] << " , " << setw(9) << R_[1] << " , " << setw(9) << R_[2] << " )\n"
         << tab << "                       ( " << setw(9) << R_[3] << " , " << setw(9) << R_[4] << " , " << setw(9) << R_[5] << " )\n"
         << tab << "                       ( " << setw(9) << R_[6] << " , " << setw(9) << R_[7] << " , " << setw(9) << R_[8] << " )\n";

      os << std::setiosflags(std::ios::right)
         << tab << "   Moment of inertia = ( " << setw(9) << I_[0] << " , " << setw(9) << I_[1] << " , " << setw(9) << I_[2] << " )\n"
         << tab << "                       ( " << setw(9) << I_[3] << " , " << setw(9) << I_[4] << " , " << setw(9) << I_[5] << " )\n"
         << tab << "                       ( " << setw(9) << I_[6] << " , " << setw(9) << I_[7] << " , " << setw(9) << I_[8] << " )\n"
         << std::resetiosflags(std::ios::right);
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SPHERE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new sphere.
 * \ingroup sphere
 *
 * \param uid The user-specific ID of the sphere.
 * \param gpos The global position of the center of the sphere.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 * \return Handle for the new sphere.
 * \exception std::invalid_argument Invalid sphere radius.
 * \exception std::invalid_argument Invalid global sphere position.
 *
 * This function creates a sphere primitive in the \b pe simulation system. The sphere with
 * user-specific ID \a uid is placed at the global position \a gpos, has the radius \a radius,
 * and consists of the material \a material. The \a visible flag sets the sphere (in-)visible
 * in all visualizations.
 *
 * \image html sphere.png
 * \image latex sphere.eps "Sphere geometry" width=200pt
 *
 * The following code example illustrates the setup of a sphere:

   \code
   // Creating the iron sphere 1 with a radius of 2.5 at the global position (2,3,4).
   // Per default the sphere is visible in all visualizations. Note that the sphere is
   // automatically added to the simulation world and is immediately part of the entire
   // simulation. The function returns a handle to the newly created sphere, which can
   // be used to for instance rotate the sphere around the global y-axis.
   SphereID sphere = createSphere( 1, Vec3(2,3,4), 2.5, iron );
   sphere->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the sphere is created inside a pe::pe_CREATE_UNION section, the sphere is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron sphere 2 with radius 1.3 at the global position (-1,4,-5).
      // Since the sphere is created inside a pe_CREATE_UNION section, the sphere is
      // directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      createSphere( 2, Vec3(-1,4,-5), 1.3, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, spheres may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
PE_PUBLIC SphereID createSphere( id_t uid, const Vec3& gpos, real radius,
                       MaterialID material, bool visible )
{
   const bool global( GlobalSection::isActive() );

   // Checking the radius
   if( radius <= real(0) )
      throw std::invalid_argument( "Invalid sphere radius" );

   // Checking the global position of the sphere
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global sphere position" );

   // Creating a new sphere
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   SphereID sphere = new Sphere( sid, uid, gpos, radius, material, visible );

   // Checking if the sphere is created inside a global section
   if( global )
      sphere->setGlobal();

   // Checking if the sphere has to be permanently fixed
   else if( sphere->isAlwaysFixed() )
      sphere->setFixed( true );

   // Registering the new sphere with the default body manager
   try {
      theDefaultManager()->add( sphere );
   }
   catch( std::exception& ) {
      delete sphere;
      throw;
   }

   // Logging the successful creation of the sphere
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created sphere " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Radius          = " << radius << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return sphere;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote sphere.
 * \ingroup sphere
 *
 * \param sid The unique system-specific ID of the sphere.
 * \param uid The user-specific ID of the sphere.
 * \param gpos The global position of the center of the sphere.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the sphere's body frame in the global world frame.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 * \param fixed \a true to fix the sphere, \a false to unfix it.
 * \param reg \a true to register the object in the default body manager.
 * \return Handle for the new sphere.
 *
 * This function instantiates a copy of a sphere with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of a sphere residing on a remote
 * MPI process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
SphereID instantiateSphere( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                            const Quat& q, real radius, MaterialID material,
                            bool visible, bool fixed, bool reg )
{
   // Checking the radius
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid sphere radius" );

   // Instantiating the sphere
   SphereID sphere = new Sphere( sid, uid, gpos, rpos, q, radius, material, visible, fixed );

   // Registering the sphere with the default body manager
   if( reg ) {
      try {
         theDefaultManager()->add( sphere );
      }
      catch( std::exception& ) {
         delete sphere;
         throw;
      }
   }

   // Logging the successful instantiation of the sphere
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated sphere " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Radius          = " << radius << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return sphere;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for spheres.
 * \ingroup sphere
 *
 * \param os Reference to the output stream.
 * \param s Reference to a constant sphere object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Sphere& s )
{
   os << "--" << pe_BROWN << "SPHERE PARAMETERS" << pe_OLDCOLOR
      << "-------------------------------------------------------------\n";
   s.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for sphere handles.
 * \ingroup sphere
 *
 * \param os Reference to the output stream.
 * \param s Constant sphere handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstSphereID s )
{
   os << "--" << pe_BROWN << "SPHERE PARAMETERS" << pe_OLDCOLOR
      << "-------------------------------------------------------------\n";
   s->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
