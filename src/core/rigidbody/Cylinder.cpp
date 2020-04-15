//=================================================================================================
/*!
 *  \file src/core/rigidbody/Cylinder.cpp
 *  \brief Source file for the cylinder class
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
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/core/Visualization.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/math/shims/Square.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DetailSection.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Cylinder class.
 *
 * \param sid Unique system-specific ID for the cylinder.
 * \param uid User-specific ID for the cylinder.
 * \param gpos Global geometric center of the cylinder.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 *
 * The cylinder is created lying along the x-axis.
 */
Cylinder::Cylinder( id_t sid, id_t uid, const Vec3& gpos, real radius,
                    real length, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, radius, length, material, visible )  // Initialization of the parent class
{
   // Checking the radius and the length
   // Since the cylinder constructor is never directly called but only used in a small number
   // of functions that already check the cylinder arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid cylinder radius" );
   pe_INTERNAL_ASSERT( length > real(0), "Invalid cylinder length" );

   // Registering the cylinder for visualization
   Visualization::add( this );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiation constructor for the Cylinder class.
 *
 * \param sid Unique system-specific ID for the cylinder.
 * \param uid User-specific ID for the cylinder.
 * \param gpos Global geometric center of the cylinder.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the cylinder's body frame in the global world frame.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 * \param fixed \a true to fix the cylinder, \a false to unfix it.
 *
 * The cylinder is created lying along the x-axis.
 */
Cylinder::Cylinder( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                    real radius, real length, MaterialID material, bool visible, bool fixed )
   : Parent( sid, uid, gpos, radius, length, material, visible )  // Initialization of the parent class
{
   // Checking the radius and the length
   // Since the cylinder constructor is never directly called but only used in a small number
   // of functions that already check the cylinder arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid cylinder radius" );
   pe_INTERNAL_ASSERT( length > real(0), "Invalid cylinder length" );

   // Initializing the instantiated cylinder
   remote_ = true;                   // Setting the remote flag
   rpos_   = rpos;                   // Setting the relative position
   q_      = q;                      // Setting the orientation
   R_      = q_.toRotationMatrix();  // Setting the rotation matrix

   if( fixed ) {
      fixed_   = true;               // Setting the fixed flag
      invMass_ = real(0);            // Setting the inverse total mass
      Iinv_    = real(0);            // Setting the inverse moment of inertia
   }

   // Registering the cylinder for visualization
   Visualization::add( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Cylinder class.
 */
Cylinder::~Cylinder()
{
   // Deregistering the cylinder from visualization
   Visualization::remove( this );

   // Logging the destruction of the cylinder
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed cylinder " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the cylinder visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the cylinder visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the cylinder visible/invisible in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 * on one process may invalidate the settings of the cylinder on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
void Cylinder::setVisible( bool visible )
{
   if( visible ^ visible_ ) {
      visible_ = visible;
      Visualization::changeVisibility( this );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the cylinder.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global cylinder inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a cylinder contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::setPosition( real px, real py, real pz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global cylinder inside an exclusive section" );

   gpos_ = Vec3( px, py, pz );

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box of the cylinder
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the cylinder.
 *
 * \param gpos The global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global cylinder inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a cylinder contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::setPosition( const Vec3& gpos )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global cylinder inside an exclusive section" );

   gpos_ = gpos;

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box of the cylinder
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the cylinder.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a cylinder contained in a union changes the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not
 *   allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this
 *   results in a \a std::logic_error.
 */
void Cylinder::setOrientation( real r, real i, real j, real k )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   q_ = Quat( r, i, j, k );     // Setting the orientation of the cylinder
   R_ = q_.toRotationMatrix();  // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();  // Updating the axis-aligned bounding box of the cylinder
   wake();             // Waking the cylinder from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the cylinder.
 *
 * \param q The global orientation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a cylinder contained in a union changes the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not
 *   allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this
 *   results in a \a std::logic_error.
 */
void Cylinder::setOrientation( const Quat& q )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   q_ = q;                      // Setting the orientation of the cylinder
   R_ = q_.toRotationMatrix();  // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();  // Updating the axis-aligned bounding box of the cylinder
   wake();             // Waking the cylinder from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the center of mass of the cylinder by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 * \exception std::logic_error Invalid translation of a global cylinder inside an exclusive section.
 *
 * \b Note:
 * - Translating a cylinder contained in a union changes the mass distribution and geometry of
 *   the union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Cylinder::translate( real dx, real dy, real dz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global cylinder inside an exclusive section" );

   gpos_[0] += dx;
   gpos_[1] += dy;
   gpos_[2] += dz;

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the center of mass of the cylinder by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 * \exception std::logic_error Invalid translation of a global cylinder inside an exclusive section.
 *
 * \b Note:
 * - Translating a cylinder contained in a union changes the mass distribution and geometry of
 *   the union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Cylinder::translate( const Vec3& dp )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global cylinder inside an exclusive section" );

   gpos_ += dp;

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the cylinder around the global rotation axis (x,y,z) by the rotation
 * \brief angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * Changing the orientation/rotation of the cylinder. The cylinder is rotated around its center
 * of mass around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotate( real x, real y, real z, real angle )
{
   rotate( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * Changing the orientation/rotation of the cylinder. The cylinder is rotated around its center
 * of mass around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotate( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   q_ = Quat( axis, angle ) * q_;  // Updating the orientation of the cylinder
   R_ = q_.toRotationMatrix();     // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();  // Updating the axis-aligned bounding box of the cylinder
   wake();             // Waking the cylinder from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * Changing the orientation/rotation of the cylinder. The cylinder is rotated around its center
 * of mass by the Euler angles \a xangle, \a yangle and \a zangle (all in radian measure). The
 * rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotate( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   // Updating the orientation of the box
   q_.rotateX( xangle );  // Rotation around the x-axis
   q_.rotateY( yangle );  // Rotation around the y-axis
   q_.rotateZ( zangle );  // Rotation around the z-axis

   R_ = q_.toRotationMatrix();  // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();  // Updating the axis-aligned bounding box of the cylinder
   wake();             // Waking the cylinder from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * Changing the orientation/rotation of the cylinder. The cylinder is rotated around its center
 * of mass by the Euler angles \a euler (all components in radian measure). The rotations are
 * applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotate( const Vec3& euler )
{
   rotate( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * Changing the orientation/rotation of the cylinder. The cylinder is rotated around its center
 * of mass by the quaternion \a dq. \n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotate( const Quat& dq )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   q_ = dq * q_;                // Updating the orientation of the cylinder
   R_ = q_.toRotationMatrix();  // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();  // Updating the axis-aligned bounding box of the cylinder
   wake();             // Waking the cylinder from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * This function rotates the cylinder around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the cylinder. The cylinder is rotated
 * around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * This function rotates the cylinder around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the cylinder. The cylinder is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotateAroundOrigin( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   const Quat dq( axis, angle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the cylinder
   q_    = dq * q_;                // Updating the orientation of the cylinder
   R_    = q_.toRotationMatrix();  // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box of the cylinder
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * This function rotates the cylinder around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the cylinder. The cylinder is rotated
 * by the Euler angles \a xangle, \a yangle and \a zangle (all components in radian measure).
 * The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the cylinder
   q_    = dq * q_;                // Updating the orientation of the cylinder
   R_    = q_.toRotationMatrix();  // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box of the cylinder
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * This function rotates the cylinder around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the cylinder. The cylinder is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * This function rotates the cylinder around the given global coordiante \a point and changes
 * both the global position and the orientation/rotation of the cylinder. The cylinder is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   const Quat dq( axis, angle );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the cylinder
   q_    = dq * q_;                  // Updating the orientation of the cylinder
   R_    = q_.toRotationMatrix();    // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box of the cylinder
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the cylinder around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global cylinder inside an exclusive section.
 *
 * This function rotates the cylinder around the given global coordinate \a point and changes
 * both the global position and the orientation/rotation of the cylinder. The cylinder is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a cylinder contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) cylinder
 *   on one process may invalidate the settings of the cylinder on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the cylinder is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Cylinder::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global cylinder inside an exclusive section" );

   const Quat dq( euler );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the cylinder
   q_    = dq * q_;                  // Updating the orientation of the cylinder
   R_    = q_.toRotationMatrix();    // Updating the rotation of the cylinder

   Cylinder::calcBoundingBox();    // Updating the axis-aligned bounding box of the cylinder
   wake();               // Waking the cylinder from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the cylinder.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the cylinder, \a false if not.
 */
bool Cylinder::containsRelPoint( real px, real py, real pz ) const
{
   const real xabs( std::fabs( px ) );         // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Cylinder half length

   return ( xabs <= hlength ) && ( sq(py) + sq(pz) ) <= ( radius_ * radius_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the cylinder.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the cylinder, \a false if not.
 */
bool Cylinder::containsRelPoint( const Vec3& rpos ) const
{
   const real xabs( std::fabs( rpos[0] ) );    // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   return ( xabs <= hlength ) && ( sq(rpos[1]) + sq(rpos[2]) ) <= ( radius_ * radius_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the cylinder.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the cylinder, \a false if not.
 */
bool Cylinder::containsPoint( real px, real py, real pz ) const
{
   return containsRelPoint( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the cylinder.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the cylinder, \a false if not.
 */
bool Cylinder::containsPoint( const Vec3& gpos ) const
{
   return containsRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the cylinder.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the surface of the cylinder, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Cylinder::isSurfaceRelPoint( real px, real py, real pz ) const
{
   const real xabs( std::fabs( px ) );         // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   // Early exit in case the x-coordinate is beyond the cylinder expansion
   if( xabs > hlength + surfaceThreshold )
      return false;

   const real yzdist( ( sq(py) + sq(pz) ) - ( radius_ * radius_ ) );  // Distance from the x-axis

   return ( std::fabs( yzdist ) <= surfaceThreshold ) ||
          ( std::fabs( xabs - hlength ) < surfaceThreshold && yzdist <= surfaceThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the cylinder.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the surface of the cylinder, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Cylinder::isSurfaceRelPoint( const Vec3& rpos ) const
{
   const real xabs( std::fabs( rpos[0] ) );    // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   // Early exit in case the x-coordinate is beyond the cylinder expansion
   if( xabs > hlength + surfaceThreshold )
      return false;

   const real yzdist( ( sq(rpos[1]) + sq(rpos[2]) ) - ( radius_ * radius_ ) );  // Distance from the x-axis

   return ( std::fabs( yzdist ) <= surfaceThreshold ) ||
          ( std::fabs( xabs - hlength ) < surfaceThreshold && yzdist <= surfaceThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the cylinder.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the surface of the cylinder, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Cylinder::isSurfacePoint( real px, real py, real pz ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the cylinder.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the surface of the cylinder, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Cylinder::isSurfacePoint( const Vec3& gpos ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the cylinder's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the cylinder and a negative value,
 * if the point lies outside the cylinder.
 */
real Cylinder::getRelDepth( real px, real py, real pz ) const
{
   const real xdist  ( real(0.5) * length_ - std::fabs( px ) );   // Distance from the caps
   const real yzdist ( radius_ - std::sqrt( sq(py) + sq(pz) ) );  // Distance from the x-axis

   if( xdist < real(0) ) {
      if( yzdist < real(0) )
         return -std::sqrt( sq(xdist) + sq(yzdist) );
      else
         return xdist;
   }
   else return min( yzdist, xdist );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the cylinder's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the cylinder and a negative value,
 * if the point lies outside the cylinder.
 */
real Cylinder::getRelDepth( const Vec3& rpos ) const
{
   const real xdist  ( real(0.5) * length_ - std::fabs( rpos[0] ) );        // Distance from the caps
   const real yzdist ( radius_ - std::sqrt( sq(rpos[1]) + sq(rpos[2]) ) );  // Distance from the x-axis

   if( xdist < real(0) ) {
      if( yzdist < real(0) )
         return -std::sqrt( sq(xdist) + sq(yzdist) );
      else
         return xdist;
   }
   else return min( yzdist, xdist );
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate cylinder.
 *
 * \param dp Change in the global position of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. This movement involves a change in the global position and the axis-aligned
 * bounding box.
 */
void Cylinder::update( const Vec3& dp )
{
   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Updating the global position
   gpos_ += dp;

   // Setting the axis-aligned bounding box
   Cylinder::calcBoundingBox();

   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate cylinder.
 *
 * \param dq Change in the orientation of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational movement.
 * This movement involves a change in the global position, the orientation/rotation and the
 * axis-aligned bounding box of the cylinder.
 */
void Cylinder::update( const Quat& dq )
{
   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Calculating the new global position
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Setting the axis-aligned bounding box
   Cylinder::calcBoundingBox();

   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a cylinder.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the cylinder output.
 * \return void
 */
void Cylinder::print( std::ostream& os, const char* tab ) const
{
   using std::setw;

   os << tab << " Cylinder " << uid_ << " with radius " << radius_ << " and length " << length_ << "\n";

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
//  CYLINDER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new cylinder.
 * \ingroup cylinder
 *
 * \param uid The user-specific ID of the cylinder.
 * \param gpos The global position of the center of the cylinder.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part of the cylinder \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 * \return Handle for the new cylinder.
 * \exception std::invalid_argument Invalid cylinder radius.
 * \exception std::invalid_argument Invalid cylinder length.
 * \exception std::invalid_argument Invalid global cylinder position.
 *
 * This function creates a cylinder primitive in the \b pe simulation system. The cylinder with
 * user-specific ID \a uid is placed at the global position \a (x,y,z), has the radius \a radius
 * and the length \a length, and consists of the material \a material. The \a visible flag sets
 * the cylinder (in-)visible in all visualizations.
 *
 * \image html cylinder.png
 * \image latex cylinder.eps "Cylinder geometry" width=200pt
 *
 * The following code example illustrates the setup of a cylinder:

   \code
   // Creating the iron cylinder 1 with a radius of 0.9 and a length 2.5 of at the global
   // position (2,3,4). Per default the cylinder is visible in all visualizations. Note
   // that the cylinder is automatically added to the simulation world and is immediately
   // part of the entire simulation. The function returns a handle to the newly created
   // cylinder, which can be used to for instance rotate the cylinder around the global
   // y-axis.
   CylinderID cylinder = createCylinder( 1, Vec3( 2.0, 3.0, 4.0 ), 0.9, 2.5, iron );
   cylinder->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the cylinder is created inside a pe::pe_CREATE_UNION section, the cylinder is
 * automatically added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron cylinder 2 with radius 1.3 and length 2.4 at the global position
      // (-1,4,-5). Since the cylinder is created inside a pe_CREATE_UNION section, the cylinder
      // is directly added to the union 'newunion' and is henceforth considered to be part
      // of the union.
      createCylinder( 2, Vec3( -1.0, 4.0, -5.0 ), 1.3, 2.4, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, cylinders may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
PE_PUBLIC CylinderID createCylinder( id_t uid, const Vec3& gpos, real radius,
                           real length, MaterialID material, bool visible )
{
   const bool global( GlobalSection::isActive() );

   // Checking the radius and the length
   if( radius  <= real(0) ) throw std::invalid_argument( "Invalid cylinder radius!" );
   if( length  <= real(0) ) throw std::invalid_argument( "Invalid cylinder length!" );

   // Checking the global position of the cylinder
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global cylinder position" );

   // Creating a new cylinder
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   CylinderID cylinder = new Cylinder( sid, uid, gpos, radius, length, material, visible );

   // Checking if the cylinder is created inside a global section
   if( global )
      cylinder->setGlobal();

   // Checking if the cylinder has to be permanently fixed
   else if( cylinder->isAlwaysFixed() )
      cylinder->setFixed( true );

   // Registering the new cylinder with the default body manager
   try {
      theDefaultManager()->add( cylinder );
   }
   catch( std::exception& ) {
      delete cylinder;
      throw;
   }

   // Logging the successful creation of the cylinder
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created cylinder " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Radius          = " << radius << "\n"
          << "   Length          = " << length << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return cylinder;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote cylinder.
 * \ingroup cylinder
 *
 * \param sid The unique system-specific ID of the cylinder.
 * \param uid The user-specific ID of the cylinder.
 * \param gpos The global position of the center of the cylinder.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the cylinder's body frame in the global world frame.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part of the cylinder \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 * \param fixed \a true to fix the cylinder, \a false to unfix it.
 * \param reg \a true to register the object in the default body manager.
 * \return Handle for the new cylinder.
 *
 * This function instantiates a copy of a cylinder with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of a cylinder residing on a remote
 * MPI process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
CylinderID instantiateCylinder( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                                real radius, real length, MaterialID material, bool visible, bool fixed, bool reg )
{
   // Checking the radius and the length
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid cylinder radius" );
   pe_INTERNAL_ASSERT( length > real(0), "Invalid cylinder length" );

   // Instantiating the cylinder
   CylinderID cylinder = new Cylinder( sid, uid, gpos, rpos, q, radius, length, material, visible, fixed );

   // Registering the cylinder with the default body manager
   if( reg ) {
      try {
         theDefaultManager()->add( cylinder );
      }
      catch( std::exception& ) {
         delete cylinder;
         throw;
      }
   }

   // Logging the successful instantiation of the cylinder
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated cylinder " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Radius          = " << radius << "\n"
          << "   Length          = " << length << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return cylinder;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for cylinders.
 * \ingroup cylinder
 *
 * \param os Reference to the output stream.
 * \param c Reference to a constant cylinder object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Cylinder& c )
{
   os << "--" << pe_BROWN << "CYLINDER PARAMETERS" << pe_OLDCOLOR
      << "-----------------------------------------------------------\n";
   c.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for cylinder handles.
 * \ingroup cylinder
 *
 * \param os Reference to the output stream.
 * \param c Constant cylinder handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstCylinderID c )
{
   os << "--" << pe_BROWN << "CYLINDER PARAMETERS" << pe_OLDCOLOR
      << "-----------------------------------------------------------\n";
   c->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
