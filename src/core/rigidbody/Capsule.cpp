//=================================================================================================
/*!
 *  \file src/core/rigidbody/Capsule.cpp
 *  \brief Source file for the capsule class
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
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/UnionSection.h>
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
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Capsule class.
 *
 * \param sid Unique system-specific ID for the capsule.
 * \param uid User-specific ID for the capsule.
 * \param gpos Global geometric center of the capsule.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the capsule.
 * \param visible Specifies if the capsule is visible in a visualization.
 *
 * The capsule is created lying along the x-axis.
 */
Capsule::Capsule( id_t sid, id_t uid, const Vec3& gpos, real radius,
                  real length, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, radius, length, material, visible )  // Initialization of the parent class
{
   // Checking the radius and the length
   // Since the capsule constructor is never directly called but only used in a small number
   // of functions that already check the capsule arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid capsule radius" );
   pe_INTERNAL_ASSERT( length > real(0), "Invalid capsule length" );

   // Registering the capsule for visualization
   Visualization::add( this );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiation constructor for the Capsule class.
 *
 * \param sid Unique system-specific ID for the capsule.
 * \param uid User-specific ID for the capsule.
 * \param gpos Global geometric center of the capsule.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the capsule's body frame in the global world frame.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the capsule.
 * \param visible Specifies if the capsule is visible in a visualization.
 * \param fixed \a true to fix the capsule, \a false to unfix it.
 *
 * The capsule is created lying along the x-axis.
 */
Capsule::Capsule( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                  real radius, real length, MaterialID material, bool visible, bool fixed )
   : Parent( sid, uid, gpos, radius, length, material, visible )  // Initialization of the parent class
{
   // Checking the radius and the length
   // Since the capsule constructor is never directly called but only used in a small number
   // of functions that already check the capsule arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid capsule radius" );
   pe_INTERNAL_ASSERT( length > real(0), "Invalid capsule length" );

   // Initializing the instantiated capsule
   remote_ = true;                   // Setting the remote flag
   rpos_   = rpos;                   // Setting the relative position
   q_      = q;                      // Setting the orientation
   R_      = q_.toRotationMatrix();  // Setting the rotation matrix

   if( fixed ) {
      fixed_   = true;               // Setting the fixed flag
      invMass_ = real(0);            // Setting the inverse total mass
      Iinv_    = real(0);            // Setting the inverse moment of inertia
   }

   // Registering the capsule for visualization
   Visualization::add( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Capsule class.
 */
Capsule::~Capsule()
{
   // Deregistering the capsule from visualization
   Visualization::remove( this );

   // Logging the destruction of the capsule
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed capsule " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the capsule visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the capsule visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the capsule visible/invisible in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 * on one process may invalidate the settings of the capsule on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
void Capsule::setVisible( bool visible )
{
   if( visible ^ visible_ ) {
      visible_ = visible;
      Visualization::changeVisibility( this );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the capsule.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global capsule inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a capsule contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::setPosition( real px, real py, real pz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global capsule inside an exclusive section" );

   gpos_ = Vec3( px, py, pz );

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box of the capsule
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the capsule.
 *
 * \param gpos The global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global capsule inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a capsule contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::setPosition( const Vec3& gpos )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global capsule inside an exclusive section" );

   gpos_ = gpos;

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box of the capsule
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the capsule.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a capsule contained in a union changes the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not
 *   allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this
 *   results in a \a std::logic_error.
 */
void Capsule::setOrientation( real r, real i, real j, real k )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   q_ = Quat( r, i, j, k );     // Setting the orientation of the capsule
   R_ = q_.toRotationMatrix();  // Updating the rotation of the capsule

   Capsule::calcBoundingBox();  // Updating the axis-aligned bounding box of the capsule
   wake();             // Waking the capsule from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the capsule.
 *
 * \param q The global orientation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a capsule contained in a union changes the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not
 *   allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this
 *   results in a \a std::logic_error.
 */
void Capsule::setOrientation( const Quat& q )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   q_ = q;                      // Setting the orientation of the capsule
   R_ = q_.toRotationMatrix();  // Updating the rotation of the capsule

   Capsule::calcBoundingBox();  // Updating the axis-aligned bounding box of the capsule
   wake();             // Waking the capsule from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the center of mass of the capsule by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 * \exception std::logic_error Invalid translation of a global capsule inside an exclusive section.
 *
 * \b Note:
 * - Translating a capsule contained in a union changes the mass distribution and geometry of
 *   the union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Capsule::translate( real dx, real dy, real dz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global capsule inside an exclusive section" );

   gpos_[0] += dx;
   gpos_[1] += dy;
   gpos_[2] += dz;

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the center of mass of the capsule by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 * \exception std::logic_error Invalid translation of a global capsule inside an exclusive section.
 *
 * \b Note:
 * - Translating a capsule contained in a union changes the mass distribution and geometry of
 *   the union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Capsule::translate( const Vec3& dp )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global capsule inside an exclusive section" );

   gpos_ += dp;

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the capsule around the global rotation axis (x,y,z) by the rotation
 * \brief angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * Changing the orientation/rotation of the capsule. The capsule is rotated around its center
 * of mass around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotate( real x, real y, real z, real angle )
{
   rotate( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * Changing the orientation/rotation of the capsule. The capsule is rotated around its center
 * of mass around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotate( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   q_ = Quat( axis, angle ) * q_;  // Updating the orientation of the capsule
   R_ = q_.toRotationMatrix();     // Updating the rotation of the capsule

   Capsule::calcBoundingBox();  // Updating the axis-aligned bounding box of the capsule
   wake();             // Waking the capsule from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * Changing the orientation/rotation of the capsule. The capsule is rotated around its center
 * of mass by the Euler angles \a xangle, \a yangle and \a zangle (all in radian measure). The
 * rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotate( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   // Updating the orientation of the box
   q_.rotateX( xangle );  // Rotation around the x-axis
   q_.rotateY( yangle );  // Rotation around the y-axis
   q_.rotateZ( zangle );  // Rotation around the z-axis

   R_ = q_.toRotationMatrix();  // Updating the rotation of the capsule

   Capsule::calcBoundingBox();  // Updating the axis-aligned bounding box of the capsule
   wake();             // Waking the capsule from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * Changing the orientation/rotation of the capsule. The capsule is rotated around its center
 * of mass by the Euler angles \a euler (all components in radian measure). The rotations are
 * applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotate( const Vec3& euler )
{
   rotate( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * Changing the orientation/rotation of the capsule. The capsule is rotated around its center
 * of mass by the quaternion \a dq. \n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotate( const Quat& dq )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   q_ = dq * q_;                // Updating the orientation of the capsule
   R_ = q_.toRotationMatrix();  // Updating the rotation of the capsule

   Capsule::calcBoundingBox();  // Updating the axis-aligned bounding box of the capsule
   wake();             // Waking the capsule from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * This function rotates the capsule around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the capsule. The capsule is rotated
 * around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * This function rotates the capsule around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the capsule. The capsule is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotateAroundOrigin( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   const Quat dq( axis, angle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the capsule
   q_    = dq * q_;                // Updating the orientation of the capsule
   R_    = q_.toRotationMatrix();  // Updating the rotation of the capsule

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box of the capsule
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * This function rotates the capsule around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the capsule. The capsule is rotated
 * by the Euler angles \a xangle, \a yangle and \a zangle (all components in radian measure).
 * The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the capsule
   q_    = dq * q_;                // Updating the orientation of the capsule
   R_    = q_.toRotationMatrix();  // Updating the rotation of the capsule

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box of the capsule
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * This function rotates the capsule around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the capsule. The capsule is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * This function rotates the capsule around the given global coordiante \a point and changes
 * both the global position and the orientation/rotation of the capsule. The capsule is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   const Quat dq( axis, angle );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the capsule
   q_    = dq * q_;                  // Updating the orientation of the capsule
   R_    = q_.toRotationMatrix();    // Updating the rotation of the capsule

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box of the capsule
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the capsule around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global capsule inside an exclusive section.
 *
 * This function rotates the capsule around the given global coordinate \a point and changes
 * both the global position and the orientation/rotation of the capsule. The capsule is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a capsule contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) capsule
 *   on one process may invalidate the settings of the capsule on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the capsule is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Capsule::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global capsule inside an exclusive section" );

   const Quat dq( euler );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the capsule
   q_    = dq * q_;                  // Updating the orientation of the capsule
   R_    = q_.toRotationMatrix();    // Updating the rotation of the capsule

   Capsule::calcBoundingBox();    // Updating the axis-aligned bounding box of the capsule
   wake();               // Waking the capsule from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the capsule.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the capsule, \a false if not.
 */
bool Capsule::containsRelPoint( real px, real py, real pz ) const
{
   const real xabs( std::fabs( px ) );         // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   if( xabs > hlength ) {
      return ( ( sq(xabs-hlength) + sq(py) + sq(pz) ) <= ( radius_ * radius_ ) );
   }
   else {
      return ( sq(py) + sq(pz) ) <= ( radius_ * radius_ );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the capsule.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the capsule, \a false if not.
 */
bool Capsule::containsRelPoint( const Vec3& rpos ) const
{
   const real xabs( std::fabs( rpos[0] ) );    // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   if( xabs > hlength ) {
      return ( ( sq(xabs-hlength) + sq(rpos[1]) + sq(rpos[2]) ) <= ( radius_ * radius_ ) );
   }
   else {
      return ( sq(rpos[1]) + sq(rpos[2]) ) <= ( radius_ * radius_ );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the capsule.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the capsule, \a false if not.
 */
bool Capsule::containsPoint( real px, real py, real pz ) const
{
   return containsRelPoint( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the capsule.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the capsule, \a false if not.
 */
bool Capsule::containsPoint( const Vec3& gpos ) const
{
   return containsRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the capsule.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the surface of the capsule, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Capsule::isSurfaceRelPoint( real px, real py, real pz ) const
{
   const real xabs( std::fabs( px ) );         // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   if( xabs > hlength ) {
      return ( std::fabs( sq(xabs-hlength) + sq(py) + sq(pz) - radius_*radius_ ) <= surfaceThreshold*surfaceThreshold );
   }
   else {
      return ( std::fabs( ( sq(py) + sq(pz) ) - ( radius_ * radius_ ) ) <= surfaceThreshold*surfaceThreshold );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the capsule.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the surface of the capsule, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Capsule::isSurfaceRelPoint( const Vec3& rpos ) const
{
   const real xabs( std::fabs( rpos[0] ) );    // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   if( xabs > hlength ) {
      return ( std::fabs( sq(xabs-hlength) + sq(rpos[1]) + sq(rpos[2]) - radius_*radius_ ) <= surfaceThreshold*surfaceThreshold );
   }
   else {
      return ( std::fabs( ( sq(rpos[1]) + sq(rpos[2]) ) - ( radius_ * radius_ ) ) <= surfaceThreshold*surfaceThreshold );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the capsule.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the surface of the capsule, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Capsule::isSurfacePoint( real px, real py, real pz ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the capsule.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the surface of the capsule, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Capsule::isSurfacePoint( const Vec3& gpos ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate capsule.
 *
 * \param dp Change in the global position of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. This movement involves a change in the global position and the axis-aligned
 * bounding box.
 */
void Capsule::update( const Vec3& dp )
{
   // Checking the state of the capsule
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid capsule state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Updating the global position
   gpos_ += dp;

   // Setting the axis-aligned bounding box
   Capsule::calcBoundingBox();

   // Checking the state of the capsule
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid capsule state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate capsule.
 *
 * \param dq Change in the orientation of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational movement.
 * This movement involves a change in the global position, the orientation/rotation and the
 * axis-aligned bounding box of the capsule.
 */
void Capsule::update( const Quat& dq )
{
   // Checking the state of the capsule
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid capsule state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Calculating the new global position
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Setting the axis-aligned bounding box
   Capsule::calcBoundingBox();

   // Checking the state of the capsule
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid capsule state detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a capsule.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the capsule output.
 * \return void
 */
void Capsule::print( std::ostream& os, const char* tab ) const
{
   using std::setw;

   os << tab << " Capsule " << uid_ << " with radius " << radius_ << " and length " << length_ << "\n";

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
//  CAPSULE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new capsule.
 * \ingroup capsule
 *
 * \param uid The user-specific ID of the capsule.
 * \param gpos The global position of the center of the capsule.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part of the capsule \f$ (0..\infty) \f$.
 * \param material The material of the capsule.
 * \param visible Specifies if the capsule is visible in a visualization.
 * \return Handle for the new capsule.
 * \exception std::invalid_argument Invalid capsule radius.
 * \exception std::invalid_argument Invalid capsule length.
 * \exception std::invalid_argument Invalid global capsule position.
 *
 * This function creates a capsule primitive in the \b pe simulation system. The capsule with
 * user-specific ID \a uid is placed at the global position \a (x,y,z), has the radius \a radius
 * and the length \a length, and consists of the material \a material. The \a visible flag sets
 * the capsule (in-)visible in all visualizations.
 *
 * \image html capsule.png
 * \image latex capsule.eps "Capsule geometry" width=200pt
 *
 * The following code example illustrates the setup of a capsule:

   \code
   // Creating the iron capsule 1 with a radius of 0.9 and a length 2.5 of at the global
   // position (2,3,4). Per default the capsule is visible in all visualizations. Note
   // that the capsule is automatically added to the simulation world and is immediately
   // part of the entire simulation. The function returns a handle to the newly created
   // capsule, which can be used to for instance rotate the capsule around the global
   // y-axis.
   CapsuleID capsule = createCapsule( 1, Vec3( 2.0, 3.0, 4.0 ), 0.9, 2.5, iron );
   capsule->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the capsule is created inside a pe::pe_CREATE_UNION section, the capsule is
 * automatically added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron capsule 2 with radius 1.3 and length 2.4 at the global position
      // (-1,4,-5). Since the capsule is created inside a pe_CREATE_UNION section, the capsule
      // is directly added to the union 'newunion' and is henceforth considered to be part
      // of the union.
      createCapsule( 2, Vec3( -1.0, 4.0, -5.0 ), 1.3, 2.4, iron );
      ...
   }
   \endcode

 * In case of an MPI parallel simulation, capsules may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
PE_PUBLIC CapsuleID createCapsule( id_t uid, const Vec3& gpos, real radius,
                                   real length, MaterialID material, bool visible )
{
   const bool global( GlobalSection::isActive() );

   // Checking the radius and the length
   if( radius  <= real(0) ) throw std::invalid_argument( "Invalid capsule radius" );
   if( length  <= real(0) ) throw std::invalid_argument( "Invalid capsule length" );

   // Checking the global position of the capsule
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global capsule position" );

   // Creating a new capsule
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   CapsuleID capsule = new Capsule( sid, uid, gpos, radius, length, material, visible );

   // Checking if the capsule is created inside a global section
   if( global )
      capsule->setGlobal();

   // Checking if the capsule has to be permanently fixed
   else if( capsule->isAlwaysFixed() )
      capsule->setFixed( true );

   // Registering the new capsule with the default body manager
   try {
      theDefaultManager()->add( capsule );
   }
   catch( std::exception& ) {
      delete capsule;
      throw;
   }

   // Logging the successful creation of the capsule
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created capsule " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Radius          = " << radius << "\n"
          << "   Length          = " << length << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return capsule;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote capsule.
 * \ingroup capsule
 *
 * \param sid The unique system-specific ID of the capsule.
 * \param uid The user-specific ID of the capsule.
 * \param gpos The global position of the center of the capsule.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the capsule's body frame in the global world frame.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part of the capsule \f$ (0..\infty) \f$.
 * \param material The material of the capsule.
 * \param visible Specifies if the capsule is visible in a visualization.
 * \param fixed \a true to fix the box, \a false to unfix it.
 * \param reg \a true to register the object in the default body manager.
 * \return Handle for the new capsule.
 *
 * This function instantiates a copy of a capsule with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of a capsule residing on a remote
 * MPI process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
CapsuleID instantiateCapsule( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                              real radius, real length, MaterialID material, bool visible, bool fixed, bool reg )
{
   // Checking the radius and the length
   pe_INTERNAL_ASSERT( radius > real(0), "Invalid capsule radius" );
   pe_INTERNAL_ASSERT( length > real(0), "Invalid capsule length" );

   // Instantiating the capsule
   CapsuleID capsule = new Capsule( sid, uid, gpos, rpos, q, radius, length, material, visible, fixed );

   // Registering the capsule with the default body manager
   if( reg ) {
      try {
         theDefaultManager()->add( capsule );
      }
      catch( std::exception& ) {
         delete capsule;
         throw;
      }
   }

   // Logging the successful instantiation of the capsule
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated capsule " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Radius          = " << radius << "\n"
          << "   Length          = " << length << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return capsule;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for capsules.
 * \ingroup capsule
 *
 * \param os Reference to the output stream.
 * \param c Reference to a constant capsule object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Capsule& c )
{
   os << "--" << pe_BROWN << "CAPSULE PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   c.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for capsule handles.
 * \ingroup capsule
 *
 * \param os Reference to the output stream.
 * \param c Constant capsule handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstCapsuleID c )
{
   os << "--" << pe_BROWN << "CAPSULE PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   c->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
