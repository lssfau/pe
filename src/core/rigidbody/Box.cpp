//=================================================================================================
/*!
 *  \file src/core/rigidbody/Box.cpp
 *  \brief Source file for the Box class
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
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/core/Visualization.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
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
/*!\brief Constructor for the Box class.
 *
 * \param sid Unique system-specific ID for the box.
 * \param uid User-specific ID for the box.
 * \param gpos Global geometric center of the box.
 * \param lengths Side lengths of the box \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 */
Box::Box( id_t sid, id_t uid, const Vec3& gpos,
          const Vec3& lengths, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, lengths, material, visible )  // Initialization of the parent class
{
   // Checking the side lengths
   // Since the box constructor is never directly called but only used in a small number
   // of functions that already check the box arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( lengths[0] > real(0), "Invalid side length in x-dimension" );
   pe_INTERNAL_ASSERT( lengths[1] > real(0), "Invalid side length in y-dimension" );
   pe_INTERNAL_ASSERT( lengths[2] > real(0), "Invalid side length in z-dimension" );

   // Registering the box for visualization
   Visualization::add( this );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiation constructor for the Box class.
 *
 * \param sid Unique system-specific ID for the box.
 * \param uid User-specific ID for the box.
 * \param gpos Global geometric center of the box.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the box's body frame in the global world frame.
 * \param lengths Side lengths of the box \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 * \param fixed \a true to fix the box, \a false to unfix it.
 */
Box::Box( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
          const Vec3& lengths, MaterialID material, bool visible, bool fixed )
   : Parent( sid, uid, gpos, lengths, material, visible )  // Initialization of the parent class
{
   // Checking the side lengths
   // Since the box constructor is never directly called but only used in a small number
   // of functions that already check the box arguments, only asserts are used here to
   // double check the arguments.
   pe_INTERNAL_ASSERT( lengths[0] > real(0), "Invalid side length in x-dimension" );
   pe_INTERNAL_ASSERT( lengths[1] > real(0), "Invalid side length in y-dimension" );
   pe_INTERNAL_ASSERT( lengths[2] > real(0), "Invalid side length in z-dimension" );

   // Initializing the instantiated box
   remote_ = true;                   // Setting the remote flag
   rpos_   = rpos;                   // Setting the relative position
   q_      = q;                      // Setting the orientation
   R_      = q_.toRotationMatrix();  // Setting the rotation matrix

   if( fixed ) {
      fixed_   = true;               // Setting the fixed flag
      invMass_ = real(0);            // Setting the inverse total mass
      Iinv_    = real(0);            // Setting the inverse moment of inertia
   }

   // Registering the box for visualization
   Visualization::add( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Box class.
 */
Box::~Box()
{
   // Deregistering the box from visualization
   Visualization::remove( this );

   // Logging the destruction of the box
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed box " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the box visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the box visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the box visible/invisible in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 * process may invalidate the settings of the box on another process. In order to synchronize
 * all rigid bodies after local changes, the World::synchronize() function should be used to
 * update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 * neglected and overwritten by the settings of the rigid body on its local process!
 */
void Box::setVisible( bool visible )
{
   if( visible ^ visible_ ) {
      visible_ = visible;
      Visualization::changeVisibility( this );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the box.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global box inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a box contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::setPosition( real px, real py, real pz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global box inside an exclusive section" );

   gpos_ = Vec3( px, py, pz );

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box of the box
   wake();               // Waking the box from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the box.
 *
 * \param gpos The global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global box inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a box contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::setPosition( const Vec3& gpos )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global box inside an exclusive section" );

   gpos_ = gpos;

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box of the box
   wake();               // Waking the box from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the box.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a box contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this changes the union and may cause an invalidation of
 *   links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not allowed
 *   to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::setOrientation( real r, real i, real j, real k )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   q_ = Quat( r, i, j, k );     // Setting the orientation of the box
   R_ = q_.toRotationMatrix();  // Updating the rotation of the box

   Box::calcBoundingBox();  // Updating the axis-aligned bounding box of the box
   wake();             // Waking the box from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the box.
 *
 * \param q The global orientation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a box contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this changes the union and may cause an invalidation of
 *   links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not allowed
 *   to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::setOrientation( const Quat& q )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   q_ = q;                      // Setting the orientation of the box
   R_ = q_.toRotationMatrix();  // Updating the rotation of the box

   Box::calcBoundingBox();  // Updating the axis-aligned bounding box of the box
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
/*!\brief Translation of the center of mass of the box by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 * \exception std::logic_error Invalid translation of a global box inside an exclusive section.
 *
 * \b Note:
 * - Translating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Box::translate( real dx, real dy, real dz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global box inside an exclusive section" );

   gpos_[0] += dx;
   gpos_[1] += dy;
   gpos_[2] += dz;

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the rigid body from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the center of mass of the box by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 * \exception std::logic_error Invalid translation of a global box inside an exclusive section.
 *
 * \b Note:
 * - Translating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Box::translate( const Vec3& dp )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global box inside an exclusive section" );

   gpos_ += dp;

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the box from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the box around the global rotation axis (x,y,z) by the rotation angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * Changing the orientation/rotation of the box. The box is rotated around its center of mass
 * around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotate( real x, real y, real z, real angle )
{
   rotate( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * Changing the orientation/rotation of the box. The box is rotated around its center of mass
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotate( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   q_ = Quat( axis, angle ) * q_;  // Updating the orientation of the box
   R_ = q_.toRotationMatrix();     // Updating the rotation of the box

   Box::calcBoundingBox();  // Updating the axis-aligned bounding box of the box
   wake();             // Waking the box from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * Changing the orientation/rotation of the box. The box is rotated around its center of mass
 * by the Euler angles \a xangle, \a yangle and \a zangle (all in radian measure). The rotations
 * are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotate( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   // Updating the orientation of the box
   q_.rotateX( xangle );  // Rotation around the x-axis
   q_.rotateY( yangle );  // Rotation around the y-axis
   q_.rotateZ( zangle );  // Rotation around the z-axis

   R_ = q_.toRotationMatrix();  // Updating the rotation of the box

   Box::calcBoundingBox();  // Updating the axis-aligned bounding box of the box
   wake();             // Waking the box from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * Changing the orientation/rotation of the box. The box is rotated around its center of mass
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotate( const Vec3& euler )
{
   rotate( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * Changing the orientation/rotation of the box. The box is rotated around its center of mass
 * by the quaternion \a dq. \n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotate( const Quat& dq )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   q_ = dq * q_;                // Updating the orientation of the box
   R_ = q_.toRotationMatrix();  // Updating the rotation of the box

   Box::calcBoundingBox();  // Updating the axis-aligned bounding box of the box
   wake();             // Waking the box from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * This function rotates the box around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the box. The box is rotated
 * around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * This function rotates the box around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the box. The box is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotateAroundOrigin( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   const Quat dq( axis, angle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the box
   q_    = dq * q_;                // Updating the orientation of the box
   R_    = q_.toRotationMatrix();  // Updating the rotation of the box

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box of the box
   wake();               // Waking the box from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * This function rotates the box around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the box. The box is rotated
 * by the Euler angles \a xangle, \a yangle and \a zangle (all components in radian measure).
 * The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the box
   q_    = dq * q_;                // Updating the orientation of the box
   R_    = q_.toRotationMatrix();  // Updating the rotation of the box

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box of the box
   wake();               // Waking the box from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * This function rotates the box around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the box. The box is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are
 * applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * This function rotates the box around the given global coordiante \a point and changes
 * both the global position and the orientation/rotation of the box. The box is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   const Quat dq( axis, angle );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the box
   q_    = dq * q_;                  // Updating the orientation of the box
   R_    = q_.toRotationMatrix();    // Updating the rotation of the box

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box of the box
   wake();               // Waking the box from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the box around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global box inside an exclusive section.
 *
 * This function rotates the box around the given global coordinate \a point and changes
 * both the global position and the orientation/rotation of the box. The box is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are
 * applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a box contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) box on one
 *   process may invalidate the settings of the box on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the box is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Box::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global box inside an exclusive section" );

   const Quat dq( euler );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the box
   q_    = dq * q_;                  // Updating the orientation of the box
   R_    = q_.toRotationMatrix();    // Updating the rotation of the box

   Box::calcBoundingBox();    // Updating the axis-aligned bounding box of the box
   wake();               // Waking the box from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the box.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the box, \a false if not.
 */
bool Box::containsRelPoint( real px, real py, real pz ) const
{
   return std::fabs(px) <= real(0.5)*lengths_[0] &&
          std::fabs(py) <= real(0.5)*lengths_[1] &&
          std::fabs(pz) <= real(0.5)*lengths_[2];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the box.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the box, \a false if not.
 */
bool Box::containsRelPoint( const Vec3& rpos ) const
{
   return std::fabs(rpos[0]) <= real(0.5)*lengths_[0] &&
          std::fabs(rpos[1]) <= real(0.5)*lengths_[1] &&
          std::fabs(rpos[2]) <= real(0.5)*lengths_[2];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the box.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the box, \a false if not.
 */
bool Box::containsPoint( real px, real py, real pz ) const
{
   return containsRelPoint( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the box.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the box, \a false if not.
 */
bool Box::containsPoint( const Vec3& gpos ) const
{
   return containsRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the box.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the surface of the box, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Box::isSurfaceRelPoint( real px, real py, real pz ) const
{
   // Checking if the body relative point lies on one of the x-faces
   if( std::fabs( real(0.5)*lengths_[0] - std::fabs(px) ) <= surfaceThreshold &&
       std::fabs(py) < real(0.5)*lengths_[1] + surfaceThreshold &&
       std::fabs(pz) < real(0.5)*lengths_[2] + surfaceThreshold ) {
      return true;
   }
   // Checking if the body relative point lies on one of the y-faces
   else if( std::fabs( real(0.5)*lengths_[1] - std::fabs(py) ) <= surfaceThreshold &&
            std::fabs(pz) < real(0.5)*lengths_[2] + surfaceThreshold &&
            std::fabs(px) < real(0.5)*lengths_[0] + surfaceThreshold ) {
      return true;
   }
   // Checking if the body relative point lies on one of the z-faces
   else if( std::fabs( real(0.5)*lengths_[2] - std::fabs(pz) ) <= surfaceThreshold &&
            std::fabs(px) < real(0.5)*lengths_[0] + surfaceThreshold &&
            std::fabs(py) < real(0.5)*lengths_[1] + surfaceThreshold ) {
      return true;
   }
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the box.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the surface of the box, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Box::isSurfaceRelPoint( const Vec3& rpos ) const
{
   // Checking if the body relative point lies on one of the x-faces
   if( std::fabs( real(0.5)*lengths_[0] - std::fabs(rpos[0]) ) <= surfaceThreshold &&
       std::fabs(rpos[1]) < real(0.5)*lengths_[1] + surfaceThreshold &&
       std::fabs(rpos[2]) < real(0.5)*lengths_[2] + surfaceThreshold ) {
      return true;
   }
   // Checking if the body relative point lies on one of the y-faces
   else if( std::fabs( real(0.5)*lengths_[1] - std::fabs(rpos[1]) ) <= surfaceThreshold &&
            std::fabs(rpos[2]) < real(0.5)*lengths_[2] + surfaceThreshold &&
            std::fabs(rpos[0]) < real(0.5)*lengths_[0] + surfaceThreshold ) {
      return true;
   }
   // Checking if the body relative point lies on one of the z-faces
   else if( std::fabs( real(0.5)*lengths_[2] - std::fabs(rpos[2]) ) <= surfaceThreshold &&
            std::fabs(rpos[0]) < real(0.5)*lengths_[0] + surfaceThreshold &&
            std::fabs(rpos[1]) < real(0.5)*lengths_[1] + surfaceThreshold ) {
      return true;
   }
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the box.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the surface of the box, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Box::isSurfacePoint( real px, real py, real pz ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the box.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the surface of the box, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Box::isSurfacePoint( const Vec3& gpos ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the box's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the box and a negative value, if the point
 * lies outside the box. The returned depth is calculated relative to the closest side of the box.
 */
real Box::getRelDepth( real px, real py, real pz ) const
{
   const real xdepth( std::fabs(px)-real(0.5)*lengths_[0] );
   const real ydepth( std::fabs(py)-real(0.5)*lengths_[1] );
   const real zdepth( std::fabs(pz)-real(0.5)*lengths_[2] );

   // Calculating the depth for relative points outside the box
   if( xdepth >= real(0) ) {
      if( ydepth >= real(0) ) {
         if( zdepth >= real(0) ) {
            return -std::sqrt( sq(xdepth) + sq(ydepth) + sq(zdepth) );
         }
         else return -std::sqrt( sq(xdepth) + sq(ydepth) );
      }
      else if( zdepth >= real(0) ) {
         return -std::sqrt( sq(xdepth) + sq(zdepth) );
      }
      else return -xdepth;
   }
   else if( ydepth >= real(0) ) {
      if( zdepth >= real(0) ) {
         return -std::sqrt( sq(ydepth) + sq(zdepth) );
      }
      else return -ydepth;
   }
   else if( zdepth >= real(0) ) {
      return -zdepth;
   }

   // Relative point is inside the box => calculating the
   // depth depending on the distance to the nearest face
   else return -max( xdepth, ydepth, zdepth );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the box's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the box and a negative value, if the point
 * lies outside the box. The returned depth is calculated relative to the closest side of the box.
 */
real Box::getRelDepth( const Vec3& rpos ) const
{
   const real xdepth( std::fabs(rpos[0])-real(0.5)*lengths_[0] );
   const real ydepth( std::fabs(rpos[1])-real(0.5)*lengths_[1] );
   const real zdepth( std::fabs(rpos[2])-real(0.5)*lengths_[2] );

   // Calculating the depth for relative points outside the box
   if( xdepth >= real(0) ) {
      if( ydepth >= real(0) ) {
         if( zdepth >= real(0) ) {
            return -std::sqrt( sq(xdepth) + sq(ydepth) + sq(zdepth) );
         }
         else return -std::sqrt( sq(xdepth) + sq(ydepth) );
      }
      else if( zdepth >= real(0) ) {
         return -std::sqrt( sq(xdepth) + sq(zdepth) );
      }
      else return -xdepth;
   }
   else if( ydepth >= real(0) ) {
      if( zdepth >= real(0) ) {
         return -std::sqrt( sq(ydepth) + sq(zdepth) );
      }
      else return -ydepth;
   }
   else if( zdepth >= real(0) ) {
      return -zdepth;
   }

   // Relative point is inside the box => calculating the
   // depth depending on the distance to the nearest face
   else return -max( xdepth, ydepth, zdepth );
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate box.
 *
 * \param dp Change in the global position of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. This movement involves a change in the global position and the axis-aligned
 * bounding box.
 */
void Box::update( const Vec3& dp )
{
   // Checking the state of the box
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid box state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Updating the global position
   gpos_ += dp;

   // Setting the axis-aligned bounding box
   Box::calcBoundingBox();

   // Checking the state of the box
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid box state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate box.
 *
 * \param dq Change in the orientation of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational movement.
 * This movement involves a change in the global position, the orientation/rotation and the
 * axis-aligned bounding box of the box.
 */
void Box::update( const Quat& dq )
{
   // Checking the state of the box
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid box state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Calculating the new global position
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Setting the axis-aligned bounding box
   Box::calcBoundingBox();

   // Checking the state of the box
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid box state detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a box.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the box output.
 * \return void
 */
void Box::print( std::ostream& os, const char* tab ) const
{
   using std::setw;

   os << tab << " Box " << uid_ << " with side lengths " << lengths_ << "\n";

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
//  BOX SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new box.
 * \ingroup box
 *
 * \param uid The user-specific ID of the box.
 * \param gpos The global position of the center of the box.
 * \param lengths The side length of the box \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 * \return Handle for the new box.
 * \exception std::invalid_argument Invalid side length.
 * \exception std::invalid_argument Invalid global box position.
 *
 * This function creates a box primitive in the \b pe simulation system. The new box with the
 * user-specific ID \a uid is placed at the global position \a gpos, has the side lengths
 * \a lengths, and consists of the material \a material. The \a visible flag sets the box
 * (in-)visible in all visualizations.
 *
 * \image html box.png
 * \image latex box.eps "Box geometry" width=200pt
 *
 * The following code example illustrates the setup of a box:

   \code
   // Creating the iron box 1 with the side lengths (3,2,1) at the global position (2,3,4).
   // Per default the box is visible in all visualizations. Note that the box is automatically
   // added to the simulation world and is immediately part of the entire simulation. The
   // function returns a handle to the newly created box, which can be used to for instance
   // rotate the box around the global y-axis.
   BoxID box = createBox( 1, Vec3( 2.0, 3.0, 4.0 ), Vec3( 3.0, 2.0, 1.0 ), iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the box is created inside a pe::pe_CREATE_UNION section, the box is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron box 2 with side lengths (2.1,1.3,4.5) at the global position
      // (-1,4,-5). Since the box is created inside a pe_CREATE_UNION section, the box
      // is directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      createBox( 2, Vec3( -1.0, 4.0, -5.0 ), Vec3( 2.1, 1.3, 4.5 ), iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, boxes may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that the final center of
 * mass of the resulting union must be inside the domain of the local process.
 */
PE_PUBLIC BoxID createBox( id_t uid, const Vec3& gpos, const Vec3& lengths,
                 MaterialID material, bool visible )
{
   const bool global( GlobalSection::isActive() );

   // Checking the side lengths
   if( lengths[0] <= real(0) || lengths[1] <= real(0) || lengths[2] <= real(0) )
      throw std::invalid_argument( "Invalid side length" );

   // Checking the global position of the box
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global box position" );

   // Creating the new box
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   BoxID box = new Box( sid, uid, gpos, lengths, material, visible );

   // Checking if the box is created inside a global section
   if( global )
      box->setGlobal();

   // Checking if the box has to be permanently fixed
   else if( box->isAlwaysFixed() )
      box->setFixed( true );

   // Registering the new box with the default body manager
   try {
      theDefaultManager()->add( box );
   }
   catch( std::exception& ) {
      delete box;
      throw;
   }

   // Logging the successful creation of the box
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created box " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Side lengths    = " << lengths << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return box;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote box.
 * \ingroup box
 *
 * \param sid The unique system-specific ID of the box.
 * \param uid The user-specific ID of the box.
 * \param gpos The global position of the center of the box.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the box's body frame in the global world frame.
 * \param lengths The side length of the box \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 * \param fixed \a true to fix the box, \a false to unfix it.
 * \param reg \a true to register the object in the default body manager.
 * \return Handle for the new box.
 *
 * This function instantiates a copy of a box with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of a box residing on a remote MPI
 * process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
BoxID instantiateBox( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                      const Vec3& lengths, MaterialID material, bool visible, bool fixed, bool reg )
{
   // Checking the side lengths
   pe_INTERNAL_ASSERT( lengths[0] > real(0), "Invalid side length in x-dimension" );
   pe_INTERNAL_ASSERT( lengths[1] > real(0), "Invalid side length in y-dimension" );
   pe_INTERNAL_ASSERT( lengths[2] > real(0), "Invalid side length in z-dimension" );

   // Instantiating the box
   BoxID box = new Box( sid, uid, gpos, rpos, q, lengths, material, visible, fixed );

   // Registering the box with the default body manager
   if( reg ) {
      try {
         theDefaultManager()->add( box );
      }
      catch( std::exception& ) {
         delete box;
         throw;
      }
   }

   // Logging the successful instantiation of the box
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated box " << sid << "\n"
          << "   User-ID         = " << uid << "\n"
          << "   Global position = " << gpos << "\n"
          << "   Side lengths    = " << lengths << "\n"
          << "   Material        = " << Material::getName( material );
   }

   return box;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for boxes.
 * \ingroup box
 *
 * \param os Reference to the output stream.
 * \param b Reference to a constant box object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Box& b )
{
   os << "--" << pe_BROWN << "BOX PARAMETERS" << pe_OLDCOLOR
      << "----------------------------------------------------------------\n";
   b.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for box handles.
 * \ingroup box
 *
 * \param os Reference to the output stream.
 * \param b Constant box handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstBoxID b )
{
   os << "--" << pe_BROWN << "BOX PARAMETERS" << pe_OLDCOLOR
      << "----------------------------------------------------------------\n";
   b->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
