//=================================================================================================
/*!
 *  \file src/core/rigidbody/Union.cpp
 *  \brief Source file for the Union class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
 *                2013-2014 Tobias Scharpff
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
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <pe/core/CollisionSystem.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/SphereVector.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/math/Constants.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/shims/Square.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DetailSection.h>
#include <pe/util/Random.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/povray.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Union class.
 *
 * \param sid Unique system-specific ID for the union.
 * \param uid User-specific ID for the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 */
Union::Union( id_t sid, id_t uid, bool visible )
   : Parent( sid, uid, visible )  // Initialization of the parent class
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiation constructor for the Union class.
 *
 * \param sid Unique system-specific ID for the union.
 * \param uid User-specific ID for the union.
 * \param gpos Global center of mass of the union.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param mass The total mass of the union.
 * \param I The moment of inertia in reference to the union's own body frame.
 * \param q The orientation of the union's body frame in the global world frame.
 * \param aabb Coordinates of the axis-aligned bounding box of the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \param fixed \a true to fix the union, \a false to unfix it.
 */
Union::Union( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, real mass,
              const Mat3& I, const Quat& q, const Vec6& aabb, bool visible, bool fixed )
   : Parent( sid, uid, visible )  // Initialization of the parent class
{
   // Checking whether the constructor is used inside a pe_INSTANTIATE_UNION section
   pe_INTERNAL_ASSERT( InstantiateUnion::isActive(), "Invalid constructor call detected" );

   // Initializing the instantiated union
   remote_ = true;                   // Setting the remote flag
   gpos_   = gpos;                   // Setting the global center of mass
   rpos_   = rpos;                   // Setting the relative position
   mass_   = mass;                   // Setting the total mass
   I_      = I;                      // Setting the moment of inertia
   q_      = q;                      // Setting the orientation
   R_      = q_.toRotationMatrix();  // Setting the rotation matrix

   aabb_[0] = aabb[0];
   aabb_[1] = aabb[1];
   aabb_[2] = aabb[2];
   aabb_[3] = aabb[3];
   aabb_[4] = aabb[4];
   aabb_[5] = aabb[5];

   if( fixed ) {
      fixed_   = true;               // Setting the fixed flag
      invMass_ = real(0);            // Setting the inverse total mass
      Iinv_    = real(0);            // Setting the inverse moment of inertia
   }
   else {
      invMass_ = real(1) / mass_;    // Setting the inverse total mass
      Iinv_    = I_.getInverse();    // Setting the inverse moment of inertia
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Union class.
 *
 * Destroying a Union also destroys any contained rigid body and link.
 */
Union::~Union()
{
   // Logging the destruction of the union
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Destroyed union " << sid_;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the entire union visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the union visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the entire union including all contained rigid bodies visible/invisible
 * in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union on one
 * process may invalidate the settings of the union on another process. In order to synchronize
 * all rigid bodies after local changes, the World::synchronize() function should be used to
 * update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 * neglected and overwritten by the settings of the rigid body on its local process!
 */
void Union::setVisible( bool visible )
{
   visible_ = visible;

   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      b->setVisible( visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the union.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global union inside an exclusive section.
 *
 * Setting the global position of the union's center of mass. If the union contains an infinite
 * rigid body, the function shifts the union to reposition its anchor point according to the
 * given global coordinate.
 *
 * \b Note:
 * - Setting the position of an union contained in another union changes the mass distribution
 *   and geometry of the other union. Therefore this may cause an invalidation of links contained
 *   in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::setPosition( real px, real py, real pz )
{
   setPosition( Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the union.
 *
 * \param gpos The global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global union inside an exclusive section.
 *
 * Setting the global position of the union's center of mass. If the union contains an infinite
 * rigid body, the function shifts the union to reposition its anchor point according to the
 * given global coordinate.
 *
 * \b Note:
 * - Setting the position of an union contained in another union changes the mass distribution
 *   and geometry of the other union. Therefore this may cause an invalidation of links contained
 *   in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within an pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::setPosition( const Vec3& gpos )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global union inside an exclusive section" );

   // Calculating the position change
   const Vec3 dp( gpos - gpos_ );

   // Setting the global position
   gpos_ = gpos;

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dp );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dp );

   Union::calcBoundingBox();    // Setting the axis-aligned bounding box
   wake();               // Waking the union from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the union.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * Setting the orientation/rotation of the entire union. This function causes all contained
 * primitives to rotate around the center of mass of the union (if the union is finite) or around
 * the anchor point of the union (if the union is infinite). The orientation of the rigid bodies
 * within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Setting the orientation of an union contained in another union changes the mass distribution
 *   and geometry of the other union. Therefore this changes the other union and may cause an
 *   invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not
 *   allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this
 *   results in a \a std::logic_error.
 */
void Union::setOrientation( real r, real i, real j, real k )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat q ( r, i, j, k );
   const Quat dq( q * q_.getInverse() );

   q_ = q;
   R_ = q_.toRotationMatrix();

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();  // Setting the axis-aligned bounding box
   wake();             // Waking the union from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the union.
 *
 * \param q The global orientation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * Setting the orientation/rotation of the entire union. This function causes all contained
 * primitives to rotate around the center of mass of the union (if the union is finite) or around
 * the anchor point of the union (if the union is infinite). The orientation of the rigid bodies
 * within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Setting the orientation of an union contained in another union changes the mass distribution
 *   and geometry of the other union. Therefore this changes the other union and may cause an
 *   invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not
 *   allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this
 *   results in a \a std::logic_error.
 */
void Union::setOrientation( const Quat& q )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat dq( q * q_.getInverse() );

   q_ = q;
   R_ = q_.toRotationMatrix();

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();  // Setting the axis-aligned bounding box
   wake();             // Waking the union from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transforming the union into an infinite rigid body.
 *
 * \return void
 *
 * In order to transform the union into an infinite rigid body, the linear and angular velocity,
 * the mass and moment of inertia of the union are adjusted to resemble an infinite rigid body.
 * The global position remains unchanged. However, it no longer represents the center of mass,
 * but a reference point for the contained rigid bodies.
 */
void Union::setInfinite()
{
   // Setting the finite flag to infinite
   finite_ = false;

   // Fixing the global position of the infinite union
   fixed_ = true;

   // Setting the linear and angular velocity to zero
   v_ = real(0);
   w_ = real(0);

   // Adjusting the total mass
   mass_    = real(0);
   invMass_ = real(0);

   // Adjusting the moment of inertia
   I_    = real(0);
   Iinv_ = real(0);
}
//*************************************************************************************************




//=================================================================================================
//
//  FIXATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the global position (the center of mass) of the union fixed.
 *
 * \return void
 *
 * This function fixes the global position (the center of mass) of the union. If the union
 * is contained in another superordinate body, fixing the contained union will also fix the
 * global position of the superordinate body. In case the union contains an infinite body
 * (as for instance a plane or an union containing a plane) the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
void Union::fix()
{
   fixed_ = true;

   // Adjusting the inverse mass and inverse moment of inertia
   invMass_ = real(0);
   Iinv_    = real(0);

   // Setting the linear and angular velocity to zero
   v_ = real(0);
   w_ = real(0);

   // Signaling the fixation change to the superordinate body
   signalFixation();

   // Setting the fixed flag of the contained rigid bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      b->setFixed( true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position (the center of mass) of the union fixed.
 *
 * \return void
 *
 * This function unfixes the global position (the center of mass) of the union. If the union
 * is contained in another superordinate body, unfixing the contained union will also unfix
 * the global position of the superordinate body if it only contains this union. In case the
 * union contains an infinite body (as for instance a plane or an union containing a plane)
 * the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
void Union::unfix()
{
   fixed_ = false;

   // Adjusting the inverse mass and inverse moment of inertia
   invMass_ = real(1) / mass_;
   Iinv_    = I_.getInverse();

   // Signaling the fixation change to the superordinate body
   signalFixation();

   // Setting the fixed flag of the contained rigid bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      b->setFixed( false );
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the center of mass of the union by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 * \exception std::logic_error Invalid translation of a global union inside an exclusive section.
 *
 * Changing the global position of the entire union's center of mass. All contained rigid bodies
 * are moved by the same displacement.
 *
 * \b Note:
 * - Translating an union contained in another union changes the mass distribution and geometry
 *   of the other union. Therefore this may cause an invalidation of links contained in the other
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Union::translate( real dx, real dy, real dz )
{
   Union::translate( Vec3( dx, dy, dz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the center of mass of the union by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 * \exception std::logic_error Invalid translation of a global union inside an exclusive section.
 *
 * Changing the global position of the entire union's center of mass. All contained rigid bodies
 * are moved by the same displacement.
 *
 * \b Note:
 * - Translating an union contained in another union changes the mass distribution and geometry
 *   of the other union. Therefore this may cause an invalidation of links contained in the other
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void Union::translate( const Vec3& dp )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global union inside an exclusive section" );

   // Changing the global position/reference point
   gpos_ += dp;

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dp );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dp );

   Union::calcBoundingBox();    // Setting the axis-aligned bounding box
   wake();               // Waking the union from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the union around the global rotation axis (x,y,z) by the rotation
 * \brief angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * Changing the orientation/rotation of the entire union. This function causes all contained
 * rigid bodies to rotate around the center of mass of the union (if the union is finite) or
 * around the anchor point of the union (if the union is infinite). The orientation of the
 * bodies within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotate( real x, real y, real z, real angle )
{
   rotate( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * Changing the orientation/rotation of the entire union. This function causes all contained
 * rigid bodies to rotate around the center of mass of the union (if the union is finite) or
 * around the anchor point of the union (if the union is infinite). The orientation of the
 * bodies within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotate( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat dq( axis, angle );

   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();  // Setting the axis-aligned bounding box
   wake();             // Waking the union from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * Changing the orientation/rotation of the entire union. This function causes all contained
 * rigid bodies to rotate around the center of mass of the union (if the union is finite) or
 * around the anchor point of the union (if the union is infinite). The rotations are applied
 * in the order x, y, z. The orientation of the bodies within the union in reference to the
 * body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotate( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   q_ = dq * q_;                // Updating the orientation of the union
   R_ = q_.toRotationMatrix();  // Updating the rotation of the union

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();  // Setting the axis-aligned bounding box
   wake();             // Waking the union from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * Changing the orientation/rotation of the entire union. This function causes all contained
 * rigid bodies to rotate around the center of mass of the union (if the union is finite) or
 * around the anchor point of the union (if the union is infinite). The rotations are applied
 * in the order x, y, z. The orientation of the bodies within the union in reference to the
 * body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotate( const Vec3& euler )
{
   rotate( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * Changing the orientation/rotation of the entire union. This function causes all contained
 * rigid bodies to rotate around the center of mass of the union (if the union is finite) or
 * around the anchor point of the union (if the union is infinite). The orientation of the
 * bodies within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotate( const Quat& dq )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   q_ = dq * q_;                // Updating the orientation of the union
   R_ = q_.toRotationMatrix();  // Updating the rotation of the union

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();  // Setting the axis-aligned bounding box
   wake();             // Waking the union from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * This function rotates the entire union around the origin of the global world frame and
 * changes both the global position and the orientation/rotation of the union. Additionally,
 * all contained rigid bodies change their position and orientation accordingly. The orientation
 * of the bodies within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * This function rotates the entire union around the origin of the global world frame and
 * changes both the global position and the orientation/rotation of the union. Additionally,
 * all contained rigid bodies change their position and orientation accordingly. The orientation
 * of the bodies within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotateAroundOrigin( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat dq( axis, angle );

   q_    = dq * q_;                // Updating the orientation of the union
   R_    = q_.toRotationMatrix();  // Updating the rotation of the union
   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the union

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();    // Setting the axis-aligned bounding box
   wake();               // Waking the union from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * This function rotates the entire union around the origin of the global world frame and
 * changes both the global position and the orientation/rotation of the union. Additionally,
 * all contained rigid bodies change their position and orientation accordingly. The rotations
 * are applied in the order x, y, z. The orientation of the bodies within the union in
 * reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   q_    = dq * q_;                // Updating the orientation of the union
   R_    = q_.toRotationMatrix();  // Updating the rotation of the union
   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the union

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();    // Setting the axis-aligned bounding box
   wake();               // Waking the union from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * This function rotates the entire union around the origin of the global world frame and
 * changes both the global position and the orientation/rotation of the union. Additionally,
 * all contained rigid bodies change their position and orientation accordingly. The rotations
 * are applied in the order x, y, z. The orientation of the bodies within the union in
 * reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * This function rotates the entire union around the given global coordiante \a point and
 * changes both the global position and the orientation/rotation of the union. Additionally,
 * all contained rigid bodies change their position and orientation accordingly. The orientation
 * of the bodies within the union in reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat dq( axis, angle );
   const Vec3 dp( gpos_ - point );

   q_    = dq * q_;                  // Updating the orientation of the union
   R_    = q_.toRotationMatrix();    // Updating the rotation of the union
   gpos_ = point + dq.rotate( dp );  // Updating the global position of the union

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();    // Setting the axis-aligned bounding box
   wake();               // Waking the union from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the union around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global union inside an exclusive section.
 *
 * This function rotates the entire union around the given global coordiante \a point and
 * changes both the global position and the orientation/rotation of the union. Additionally,
 * all contained rigid bodies change their position and orientation accordingly. The rotations
 * are appliced in the order x, y, z. The orientation of the bodies within the union in
 * reference to the body frame of the union is not changed.
 *
 * \b Note:
 * - Rotating an union contained in another union changes the mass distribution and geometry of
 *   the other union. Therefore this changes the union and may cause an invalidation of links
 *   contained in the other union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) union
 *   on one process may invalidate the settings of the union on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the union is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void Union::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global union inside an exclusive section" );

   const Quat dq( euler );
   const Vec3 dp( gpos_ - point );

   q_    = dq * q_;                  // Updating the orientation of the union
   R_    = q_.toRotationMatrix();    // Updating the rotation of the union
   gpos_ = point + dq.rotate( dp );  // Updating the global position of the union

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   Union::calcBoundingBox();    // Setting the axis-aligned bounding box
   wake();               // Waking the union from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the union.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the union, \a false if not.
 */
bool Union::containsRelPoint( real px, real py, real pz ) const
{
   const Vec3 gpos( pointFromBFtoWF( px, py, pz ) );
   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( gpos ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the union.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the union, \a false if not.
 */
bool Union::containsRelPoint( const Vec3& rpos ) const
{
   const Vec3 gpos( pointFromBFtoWF( rpos ) );
   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( gpos ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the union.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the union, \a false if not.
 */
bool Union::containsPoint( real px, real py, real pz ) const
{
   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( px, py, pz ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the union.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the union, \a false if not.
 */
bool Union::containsPoint( const Vec3& gpos ) const
{
   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( gpos ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the union.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the surface of the union, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Union::isSurfaceRelPoint( real px, real py, real pz ) const
{
   bool surface( false );
   const Vec3 gpos( pointFromBFtoWF( px, py, pz ) );

   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( gpos ) ) return false;
      else if( b->isSurfacePoint( gpos ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the union.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the surface of the union, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Union::isSurfaceRelPoint( const Vec3& rpos ) const
{
   bool surface( false );
   const Vec3 gpos( pointFromBFtoWF( rpos ) );

   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( gpos ) ) return false;
      else if( b->isSurfacePoint( gpos ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the union.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the surface of the union, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Union::isSurfacePoint( real px, real py, real pz ) const
{
   bool surface( false );

   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( px, py, pz ) ) return false;
      else if( b->isSurfacePoint( px, py, pz ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the union.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the surface of the union, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Union::isSurfacePoint( const Vec3& gpos ) const
{
   bool surface( false );

   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( gpos ) ) return false;
      else if( b->isSurfacePoint( gpos ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTROY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destroying a rigid body contained in the union.
 *
 * \param pos Iterator to the rigid body to be destroyed.
 * \return Iterator to the body after the destroyed body.
 *
 * This function destroys a rigid body contained in the union and updates all union properties
 * that change due to the destroyed rigid body: the center of mass, the relative position of
 * all bodies, the attached sections of all contained link, the moment of intertia and the
 * axis-aligned bounding box. Note that destroying the rigid body invalidates all remaining
 * references/IDs to the body!
 */
Union::Iterator Union::destroy( Iterator pos )
{
   // Checking the validity of the iterator
   pe_INTERNAL_ASSERT( pos->getManager() == ManagerID( this ), "Invalid body iterator" );

   // Destroying and deregistering the rigid body
   destroyBody( *pos );
   pos = bodies_.erase( pos );

   // Setting the finite flag
   finite_ = true;
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( !b->isFinite() ) {
         finite_ = false;
         break;
      }
   }

   // Setting the union's total mass and center of mass
   calcCenterOfMass();

   // Setting the contained primitives' relative position in reference to the center of mass
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateRelPosition( *b );

   // Updating the sections of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding box
   Union::calcBoundingBox();

   // Signaling the internal modification to the superordinate body
   signalModification();

   return pos;
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate union.
 *
 * \param dp Change in the global position of the superordinate union.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. This movement involves a change in the global position and the axis-aligned
 * bounding box.
 */
void Union::update( const Vec3& dp )
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Updating the global position
   gpos_ += dp;

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dp );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dp );

   // Setting the axis-aligned bounding box
   Union::calcBoundingBox();

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate union.
 *
 * \param dq Change in the orientation of the superordinate union.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational movement.
 * This movement involves a change in the global position, the orientation/rotation and the
 * axis-aligned bounding box of the box.
 */
void Union::update( const Quat& dq )
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Calculating the new global position
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Updating the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateBody( *b, dq );

   // Updating the links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->update( dq );

   // Setting the axis-aligned bounding box
   Union::calcBoundingBox();

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  RIGID BODY MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a rigid body to the union.
 *
 * \param body The rigid body to be added to the union.
 * \return void
 * \exception std::logic_error Invalid adding into a global union inside an exclusive section.
 * \exception std::logic_error Global flags of body and union do not match.
 *
 * This function adds another rigid body to the union (as for instance a sphere, a box, a
 * capsule, or another union). It updates all union properties that change due to the new
 * rigid body: the center of mass, the relative position of all contained rigid bodies, the
 * attached sections of all contained links, the moment of inertia, and the axis-aligned
 * bounding box.\n
 * The union takes full responsibility for the newly added body, including the necessary
 * memory management. After adding the body to the union, the body is considered part of the
 * union. All functions called on the union (as for instance all kinds of set functions,
 * translation or rotation functions) additionally affect the rigid body. However, the body
 * can still individually be positioned, oriented, translated, rotated, or made (in-)visible
 * within the union.\n\n
 *
 *
 * \section union_add_infinite Adding infinite rigid bodies
 *
 * Adding an infinite rigid body (as for instance a plane) to the union makes the union an
 * infinite rigid body. This additionally resets the linear and angular velocity of the union
 * and fixes its global position. Note that removing the last infinite body from an union will
 * not restore previous settings such as the velocities and will not unfix the union!\n\n
 *
 *
 * \section union_add_global Global bodies/unions
 *
 * Adding a global rigid body (i.e. a body defined inside the pe_GLOBAL_SECTION) to the union
 * requires the union to be also global. Adding a non-global rigid body to a union requires
 * the union to be also non-global. The attempt to add a global rigid body to a non-global
 * union or a non-global body to a global union results in a \a std::logic_error exception.
 *
 *
 * \section union_add_rules Additional rules
 *
 * The following rules apply for the mobility and visibility of the resulting union:
 *  - If either the union or the added rigid body is fixed, the new compound will also be fixed.
 *    For instance, adding a fixed rigid body to an union will fix the union, and adding a rigid
 *    body to a fixed union will fix the body.
 *  - Neither the (in-)visibility of the added rigid body nor (in-)visibility of the union will
 *    change due to the add operation. For instance adding a visible rigid body to an invisible
 *    union will not change the visibility of the body. Neither is the visibility of the union
 *    changed. In order to change the visiblity, the setVisible() function can be either called
 *    individually for the rigid body (to exclusively make the body (in-)visible) or the entire
 *    union (to make the entire union (in-)visible.
 */
void Union::add( BodyID body )
{
   // Checking for "self-assignment"
   if( body == BodyID( this ) ) return;

   // Checking the body manager
   if( body->hasManager() && body->getManager() == ManagerID( this ) ) return;

   // Checking whether a body is added to a global union inside an exclusive section
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid adding into a global union inside an exclusive section" );

   // Checking the global flags of the body and the union in MPI parallel simulations
   if( body->isGlobal() ^ global_ )
      throw std::logic_error( "Global flags of body and union do not match" );

   // Deregistering the rigid body from the old body manager
   if( body->hasManager() ) {
      ManagerID manager = body->getManager();
      manager->remove( body );
   }

   // Registering the union as new body manager
   setManager( body );

   // Registering the rigid body
   setSuperBody( body );
   registerSuperBody( body );
   bodies_.pushBack( body );

   // Updating the axis-aligned bounding box
   if( bodies_.size() == 1 )
      aabb_ = body->getAABB();
   else
      aabb_ += body->getAABB();

   // Setting the finite and fixed flag of the union
   if( !body->isFinite() ) setInfinite();
   else if( !global_ && body->isFixed() ) setFixed( true );

   // Skipping the rest of the initializations in case the union is instantiated
   // within a pe_INSTANTIATE_UNION section
   if( InstantiateUnion::isActive() ) return;

   // Setting the fixed flag of the rigid body
   if( fixed_ && !body->isFixed() ) body->setFixed( true );

   // Setting the union's total mass and center of mass
   calcCenterOfMass();

   // Setting the contained primitives' relative position in reference to the center of mass
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateRelPosition( *b );

   // Updating the sections of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   // Signaling the internal modification to the superordinate body
   signalModification();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the union.
 *
 * \param body The rigid body to be removed.
 * \return void
 *
 * This function is a requirement for all rigid body manager instances and removes/deregisters
 * a rigid body from the union.
 * The function removes \a body from the union and updates all union properties that change
 * due to the removed rigid body: the center of mass, the relative position of all bodies, the
 * attached sections of all contained links, the moment of inertia and the axis-aligned bounding
 * box.
 *
 * \b Note: This function doesn't have to be called explicitly. It is automatically called in
 * case the body manager is changed or if the rigid body is destroyed.
 */
void Union::remove( BodyID body )
{
   pe_INTERNAL_ASSERT( body->getManager() == this, "Rigid body has wrong body manager" );

   // Checking if the rigid body is contained in the union
   const Iterator pos( std::find( bodies_.begin(), bodies_.end(), body ) );
   pe_INTERNAL_ASSERT( pos != bodies_.end(), "Rigid body is not contained in the union" );

   // Deregistering the union as new body manager
   resetManager( body );

   // Deregistering the rigid body
   resetSuperBody( body );
   deregisterSuperBody( body );
   bodies_.erase( pos );
   pe_INTERNAL_ASSERT( !body->hasSuperBody(), "Invalid superbody detected" );

   // Resetting the body's relative position
   updateRelPosition( body );

   // Setting the body's linear and angular velocities
   if( !fixed_ && !body->isFixed() ) {
      body->setLinearVel ( velFromWF( body->getPosition() ) );
      body->setAngularVel( getAngularVel() );
   }

   // Setting the finite flag
   finite_ = true;
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( !b->isFinite() ) {
         finite_ = false;
         break;
      }
   }

   // Setting the union's total mass and center of mass
   calcCenterOfMass();

   // Setting the contained primitives' relative position in reference to the center of mass
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateRelPosition( *b );

   // Updating the sections of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding box
   Union::calcBoundingBox();

   // Signaling the internal modification to the superordinate body
   signalModification();
}
//*************************************************************************************************




//=================================================================================================
//
//  LINK SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new link within the union.
 * \ingroup link
 *
 * \param id The user-specific ID of the link.
 * \param b1 The first directly attached rigid body.
 * \param b2 The second directly attached rigid body.
 * \return Handle for the new link.
 * \exception std::logic_error Cannot create link in MPI parallel simulation.
 * \exception std::invalid_argument Invalid link parameters.
 *
 * Creating a new link between the two rigid bodies \a b1 and \a b2 contained in the union.
 * Both bodies have to touch each other or the setup of the link fails, which will result
 * in a \a std::invalid_argument exception. Additionally it will be checked if previously
 * an identical link has been defined, which will also result in a \a std::invalid_argument
 * exception. If the structure of the union can be uniquely separated into two distinct,
 * non-disjoint, and non-cyclic sections (where the first body will be in section 1 and
 * the second body in section 2), then in each time step the forces and torques in the
 * link will be calculated.
 *
 * \b Note: Currently it is not possible to create links in MPI parallel simulations (i.e.,
 * in case the simulation uses more than a single process). The attempt to create a link in
 * a parallel simulation results results in a \a std::logic_error exception.
 */
LinkID Union::createLink( id_t id, BodyID b1, BodyID b2 )
{
   // Checking the number of active MPI processes
   if( MPISettings::size() > 1 )
      throw std::logic_error( "Cannot create link in MPI parallel simulation" );

   // Checking the rigid body parameters
   if( b1->getSuperBody() != this || b2->getSuperBody() != this )
      throw std::invalid_argument( "Invalid rigid body IDs!" );

   // Checking for a previously defined identical link
   for( ConstLinkIterator link=links_.begin(); link!=links_.end(); ++link ) {
      if( ( link->getBody1() == b1 && link->getBody2() == b2 ) ||
          ( link->getBody1() == b2 && link->getBody2() == b1 ) ) {
         throw std::invalid_argument( "Duplicate link definition!" );
      }
   }

   // Creating a new link between the given bodies
   LinkID link = new Link( id, this, b1, b2 );
   link->setupLink( bodies_ );
   links_.pushBack( link );
   return link;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destroys the given link.
 * \ingroup link
 *
 * \param link The link to be destroyed.
 *
 * \b Note: Destroying the link invalidates all remaining LinkID referencing the link!
 */
void Union::destroyLink( LinkID link )
{
   const LinkIterator pos( std::find( links_.begin(), links_.end(), link ) );
   pe_INTERNAL_ASSERT( pos != links_.end(), "Link is not contained in the union" );
   links_.erase( pos );

   delete link;
}
//*************************************************************************************************




//=================================================================================================
//
//  FUNCTIONS FOR INTERNAL CHANGES IN COMPOUND GEOMETRIES
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Signals an internal modification of a contained subordiante rigid body.
 *
 * \return void
 *
 * In case one of the contained rigid bodies is interally modified, this function is called to
 * recalculate the changed properties of the union.
 */
void Union::handleModification()
{
   // Setting the finite flag
   finite_ = true;
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( !b->isFinite() ) {
         finite_ = false;
         break;
      }
   }

   // Setting the finiteness of the union
   if( !finite_ ) setInfinite();

   // Setting the union's total mass and center of mass
   calcCenterOfMass();

   // Setting the contained primitives' relative position in reference to the center of mass
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateRelPosition( *b );

   // Updating the sections of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   // Updating the axis-aligned bounding box
   Union::calcBoundingBox();

   // Signaling the internal modification to the superordinate body
   signalModification();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signals a position change of a contained subordiante rigid body.
 *
 * \return void
 *
 * In case one of the contained rigid bodies changes its position, this function is called
 * to recalculate the changed properties of the union.
 */
void Union::handleTranslation()
{
   // Setting the union's total mass and center of mass
   calcCenterOfMass();

   // Setting the contained bodies' relative position in reference to the center of mass
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateRelPosition( *b );

   // Updating the sections of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   Union::calcBoundingBox();    // Setting the axis-aligned bounding box
   wake();               // Waking the union from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signals an orientation change of a contained subordiante rigid body.
 *
 * \return void
 *
 * In case one of the contained rigid bodies changes its orientation, this function is called
 * to recalculate the changed properties of the union.
 */
void Union::handleRotation()
{
   // Updating the sections of the contained links
   for( LinkIterator link=links_.begin(); link!=links_.end(); ++link )
      link->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   Union::calcBoundingBox();  // Setting the axis-aligned bounding box
   wake();             // Waking the union from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signals a fixation change of a contained subordiante rigid body.
 *
 * \return void
 *
 * In case one of the contained rigid bodies changes its fixation flag, this function is called
 * to re-evaluate the fixation of the union.
 */
void Union::handleFixation()
{
   // Checking the global flag of the union
   pe_INTERNAL_ASSERT( !global_, "Invalid fixation change within global union" );

   // Early exit in case the flag is already set accordingly, the body is infinite or global
   if( !finite_ )
      return;

   // Estimating the fixed flag
   bool fixed( false );
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->isFixed() ) {
         fixed = true;
         break;
      }
   }

   // Handling a change in the fixation
   if( fixed_ != fixed )
   {
      fixed_ = fixed;

      if( fixed ) {
         // Adjusting the inverse mass and inverse moment of inertia
         invMass_ = real(0);
         Iinv_    = real(0);

#if !MOBILE_INFINITE
         // Setting the linear and angular velocity to zero
         v_ = real(0);
         w_ = real(0);
#endif
      }
      else {
         // Adjusting the inverse mass and inverse moment of inertia
         invMass_ = real(1) / mass_;
         Iinv_    = I_.getInverse();
      }

      // Signaling the fixation change to the superordinate body
      signalFixation();
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of an union.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the union output.
 * \return void
 */
void Union::print( std::ostream& os, const char* tab ) const
{
   using std::setw;

   os << tab << " Union " << uid_ << " with " << bodies_.size();
   if( bodies_.size() == 1 ) os << " rigid body\n";
   else os << " rigid bodies\n";

   if( verboseMode ) {
      os << tab << "   Fixed: " << fixed_ << " , sleeping: " << !awake_ << "\n";
   }

   os << tab << "   System ID         = " << sid_ << "\n"
      << tab << "   Total mass        = ";
   if( finite_ ) os << mass_ << "\n";
   else os << "*infinite*\n";

   os << tab << "   Global position   = " << gpos_ << "\n"
      << tab << "   Relative position = " << rpos_ << "\n"
      << tab << "   Linear velocity   = " << v_ << "\n"
      << tab << "   Angular velocity  = " << w_ << "\n";

   if( verboseMode )
   {
      os << tab << "   Bounding box      = " << aabb_ << "\n"
         << tab << "   Quaternion        = " << q_ << "\n"
         << tab << "   Rotation matrix   = ( " << setw(9) << R_[0] << " , " << setw(9) << R_[1] << " , " << setw(9) << R_[2] << " )\n"
         << tab << "                       ( " << setw(9) << R_[3] << " , " << setw(9) << R_[4] << " , " << setw(9) << R_[5] << " )\n"
         << tab << "                       ( " << setw(9) << R_[6] << " , " << setw(9) << R_[7] << " , " << setw(9) << R_[8] << " )\n";

      if( finite_ ) {
         os << std::setiosflags(std::ios::right)
            << tab << "   Moment of inertia = ( " << setw(9) << I_[0] << " , " << setw(9) << I_[1] << " , " << setw(9) << I_[2] << " )\n"
            << tab << "                       ( " << setw(9) << I_[3] << " , " << setw(9) << I_[4] << " , " << setw(9) << I_[5] << " )\n"
            << tab << "                       ( " << setw(9) << I_[6] << " , " << setw(9) << I_[7] << " , " << setw(9) << I_[8] << " )\n"
            << std::resetiosflags(std::ios::right);
      }
   }

   // Extending the indentation for the nested bodies and links
   std::string longtab( tab );
   longtab.append( "  " );

   // Printing all contained bodies
   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      os << "\n";
      b->print( os, longtab.c_str() );
   }

   // Printing all links
   for( ConstLinkIterator l=links_.begin(); l!=links_.end(); ++l ) {
      os << "\n";
      l->print( os, longtab.c_str() );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  UNION SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new union.
 * \ingroup union
 *
 * \param uid The user-specific ID of the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new union.
 *
 * This function creates an empty union. In order to add rigid bodies to the union, the
 * add() function can be used.
 *
 * \b Note: Any body added to an invisible union will also be invisible. However, either
 * the rigid body individually or the entire union can be made visible by the according
 * setVisible() function (e.g. Union::setVisible()).
 */
PE_PUBLIC UnionID createUnion( id_t uid, bool visible )
{
   const bool global( GlobalSection::isActive() );

   // Creating a new union
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   UnionID u = new Union( sid, uid, visible );

   // Checking if the union is created inside a global section
   if( global )
      u->setGlobal();

   // Checking if the union has to be permanently fixed
   else if( u->isAlwaysFixed() )
      u->setFixed( true );

   // Registering the new union with the default body manager
   try {
      theDefaultManager()->add( u );
   }
   catch( std::exception& ) {
      delete u;
      throw;
   }

   // Logging the successful creation of the union
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created union " << sid << "\n"
          << "   User-ID  = " << uid;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new union.
 * \ingroup union
 *
 * \param uid The user-specific ID of the union.
 * \param body The rigid body to be added to the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new union.
 * \exception std::logic_error Global flags of body and union do not match.
 * \exception std::logic_error Invalid adding into a global union inside an exclusive section.
 *
 * This function creates union initially consisting only of the given rigid body \a body.
 * In order to add further rigid bodies to the union, the add() function can be used.
 *
 * \b Note: Any body added to an invisible union will also be invisible. However, either
 * the rigid body individually or the entire union can be made visible by the according
 * setVisible() function (e.g. Union::setVisible()).
 */
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body, bool visible )
{
   // Creating a new union
   UnionID u = createUnion( uid, visible );

   // Adding the rigid body to the union
   try {
      u->add( body );
   }
   catch( std::exception& ) {
      destroy( u );
      throw;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new union.
 * \ingroup union
 *
 * \param uid The user-specific ID of the union.
 * \param body1 The first rigid body to be added to the union.
 * \param body2 The second rigid body to be added to the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new union.
 *
 * This function creates union initially consisting only of the two given rigid bodies
 * \a body1 and \a body2. In order to add further rigid bodies to the union, the add()
 * function can be used.
 *
 * \b Note: Any body added to an invisible union will also be invisible. However, either
 * the rigid body individually or the entire union can be made visible by the according
 * setVisible() function (e.g. Union::setVisible()).
 */
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body1, BodyID body2, bool visible )
{
   // Creating a new union
   UnionID u = createUnion( uid, visible );

   // Adding the rigid bodies to the union
   try {
      u->add( body1 );
      u->add( body2 );
   }
   catch( std::exception& ) {
      destroy( u );
      throw;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new union.
 * \ingroup union
 *
 * \param uid The user-specific ID of the union.
 * \param body1 The first rigid body to be added to the union.
 * \param body2 The second rigid body to be added to the union.
 * \param body3 The third rigid body to be added to the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new union.
 *
 * This function creates union initially consisting only of the three given rigid bodies
 * \a body1, \a body2 and \a body3. In order to add further rigid bodies to the union, the
 * add() function can be used.
 *
 * \b Note: Any body added to an invisible union will also be invisible. However, either
 * the rigid body individually or the entire union can be made visible by the according
 * setVisible() function (e.g. Union::setVisible()).
 */
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3, bool visible )
{
   // Creating a new union
   UnionID u = createUnion( uid, visible );

   // Adding the rigid bodies to the union
   try {
      u->add( body1 );
      u->add( body2 );
      u->add( body3 );
   }
   catch( std::exception& ) {
      destroy( u );
      throw;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new union.
 * \ingroup union
 *
 * \param uid The user-specific ID of the union.
 * \param body1 The first rigid body to be added to the union.
 * \param body2 The second rigid body to be added to the union.
 * \param body3 The third rigid body to be added to the union.
 * \param body4 The fourth rigid body to be added to the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new union.
 *
 * This function creates union initially consisting only of the four given rigid bodies
 * \a body1, \a body2, \a body3 and \a body4. In order to add further rigid bodies to the
 * union, the add() function can be used.
 *
 * \b Note: Any body added to an invisible union will also be invisible. However, either
 * the rigid body individually or the entire union can be made visible by the according
 * setVisible() function (e.g. Union::setVisible()).
 */
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3,
                               BodyID body4, bool visible )
{
   // Creating a new union
   UnionID u = createUnion( uid, visible );

   // Adding the rigid bodies to the union
   try {
      u->add( body1 );
      u->add( body2 );
      u->add( body3 );
      u->add( body4 );
   }
   catch( std::exception& ) {
      destroy( u );
      throw;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new union.
 * \ingroup union
 *
 * \param uid The user-specific ID of the union.
 * \param body1 The first rigid body to be added to the union.
 * \param body2 The second rigid body to be added to the union.
 * \param body3 The third rigid body to be added to the union.
 * \param body4 The fourth rigid body to be added to the union.
 * \param body5 The fifth rigid body to be added to the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new union.
 *
 * This function creates union initially consisting only of the four given rigid bodies
 * \a body1, \a body2, \a body3, \a body4 and \a body5. In order to add further rigid
 * bodies to the union, the add() function can be used.
 *
 * \b Note: Any body added to an invisible union will also be invisible. However, either
 * the rigid body individually or the entire union can be made visible by the according
 * setVisible() function (e.g. Union::setVisible()).
 */
PE_PUBLIC UnionID createUnion( id_t uid , BodyID body1, BodyID body2, BodyID body3,
                               BodyID body4, BodyID body5, bool visible )
{
   // Creating a new union
   UnionID u = createUnion( uid, visible );

   // Adding the rigid bodies to the union
   try {
      u->add( body1 );
      u->add( body2 );
      u->add( body3 );
      u->add( body4 );
      u->add( body5 );
   }
   catch( std::exception& ) {
      destroy( u );
      throw;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a remote union.
 * \ingroup union
 *
 * \param sid Unique system-specific ID for the union.
 * \param uid User-specific ID for the union.
 * \param gpos Global center of mass of the union.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param mass The total mass of the union.
 * \param I The moment of inertia in reference to the union's own body frame.
 * \param q The orientation of the union's body frame in the global world frame.
 * \param aabb Coordinates of the axis-aligned bounding box of the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \param fixed \a true to fix the union, \a false to unfix it.
 * \param reg \a true to register the object in the default body manager.
 * \return Handle for the new union.
 *
 * This function instantiates a copy of a union with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of an union residing on a remote
 * MPI process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
UnionID instantiateUnion( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, real mass,
                          const Mat3& I, const Quat& q, const Vec6& aabb, bool visible, bool fixed, bool reg )
{
   // Checking whether the function is used inside a pe_INSTANTIATE_UNION section
   pe_INTERNAL_ASSERT( InstantiateUnion::isActive(), "Invalid function call detected" );

   // Instantiating the union
   UnionID u = new Union( sid, uid, gpos, rpos, mass, I, q, aabb, visible, fixed );

   // Registering the union with the default body manager
   if( reg ) {
      try {
         theDefaultManager()->add( u );
      }
      catch( std::exception& ) {
         delete u;
         throw;
      }
   }

   // Logging the successful instantiation of the union
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated union " << sid << "\n"
          << "   User-ID = " << uid;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Instantiates a process local union from given union parameter data.
 * \param objparam A union description in form of union parameter data.
 * \param offset An offset to subtract from all global positions.
 * \param remote Flag indicating whether the union is remote or not.
 * \param reg \a true to register the object in the default body manager.
 * \return void
 */
UnionID instantiateUnion( const Union::Parameters& objparam, const Vec3& offset, bool remote, bool reg ) {
   UnionID ret = 0;

   pe_INSTANTIATE_UNION( obj, objparam.sid_, objparam.uid_, objparam.gpos_ - offset, objparam.rpos_, objparam.m_, objparam.I_, objparam.q_, objparam.aabb_, objparam.visible_, objparam.fixed_, reg ) {
      ret = obj;

      for( std::vector<Sphere::Parameters>::const_iterator it = objparam.spheres_.begin(); it != objparam.spheres_.end(); ++it ) {
         const Sphere::Parameters& subobjparam = *it;
         SphereID subobj = instantiateSphere( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - offset, subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( remote );
      }
      for( std::vector<Box::Parameters>::const_iterator it = objparam.boxes_.begin(); it != objparam.boxes_.end(); ++it ) {
         const Box::Parameters& subobjparam = *it;
         BoxID subobj = instantiateBox( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - offset, subobjparam.rpos_, subobjparam.q_, subobjparam.lengths_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( remote );
      }
      for( std::vector<Capsule::Parameters>::const_iterator it = objparam.capsules_.begin(); it != objparam.capsules_.end(); ++it ) {
         const Capsule::Parameters& subobjparam = *it;
         CapsuleID subobj = instantiateCapsule( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - offset, subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.length_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( remote );
      }
      for( std::vector<Cylinder::Parameters>::const_iterator it = objparam.cylinders_.begin(); it != objparam.cylinders_.end(); ++it ) {
         const Cylinder::Parameters& subobjparam = *it;
         CylinderID subobj = instantiateCylinder( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - offset, subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.length_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( remote );
      }
      for( std::vector<Union::Parameters>::const_iterator it = objparam.unions_.begin(); it != objparam.unions_.end(); ++it ) {
         instantiateUnion( *it, offset, remote );
         //const Union::Parameters& subobjparam = *it;
         //UnionID subobj = instantiateUnion( subobjparam, remote );
         //subobj->setRemote( remote );
      }
      for( std::vector<Plane::Parameters>::const_iterator it = objparam.planes_.begin(); it != objparam.planes_.end(); ++it ) {
         const Plane::Parameters& subobjparam = *it;
         PlaneID subobj = instantiatePlane( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - offset, subobjparam.rpos_, subobjparam.q_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_ );
         subobj->setRemote( remote );
      }
   }
   ret->setRemote( remote );

   return ret;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new tetrasphere.
 * \ingroup union
 *
 * \param uid The user-specific ID of the tetrasphere.
 * \param gpos The global position of the center of the tetrasphere.
 * \param radius Radius of the four spheres of the tetrasphere \f$ (0..\infty) \f$.
 * \param material The material of the tetrasphere.
 * \param visible Specifies if the tetrasphere is visible in a visualization.
 * \return Handle for the new tetrasphere.
 * \exception std::invalid_argument Invalid sphere radius.
 * \exception std::invalid_argument Invalid global union position.
 *
 * \image html tetrasphere.png
 * \image latex tetrasphere.eps "Tetrasphere" width=200pt
 *
 * This function creates an union containing four spheres that are arranged in a tetrahedral
 * fashion. Each sphere will have the same radius and material. \a gpos specifies the global
 * position of the center of mass of the entire union.
 */
PE_PUBLIC UnionID createTetrasphere( id_t uid, const Vec3& gpos, real radius,
                                     MaterialID material, bool visible )
{
   // Checking the radius
   if( radius <= real(0) ) throw std::invalid_argument( "Invalid sphere radius" );

   // Creating a new union
   UnionID u( 0 );
   pe_CREATE_UNION( newunion, uid )
   {
      const real sqrt3 ( std::sqrt( real(3) ) );
      const real sqrt83( std::sqrt( real(8)/real(3) ) );

      // Creating the four contained spheres
      createSphere( 1,        0,            0,             0, radius, material, visible );
      createSphere( 2, 2*radius,            0,             0, radius, material, visible );
      createSphere( 3,   radius, radius*sqrt3,             0, radius, material, visible );
      createSphere( 4,   radius, radius/sqrt3, sqrt83*radius, radius, material, visible );

      // Setting the global position and the visibility of the union
      newunion->setPosition( gpos    );
      newunion->setVisible ( visible );

      u = newunion;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new granular particle.
 * \ingroup union
 *
 * \param uid The user-specific ID of the particle.
 * \param gpos The global position of the center of the particle.
 * \param radius Radius of the granular particle \f$ (0..\infty) \f$.
 * \param material The material of the particle.
 * \param visible Specifies if the particle is visible in a visualization.
 * \return Handle for the new particle.
 * \exception std::invalid_argument Invalid particle parameters.
 *
 * \image html granularparticle.png
 * \image latex granularparticle.eps "Examples for granular particles" width=650pt
 *
 * This function creates a union containing two to four spheres that are arranged in a random
 * fashion. Each sphere will have the same material. \a gpos specifies the global position of
 * the center of mass of the entire union and \a radius specifies the radius of the granular
 * particle.
 */
PE_PUBLIC UnionID createGranularParticle( id_t uid, const Vec3& gpos, real radius,
                                          MaterialID material, bool visible )
{
   const size_t number( rand<size_t>( 2, 4 ) );
   const real   rmin  ( real(0.6)*radius );
   const real   rmax  ( real(0.8)*radius );

   // Creating a new union
   UnionID u( 0 );
   pe_CREATE_UNION( newunion, uid )
   {
      // Creating a random number of contained spheres
      for( size_t i=0; i<number; ++i ) {
         const real rad( rand<real>( rmin, rmax ) );
         Vec3 disp( rand<real>(-1,1), rand<real>(-1,1), rand<real>(-1,1) );
         disp.normalize();
         disp.scale( radius - rad );
         createSphere( uid, disp, rad, material, visible );
      }

      // Setting the global position and the visibility of the union
      newunion->setPosition( gpos    );
      newunion->setVisible ( visible );

      u = newunion;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Random sphere setup of the union.
 *
 * \param uid The user-specific ID of the union.
 * \param gpos The global position of the center of the agglomerate.
 * \param radius The radius of the spheres to be created.
 * \param material The material of the spheres to be created.
 * \param number The number of spheres to be created.
 * \param threshold Degree of clustering of the spheres \f$[0..1]\f$.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return void
 * \exception std::invalid_argument Invalid sphere radius.
 * \exception std::invalid_argument Invalid number of spheres.
 * \exception std::invalid_argument Invalid threshold value.
 * \exception std::invalid_argument Invalid global union position.
 *
 * \image html agglomerate.png
 * \image latex agglomerate.eps "Particle agglomeate" width=430pt
 *
 * This function creates a random particle agglomerate consisting of \a number random spheres.
 * Each of the spheres is given the specified radius and material. The threshold value, which
 * has to be in the range \f$ [0..1] \f$, varies the packing density of the resulting union
 * from clustered (0) to branched (1).
 */
PE_PUBLIC UnionID createAgglomerate( id_t uid, const Vec3& gpos, real radius, MaterialID material,
                                     size_t number, real threshold, bool visible )
{
   // Checking the radius
   if( radius <= real(0) ) throw std::invalid_argument( "Invalid sphere radius" );

   // Checking the number of random spheres
   if( number == 0 ) throw std::invalid_argument( "Invalid number of spheres" );

   // Checking the threshold value
   if( threshold < real(0) || threshold > real(1) )
      throw std::invalid_argument( "Invalid threshold value" );

   bool overlap;
   size_t sid(0);
   Vec3 pos, n, tmp;
   SphereID sphere;
   SphereVector spheres;
   SphereVector::SizeType index;

   const real thresholdvalue( threshold*radius );

   // Creating the agglomerate
   UnionID u( 0 );
   pe_CREATE_UNION( newunion, uid )
   {
      // Creating an initial sphere
      spheres.pushBack( createSphere( ++sid, 0, 0, 0, radius, material ) );
      --number;

      // Creating the rest of the spheres
      while( number )
      {
         // Selecting a sphere
         index  = rand<SphereVector::SizeType>( 0, spheres.size()-1 );
         sphere = spheres[index];

         // Creating a random direction
         n[0] = rand<real>( -static_cast<real>(0.5), static_cast<real>(0.5) );
         n[1] = rand<real>( -static_cast<real>(0.5), static_cast<real>(0.5) );
         n[2] = rand<real>( -static_cast<real>(0.5), static_cast<real>(0.5) );
         n.normalize();

         // Calculating the new sphere position
         tmp = sphere->getPosition();
         pos = tmp + static_cast<real>( 2 ) * radius * n;

         // Testing for overlaps
         overlap = false;

         for( SphereVector::ConstIterator s=spheres.begin(); s!=spheres.end(); ++s )
         {
            if( *s == sphere ) continue;
            tmp = s->getPosition() - pos;
            if( ( tmp.length() - static_cast<real>( 2 )*radius ) < thresholdvalue ) {
               overlap = true;
               break;
            }
         }

         if( !overlap ) {
            spheres.pushBack( createSphere( ++sid, pos, radius, material, visible ) );
            --number;
         }
      }

      // Setting the global position and the visibility of the union
      newunion->setPosition( gpos    );
      newunion->setVisible ( visible );

      u = newunion;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a y-shaped (tristar) union consisting of three capsules.
 *
 * \param uid The user-specific ID of the tristar.
 * \param gpos The global position of the center of the tristar.
 * \param radius Radius of the three capsules of the tristar \f$ (0..\infty) \f$.
 * \param length Length of the three capsules of the tristar \f$ (0..\infty) \f$.
 * \param material The material of the tristar.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new tristar.
 * \exception std::invalid_argument Invalid capsule radius.
 * \exception std::invalid_argument Invalid capsule length.
 * \exception std::invalid_argument Invalid global tristar position.
 *
 * \image html tristar.png
 * \image latex tristar.eps "Tristar union" width=200pt
 *
 * This function creates a y-shaped (tristar) union consisting of three capsules. Each capsule
 * will have the same radius, length and material. The given global position \a gpos specifies
 * the position of the center of mass of the entire union.
 */
PE_PUBLIC UnionID createTristar( id_t uid, const Vec3& gpos, real radius, real length,
                                 MaterialID material, bool visible )
{
   // Checking the global position of the tristar
   if( !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global tristar position" );

   // Creating the tristar union
   UnionID u( 0 );
   pe_CREATE_UNION( newunion, uid )
   {
      const Vec3 cgpos( gpos[0]+length/real(2), gpos[1], gpos[2] );  // Center of mass of the capsules
      const Vec3 axis ( 0, 0, 1 );                                   // Rotation axis
      const real angle( real(2)/real(3)*M_PI );                      // Rotation angle

      // Creating the first capsule
      createCapsule( 1, cgpos, radius, length, material, visible );

      // Creating the second capsule
      CapsuleID capsule2 = createCapsule( 2, cgpos, radius, length, material, visible );
      capsule2->rotateAroundPoint( gpos, axis, angle );

      // Creating the third capsule
      CapsuleID capsule3 = createCapsule( 3, cgpos, radius, length, material, visible );
      capsule3->rotateAroundPoint( gpos, axis, -angle );

      // Setting the visibility of the union
      newunion->setPosition( gpos    );
      newunion->setVisible ( visible );

      u = newunion;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a chain link consisting of eight capsules
 *
 * \param uid The user-specific ID of the chain link.
 * \param gpos The global position of the center of the chain link.
 * \param radius Radius of the eight capsules of the chain link \f$ (0..\infty) \f$.
 * \param length Length of the chain link \f$ (0..\infty) \f$.
 * \param width Width of the chain link \f$ (0..\infty) \f$.
 * \param material The material of the chain link.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new chain link.
 * \exception std::invalid_argument Invalid capsule radius.
 * \exception std::invalid_argument Invalid capsule length.
 * \exception std::invalid_argument Invalid global chain link position.
 *
 * \image html chainlink.png
 * \image latex chainlink.eps "Chain link union" width=400pt
 *
 * This function creates a chain link consisting of eight capsules. The \a radius parameter
 * specifies the radius of every single capsule and the \a length and \a width parameters
 * specify the length and width of the entire chain link (see the above illustration). The
 * given global position \a gpos specifies the position of the center of mass of the chain
 * link.
 */
PE_PUBLIC UnionID createChainLink( id_t uid, const Vec3& gpos, real radius, real length, real width,
                                   MaterialID material, bool visible )
{
   // Checking the global position of the chain link
   if( !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global chain link position" );

   const real diag( std::sqrt( width ) );
   const real disp( std::sqrt( sq(diag)/real(2) )  );
   const real hl  ( length / real(2) );
   const real hw  ( width  / real(2) );
   const real px  ( hl - disp/real(2) );
   const real py  ( hw - disp/real(2) );

   // Adjusting the length and width of the chain link
   length = length - real(2)*disp;
   width  = width  - real(2)*disp;

   // Creating the chain link union
   UnionID u( 0 );
   pe_CREATE_UNION( link, uid )
   {
      // Creating the first capsule
      createCapsule( 1,  0, -hw, 0, radius, length, material );

      // Creating the second capsule
      createCapsule( 2,  0,  hw, 0, radius, length, material );

      // Creating the third capsule
      CapsuleID c3 = createCapsule( 3, -hl, 0, 0, radius, width, material );
      c3->rotate( 0.0, 0.0, M_PI/2 );

      // Creating the fourth capsule
      CapsuleID c4 = createCapsule( 4,  hl, 0, 0, radius, width, material );
      c4->rotate( 0.0, 0.0, M_PI/2 );

      // Creating the fifth capsule
      CapsuleID c5 = createCapsule( 5, -px, -py, 0, radius, diag, material );
      c5->rotate( 0.0, 0.0, -M_PI/4 );

      // Creating the sixth capsule
      CapsuleID c6 = createCapsule( 6,  px, -py, 0, radius, diag, material );
      c6->rotate( 0.0, 0.0, M_PI/4 );

      // Creating the seventh capsule
      CapsuleID c7 = createCapsule( 7, -px,  py, 0, radius, diag, material );
      c7->rotate( 0.0, 0.0, M_PI/4 );

      // Creating the eighth capsule
      CapsuleID c8 = createCapsule( 8,  px,  py, 0, radius, diag, material );
      c8->rotate( 0.0, 0.0, -M_PI/4 );

      // Setting the global position and the visibility of the union
      link->setPosition( gpos    );
      link->setVisible ( visible );

      u = link;
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a dummy.
 *
 * \param uid The user-specific ID of the dummy.
 * \param gpos The global position of the center of the dummy.
 * \param size Size of the dummy \f$ (0..\infty) \f$.
 * \param material The material of the dummy.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new dummy.
 * \exception std::invalid_argument Invalid dummy size.
 * \exception std::invalid_argument Invalid global dummy position.
 * \exception std::runtime_error Union is not exclusively created on one process.
 *
 * \image html dummy.png
 * \image latex dummy.eps "Dummy union" width=400pt
 *
 * This function creates a dummy consisting of a single box (the torso), one capsule (the arms)
 * and three spheres (the head and the two legs) as illustrated in the figure above. The \a size
 * parameter specifies the height of the dummy. All other proportions are scaled accordingly.
 * The given global position \a gpos specifies the position of the center of mass of the dummy.
 */
PE_PUBLIC UnionID createDummy( id_t uid, const Vec3& gpos, real size, MaterialID material,
                               bool visible )
{
   // Checking the dummy size parameter
   if( size <= real(0) ) throw std::invalid_argument( "Invalid dummy size" );

   // Checking the global position of the dummy
   if( !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global dummy position" );

   // Scaling factor for the dummy size
   const real f( size/static_cast<real>( 5 ) );

   // Creating the dummy union
   UnionID u( 0 );
   pe_CREATE_UNION( dummy, uid )
   {
      // Adding the torso, the head, the arms and the legs (in this order)
      createBox    ( 1,  static_cast<real>(0.0)  , static_cast<real>(0.0),  static_cast<real>(0.0)  , static_cast<real>(2.0)*f, static_cast<real>(1.05)*f, static_cast<real>(3.0)*f, material );
      createSphere ( 2,  static_cast<real>(0.0)  , static_cast<real>(0.0),  static_cast<real>(2.0)*f, static_cast<real>(1.0)*f, material );
      createCapsule( 3,  static_cast<real>(0.0)  , static_cast<real>(0.0),  static_cast<real>(1.0)*f, static_cast<real>(0.5)*f, static_cast<real>(3.0)*f, material );
      createSphere ( 4,  static_cast<real>(0.5)*f, static_cast<real>(0.0), -static_cast<real>(1.5)*f, static_cast<real>(0.5)*f, material );
      createSphere ( 5, -static_cast<real>(0.5)*f, static_cast<real>(0.0), -static_cast<real>(1.5)*f, static_cast<real>(0.5)*f, material );

      // Setting the global position and visibility of the dummy
      dummy->setPosition( gpos    );
      dummy->setVisible ( visible );

      u = dummy;
   }

   return u;
}
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Setup of a new stella octangula.
 * \ingroup union
 *
 * \param uid The user-specific ID of the particle.
 * \param gpos The global position of the center of the particle.
 * \param radius Radius of the stella octangula \f$ (0..\infty) \f$.
 * \param material The material of the particle.
 * \param pov Handel to the povray writer if setColoredTriangleTexture() should be called for the tetraheadrons. Otherwise NULL
 * \param visible Specifies if the particle is visible in a visualization.
 * \return Handle for the new stella octangula.
 *
 *
 * This function creates a union containing two tetrahedron arranged as an stella octangula,
 * meaning that the tetrahedron are aranged in a way that all tip penetrats a face of the
 * the other tetrahedron in its centre
 */
PE_PUBLIC UnionID createStellaOctangula( id_t uid, const Vec3& gpos, real radius,
                                          MaterialID material, povray::WriterID pov, bool visible )
{
   // Creating a new union
   UnionID u( 0 );
   pe_CREATE_UNION( newunion, uid )
   {

      TriangleMeshID utet1 = createRegularTetrahedron(uid, Vec3(0.0, 0.0, 0.0), radius, material);
      TriangleMeshID utet2 = createRegularTetrahedron(uid, Vec3(0.0, 0.0, 0.0), radius, material);
      utet2->rotate(0.0, M_PI, M_PI);

      utet1->setColoredTriangleTexture(pov);
      utet2->setColoredTriangleTexture(pov);


      // Setting the global position and the visibility of the union
      newunion->setPosition( gpos    );
      newunion->setVisible ( visible );

      u = newunion;
   }

   return u;
}
//*************************************************************************************************



//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for unions.
 * \ingroup union
 *
 * \param os Reference to the output stream.
 * \param u Reference to a constant union object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Union& u )
{
   os << "--" << pe_BROWN << "UNION PARAMETERS" << pe_OLDCOLOR
      << "--------------------------------------------------------------\n";
   u.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for union handles.
 * \ingroup union
 *
 * \param os Reference to the output stream.
 * \param u Constant union handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstUnionID u )
{
   os << "--" << pe_BROWN << "UNION PARAMETERS" << pe_OLDCOLOR
      << "--------------------------------------------------------------\n";
   u->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
