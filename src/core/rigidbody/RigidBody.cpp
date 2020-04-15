//=================================================================================================
/*!
 *  \file src/core/rigidbody/RigidBody.cpp
 *  \brief Source file for the RigidBody class
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
#include <pe/core/attachable/Attachable.h>
#include <pe/core/BodyManager.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the RigidBody class.
 *
 * \param type The type of the rigid body.
 * \param finite Specifies if the rigid body is finite or not.
 * \param visible Specifies if the rigid body is visible or not.
 * \param sid The unique system-specific ID of the rigid body.
 * \param uid The user-specific ID of the rigid body.
 */
RigidBody::RigidBody( GeomType type, bool finite, bool visible, id_t sid, id_t uid )
   : Parent  (this)     // Initialization of the body base class
   , manager_(0)        // The rigid body manager responsible for the rigid body
   , sb_     (this)     // The superordinate rigid body
   , type_   (type)     // Geometry type of the rigid body
   , finite_ (finite)   // Finiteness flag
   , visible_(visible)  // Visibility flag
   , remote_ (false)    // Remote flag
   , global_ (false)    // Global flag
   , sid_    (sid)      // System-specific body index
   , uid_    (uid)      // User-specific body ID
   , contacts_()        // Vector of the currently attached contacts
   , superBodies_()     // All superordinate bodies containing this rigid body
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the RigidBody class.
 */
RigidBody::~RigidBody()
{
   // Detaching all attached attachables
   while( !attachables_.isEmpty() ) {
      detach( attachables_[0] );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the global acceleration of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return The global acceleration.
 *
 * The function calculates the global acceleration of a point in global coordinates.
 */
const Vec3 RigidBody::accFromWF( const Vec3& gpos ) const
{
   if( !hasSuperBody() )
   {
      // Calculating the linear acceleration
      const Vec3 vdot( force_ * invMass_ );

      // Calculating the angular acceleration
      const Vec3 wdot( R_ * Iinv_ * trans(R_) * ( torque_ - w_ % ( R_ * I_ * trans(R_) * w_ ) ) );

      // Calculating the distance to the center of mass of the superordinate body
      const Vec3 r( gpos - gpos_ );

      // Calculating the acceleration of the point 'gpos'
      return vdot + wdot % r + w_ % ( w_ % r );
   }
   else return sb_->accFromWF( gpos );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the global position (the center of mass) of the rigid body fixed.
 *
 * \param fixed \a true to fix the rigid body, \a false to unfix it.
 * \return void
 *
 * This function either fixes or unfixes the global position (the center of mass) of a finite
 * rigid body. If the body is contained in a superordinate body, fixing/unfixing the contained
 * body will also fix/unfix the global position of the superordinate body. In case the body
 * is infinite or contained in an infinite superordinate body (as for instance a plane or an
 * union containing a plane) the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
void RigidBody::setFixed( bool fixed )
{
   // Early exit in case the flag is already set accordingly, the body is infinite or global
   if( fixed_ == fixed || !finite_ )
      return;

   // Fixing/unfixing the global position of the rigid body
   if( fixed ) fix();
   else if( !isAlwaysFixed() ) unfix();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void RigidBody::setVisible( bool visible )
 * \brief Setting the rigid body visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the rigid body visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the rigid body visible/invisible in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void RigidBody::setPosition( real px, real py, real pz )
 * \brief Setting the global position of the rigid body.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 *
 * This function sets the global position of the rigid body to the given coordinate (px,py,pz).
 *
 * \b Note:
 * - Setting the position of a rigid body contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 *   on one process may invalidate the settings of the body on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the rigid body is global (i.e. if it was created
 *   inside a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is
 *   not allowed to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void RigidBody::setPosition( const Vec3& gpos )
 * \brief Setting the global position of the rigid body.
 *
 * \param gpos The global position.
 * \return void
 *
 * This function sets the global position of the rigid body to the given coordinate \a gpos.
 *
 * \b Note:
 * - Setting the position of a rigid body contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 *   on one process may invalidate the settings of the body on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the rigid body is global (i.e. if it was created
 *   inside a pe_GLOBAL_SECTION) the position change must be applied on all processes. It is
 *   not allowed to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void RigidBody::setOrientation( real r, real i, real j, real k )
 * \brief Setting the global orientation of the rigid body.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 *
 * This function sets the global orientation of the rigid body to the given quaternion
 * \a (r,i,j,k).
 *
 * \b Note:
 * - Setting the orientation of a rigid body contained in a union changes the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 *   on one process may invalidate the settings of the box on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the rigid body is global (i.e. if it was created
 *   inside a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void RigidBody::setOrientation( const Quat& q )
 * \brief Setting the global orientation of the rigid body.
 *
 * \param q The global orientation.
 * \return void
 *
 * This function sets the global orientation of the rigid body to the given quaternion \a q.
 *
 * \b Note:
 * - Setting the orientation of a rigid body contained in a union changes the mass distribution
 *   and geometry of the union. Therefore this changes the union and may cause an invalidation
 *   of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 *   on one process may invalidate the settings of the box on another process. In order to
 *   synchronize all rigid bodies after local changes, the World::synchronize() function should
 *   be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 *   bodies are neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the rigid body is global (i.e. if it was created
 *   inside a pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is
 *   not allowed to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do
 *   this results in a \a std::logic_error.
 */
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the calculation of the bounding box of the geometric primitive.
 *
 * \return void
 *
 * This default implementation is overridden by the bounding box calculations of each geometric
 * primitive. The default implementation effects an assertion failure unless the engine is
 * compiled in release mode.
 */
void RigidBody::calcBoundingBox()
{
    pe_INTERNAL_ASSERT( false, "Undefined bounding box calculation" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return The support point in world-frame coordinates in direction a\ d.
 */
Vec3 RigidBody::support(const Vec3& /*d*/) const
{
   pe_INTERNAL_ASSERT( false, "Undefined support point calculation" );
   return gpos_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 */
Vec3 RigidBody::supportContactThreshold(const Vec3& /*d*/) const
{
   pe_INTERNAL_ASSERT( false, "Undefined support point calculation" );
   return gpos_;
}
//*************************************************************************************************


//=================================================================================================
//
//  FIXATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the rigid body is permanently fixed.
 *
 * \return \a true in case the rigid body is permanently fixed, \a false in case it is not.
 *
 * This function returns whether the rigid body is permanently fixed (i.e. cannot move at all)
 * or can be unfixed. In case the rigid body is always fixed, the function returns \a true, in
 * case it can be unfixed it returns \a false.
 */
bool RigidBody::isAlwaysFixed() const
{
   return !finite_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position (the center of mass) of the rigid body fixed.
 *
 * \return void
 *
 * This function fixes the global position (the center of mass) of a finite rigid body. If
 * the body is contained in a superordinate body, fixing the contained body will also fix the
 * global position of the superordinate body. In case the body is infinite or contained in an
 * infinite superordinate body (as for instance a plane or an union containing a plane) the
 * function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
void RigidBody::fix()
{
   fixed_ = true;

   // Adjusting the inverse mass and inverse moment of inertia
   invMass_ = real(0);
   Iinv_    = real(0);

#if !MOBILE_INFINITE
   // Setting the linear and angular velocity to zero
   v_ = real(0);
   w_ = real(0);
#endif

   // Signaling the fixation change to the superordinate body
   signalFixation();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unfixing the global position (the center of mass) of the rigid body.
 *
 * \return void
 *
 * This function unfixes the global position (the center of mass) of a finite rigid body. If
 * the body is contained in a superordinate body, unfixing the contained body will only unfix
 * the global position of the superordinate body if it only contains this rigid body. In case
 * the body is infinite or contained in an infinite superordinate body (as for instance a
 * plane or an union containing a plane) the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
void RigidBody::unfix()
{
   fixed_ = false;

   // Adjusting the inverse mass and inverse moment of inertia
   invMass_ = real(1) / mass_;
   Iinv_    = I_.getInverse();

   // Signaling the fixation change to the superordinate body
   signalFixation();
}
//*************************************************************************************************




//=================================================================================================
//
//  ATTACHABLE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a single attachable with the rigid body.
 *
 * \param attachable The attachable to be registered with the rigid body.
 * \return void
 *
 * This function must NOT be called explicitly, but is reserved for internal use only!
 * Attachables are automatically registered during their construction process.
 */
void RigidBody::registerAttachable( AttachableID attachable )
{
   // Registering the attachable
   attachables_.pushBack( attachable );

   // Registering all newly attached rigid bodies
   const Attachable::Iterator end( attachable->end() );
   for( Attachable::Iterator body=attachable->begin(); body!=end; ++body ) {
      const AttachedBodyIterator bbegin( attachedBodies_.begin() );
      const AttachedBodyIterator bend  ( attachedBodies_.end()   );
      if( *body != this && std::find( bbegin, bend, *body ) == bend )
         attachedBodies_.pushBack( *body );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistering a single attachable from the rigid body.
 *
 * \param attachable The attachable to be deregistered from the rigid body.
 * \return void
 *
 * This function must NOT be called explicitly, but is reserved for internal use only!
 * Attachables are automatically deregistered during their destruction process.
 */
void RigidBody::deregisterAttachable( AttachableID attachable )
{
   // Deregistering the attachable
   Attachables::Iterator pos( std::find( attachables_.begin(), attachables_.end(), attachable ) );
   pe_INTERNAL_ASSERT( pos != attachables_.end(), "Attachable is not registered" );
   attachables_.erase( pos );

   // Deregistering all attached rigid bodies
   attachedBodies_.clear();

   // Recreating the vector of attached rigid bodies
   for( Attachables::Iterator it=attachables_.begin(); it!=attachables_.end(); ++it ) {
      const Attachable::Iterator end( it->end() );
      for( Attachable::Iterator body=it->begin(); body!=end; ++body ) {
         const AttachedBodyIterator bbegin( attachedBodies_.begin() );
         const AttachedBodyIterator bend  ( attachedBodies_.end()   );
         if( *body != this && std::find( bbegin, bend, *body ) == bend )
            attachedBodies_.pushBack( *body );
      }
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\fn void RigidBody::update( const Vec3& dp )
 * \brief Translation update of a subordinate rigid body.
 *
 * \param dp Change in the global position of the superordinate rigid body.
 * \return void
 *
 * This function calculates the necessary updates for a subordinate rigid body contained
 * in a superordinate rigid body that has performed a translational movement. This function
 * is triggered automatically by the superordinate body in case of a translational movement.
 * All classes deriving from the RigidBody class have to implement this function to update
 * the properties of the rigid body. The following properties of the body might change due
 * to this translation. All derived classes have to make sure these properties are updated
 * correctly:
 *
 *   - the global position
 *   - the axis-aligned bounding box
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void RigidBody::update( const Quat& dq )
 * \brief Rotation update of a subordinate rigid body.
 *
 * \param dq Change in the orientation of the superordinate rigid body.
 * \return void
 *
 * This function calculates the necessary updates for a subordinate rigid body contained
 * in a superordinate rigid body that has performed a rotational movement. The function
 * is triggered automatically by the superordinate body in case of a rotational movement.
 * All classes deriving from the RigidBody class have to implement this function to update
 * the properties of the rigid body. The following properties of the body might change due
 * to this rotation. All derived classes have to make sure these properties are updated
 * correctly:
 *
 *   - the global position
 *   - the orientation/rotation (i.e. the quaterion and the rotation matrix)
 *   - the axis-aligned bounding box
 */
//*************************************************************************************************




//=================================================================================================
//
//  FUNCTIONS FOR INTERNAL CHANGES IN COMPOUND GEOMETRIES
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Handling an internal modification of a contained subordinate body.
 *
 * \return void
 *
 * This function handles an internal modification of one of the contained subordinate bodies.
 * Derived compound geometries that contain other primitive or compound bodies are required
 * to override this function in order to react to the modification. All primitive geometries
 * can use the empty default implementation.
 */
void RigidBody::handleModification()
{
   pe_INTERNAL_ASSERT( false, "Invalid call of default handle function" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling a position change of a contained subordinate body.
 *
 * \return void
 *
 * This function handles a translation or position change of one of the contained subordinate
 * bodies. Derived compound geometries that contain other primitive bodies are required to
 * override this function in order to react to the translation. All primitive geometries can
 * use the empty default implementation.
 */
void RigidBody::handleTranslation()
{
   pe_INTERNAL_ASSERT( false, "Invalid call of default handle function" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling an orientation change of a contained subordinate body.
 *
 * \return void
 *
 * This function handles a rotation or orientation change of one of the contained subordinate
 * bodies. Derived compound geometries that contain other primitive bodies are required to
 * override this function in order to react to the rotation. All primitive geometries can use
 * the empty default implementation.
 */
void RigidBody::handleRotation()
{
   pe_INTERNAL_ASSERT( false, "Invalid call of default handle function" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handling a fixation change of a contained subordinate body.
 *
 * \return void
 *
 * This function handles a fixation change of one of the contained subordinate bodies. Derived
 * compound geometries that contain other primitive bodies are required to override this function
 * in order to react to the fixation change. All primitive geometries can use the empty default
 * implementation.
 */
void RigidBody::handleFixation()
{
   pe_INTERNAL_ASSERT( false, "Invalid call of default handle function" );
}
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks the validity of the state of the rigid body.
 *
 * \return \a true if no error is detected, \a false if any error if found.
 */
bool RigidBody::checkInvariants()
{
   bool error( false );

   // Checking that an infinite rigid body is fixed
   if( !finite_ && !fixed_ ) {
      std::cerr << pe_RED << "\n **** Infinite rigid body is not fixed ****" << pe_OLDCOLOR;
      error = true;
   }

   // Checking that a global rigid body is local
   if( global_ && remote_ ) {
      std::cerr << pe_RED << "\n **** Global rigid body is not local ****" << pe_OLDCOLOR;
      error = true;
   }

   // Checking the mass properties
   if( mass_ < real(0) || invMass_ < real(0) ) {
      std::cerr << pe_RED << "\n **** Invalid mass properties detected ****" << pe_OLDCOLOR;
      error = true;
   }

   // Checking the mass properties of a fixed rigid body
   if( fixed_ && ( invMass_ != real(0) || Iinv_ != real(0) ) ) {
      std::cerr << pe_RED << "\n **** Invalid mass properties for fixed rigid body ****" << pe_OLDCOLOR;
      error = true;
   }

   // Checking that a fixed or sleeping rigid body has zero linear and angular velocity
#if MOBILE_INFINITE
   if( !awake_ && ( v_ != real(0) || w_ != real(0) ) ) {
#else
   if( ( fixed_ || !awake_ ) && ( v_ != real(0) || w_ != real(0) ) ) {
#endif
      std::cerr << pe_RED << "\n **** Invalid velocity for immobile rigid body " << uid_ << " ****" << pe_OLDCOLOR;
      error = true;
   }

   // Checking for nan-values
   if( isnan(  gpos_  ) || isnan( v_ ) || isnan( w_ ) || isnan( force_ ) ||
       isnan( torque_ ) || isnan( q_ ) || isnan( R_ ) ) {
      std::cerr << pe_RED << "\n **** Nan-value detected in rigid body ****" << pe_OLDCOLOR;
      error = true;
   }

   // Printing the current state of the rigid body
   if( error ) {
      std::cerr << std::boolalpha << "\n"
                << "   User-ID           = " << uid_ << "\n"
                << "   System-ID         = " << sid_ << "\n"
                << "   Finite            = " << finite_ << "\n"
                << "   Awake             = " << awake_ << "\n"
                << "   Fixed             = " << fixed_ << "\n"
                << "   Visible           = " << visible_ << "\n"
                << "   Remote            = " << remote_ << "\n"
                << "   Global            = " << global_ << "\n"
                << "   Total mass        = " << mass_ << "\n"
                << "   Inverse mass      = " << invMass_ << "\n"
                << "   Global position   = " << gpos_ << "\n"
                << "   Relative position = " << rpos_ << "\n"
                << "   Linear velocity   = " << v_ << "\n"
                << "   Angular velocity  = " << w_ << "\n"
                << "   Acting force      = " << force_ << "\n"
                << "   Acting torque     = " << torque_ << "\n"
                << "   Bounding box      = " << aabb_ << "\n"
                << "   Quaternion        = " << q_ << "\n"
                << "   Rotation matrix   = ( " << R_[0] << " , " << R_[1] << " , " << R_[2] << " )\n"
                << "                       ( " << R_[3] << " , " << R_[4] << " , " << R_[5] << " )\n"
                << "                       ( " << R_[6] << " , " << R_[7] << " , " << R_[8] << " )\n"
                << std::setiosflags(std::ios::right)
                << "   Moment of inertia :\n"
                << std::setw(18) << I_[0] << std::setw(18) << I_[1] << std::setw(18) << I_[2] << "\n"
                << std::setw(18) << I_[3] << std::setw(18) << I_[4] << std::setw(18) << I_[5] << "\n"
                << std::setw(18) << I_[6] << std::setw(18) << I_[7] << std::setw(18) << I_[8] << "\n"
                << "   Inverse moment of inertia :\n"
                << std::setw(18) << Iinv_[0] << std::setw(18) << Iinv_[1] << std::setw(18) << Iinv_[2] << "\n"
                << std::setw(18) << Iinv_[3] << std::setw(18) << Iinv_[4] << std::setw(18) << Iinv_[5] << "\n"
                << std::setw(18) << Iinv_[6] << std::setw(18) << Iinv_[7] << std::setw(18) << Iinv_[8] << "\n"
                << std::resetiosflags(std::ios::right)
                << std::endl;

      return false;
   }
   else return true;
}
//*************************************************************************************************




//=================================================================================================
//
//  RIGID BODY SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destroys the given rigid body.
 * \ingroup rigid_body
 *
 * \param body The rigid body to be destroyed.
 * \return void
 * \exception std::logic_error Invalid destruction of global rigid body inside an exclusive section.
 *
 * \b Note: Destroying the rigid body invalidates all remaining references/IDs to the body!
 */
PE_PUBLIC void destroy( BodyID body )
{
   // Checking whether a global rigid body is destroyed inside an exclusive section
   if( body->isGlobal() && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid destruction of global rigid body inside an exclusive section" );

   pe_INTERNAL_ASSERT( body->hasManager(), "Body has no rigid body manager" );
   body->getManager()->remove( body );
   delete body;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for rigid bodies.
 * \ingroup rigid_body
 *
 * \param os Reference to the output stream.
 * \param b Reference to a constant rigid body object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const RigidBody& b )
{
   os << "--" << pe_BROWN << "RIGID BODY PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------\n";
   b.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for rigid body handles.
 * \ingroup rigid_body
 *
 * \param os Reference to the output stream.
 * \param b Constant rigid body handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstBodyID b )
{
   os << "--" << pe_BROWN << "RIGID BODY PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------------\n";
   b->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
