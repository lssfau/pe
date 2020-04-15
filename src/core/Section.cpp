//=================================================================================================
/*!
 *  \file src/core/Section.cpp
 *  \brief Source file for the Section class
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

#include <iostream>
#include <pe/core/MPI.h>
#include <pe/core/Section.h>
#include <pe/math/Matrix3x3.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Section class.
 *
 * \param sb The superordinate body of the section.
 */
Section::Section( BodyID sb )
   : SuperBody( unionType, true, false, 0, 0 )  // Initializing the base object
{
   // Setting the superordinate body
   sb_ = sb;
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Section class.
 */
Section::~Section()
{
   // Deregistering from all subordinate bodies
   for( Bodies::Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      deregisterSuperBody( *b );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the entire section visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the section visible, \a false to make it invisible.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::setVisible( bool /*visible*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the section.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::setPosition( real /*px*/, real /*py*/, real /*pz*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the section.
 *
 * \param gpos The global position.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::setPosition( const Vec3& /*gpos*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the section.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::setOrientation( real /*r*/, real /*i*/, real /*j*/, real /*k*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the section.
 *
 * \param q The global orientation.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::setOrientation( const Quat& /*q*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the center of mass of the section by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::translate( real /*dx*/, real /*dy*/, real /*dz*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the center of mass of the box by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::translate( const Vec3& /*dp*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the section around the global rotation axis (x,y,z) by the rotation
 * \brief angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotate( real /*x*/, real /*y*/, real /*z*/, real /*angle*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotate( const Vec3& /*axis*/, real /*angle*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotate( real /*xangle*/, real /*yangle*/, real /*zangle*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotate( const Vec3& /*euler*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotate( const Quat& /*dq*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotateAroundOrigin( real /*x*/, real /*y*/, real /*z*/, real /*angle*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotateAroundOrigin( const Vec3& /*axis*/, real /*angle*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotateAroundOrigin( real /*xangle*/, real /*yangle*/, real /*zangle*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotateAroundOrigin( const Vec3& /*euler*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotateAroundPoint( const Vec3& /*point*/, const Vec3& /*axis*/, real /*angle*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the section around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 *
 * This function merely exists due to the requirements of the RigidBody base class. Under normal
 * conditions, this function should never be called!
 */
void Section::rotateAroundPoint( const Vec3& /*point*/, const Vec3& /*euler*/ )
{
   pe_INTERNAL_ASSERT( false, "Invalid function call" );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the section.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the section, \a false if not.
 */
bool Section::containsRelPoint( real px, real py, real pz ) const
{
   const Vec3 gpos( pointFromBFtoWF( px, py, pz ) );
   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( gpos ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the section.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the section, \a false if not.
 */
bool Section::containsRelPoint( const Vec3& rpos ) const
{
   const Vec3 gpos( pointFromBFtoWF( rpos ) );
   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( gpos ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the section.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the section, \a false if not.
 */
bool Section::containsPoint( real px, real py, real pz ) const
{
   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( px, py, pz ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the section.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the section, \a false if not.
 */
bool Section::containsPoint( const Vec3& gpos ) const
{
   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      if( b->containsPoint( gpos ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the section.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the surface of the section, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Section::isSurfaceRelPoint( real px, real py, real pz ) const
{
   bool surface( false );
   const Vec3 gpos( pointFromBFtoWF( px, py, pz ) );

   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( gpos ) ) return false;
      else if( b->isSurfacePoint( gpos ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the section.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the surface of the section, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Section::isSurfaceRelPoint( const Vec3& rpos ) const
{
   bool surface( false );
   const Vec3 gpos( pointFromBFtoWF( rpos ) );

   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( gpos ) ) return false;
      else if( b->isSurfacePoint( gpos ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the section.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the surface of the section, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Section::isSurfacePoint( real px, real py, real pz ) const
{
   bool surface( false );

   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( px, py, pz ) ) return false;
      else if( b->isSurfacePoint( px, py, pz ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the section.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the surface of the section, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool Section::isSurfacePoint( const Vec3& gpos ) const
{
   bool surface( false );

   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( b->containsPoint( gpos ) ) return false;
      else if( b->isSurfacePoint( gpos ) ) surface = true;
   }

   return surface;
}
//*************************************************************************************************




//=================================================================================================
//
//  SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of the section.
 *
 * \param bodies Vector of all rigid bodies contained in the section.
 * \return void
 */
void Section::setSection( Bodies& bodies )
{
   // Checking the number of bodies
   if( bodies.isEmpty() ) return;

   // Registering the bodies
   for( Bodies::Iterator body=bodies.begin(); body!=bodies.end(); ++body ) {
      pe_INTERNAL_ASSERT( body->isFinite(), "Infinite rigid body detected" );
      registerSuperBody( *body );
      bodies_.pushBack( *body );
   }

   // Setting the orientation and rotation
   q_ = sb_->getQuaternion();
   R_ = sb_->getRotation();

   // Setting the section's total mass and center of mass
   calcCenterOfMass();

   // Calculating the relative position within the superordinate body
   calcRelPosition();

   // Calculating the moment of inertia
   calcInertia();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset of the section.
 *
 * \return void
 */
void Section::clearSection()
{
   // Deregistering from the subordinate bodies and clearing the body vector
   for( Bodies::Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      deregisterSuperBody( *b );
   bodies_.clear();

   // Removing the force and torque
   force_  = real(0);
   torque_ = real(0);
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate section.
 *
 * \param dp Change in the global position of the superordinate rigid body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. In case of a finite superordinate body, this movement only changes the global
 * position. In case of an infinite superordinate body, the relative position in reference
 * to the origin of the global world frame could be changed additionally, if the section itself
 * is finite. If both the superordinate body and the section are infinite, the state of
 * the section is unchanged.
 */
void Section::update( const Vec3& dp )
{
   // Checking the state of the section
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid section state detected" );

   // Updating the global and relative position
   pe_INTERNAL_ASSERT( sb_->isFinite(), "Infinite superordinate union detected" );
   gpos_ += dp;

   // The update of the axis-aligned bounding box is not necessary since
   // the section is not involved in collisions.

   // Checking the state of the section
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid section state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate section.
 *
 * \param dq Change in the orientation of the superordinate rigid body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational movement.
 * This movement involves a change in the global position and the orientation/rotation of the
 * section.
 */
void Section::update( const Quat& dq )
{
   // Checking the state of the section
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid section state detected" );

   // Calculating the new global position
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // The update of the axis-aligned bounding box is not necessary since
   // the section is not involved in collisions.

   // Checking the state of the section
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid section state detected" );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the center of mass of the section.
 *
 * \return void
 */
void Section::calcCenterOfMass()
{
   // Initializing the total mass and the center of mass
   mass_ = real(0);
   gpos_ = real(0);

   // Calculating the total mass and the center of mass
   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      mass_ += b->getMass();
      gpos_ += b->getPosition() * b->getMass();
   }

   invMass_ = real(1) / mass_;
   gpos_ /= mass_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the moment of inertia in reference to the body frame of the section.
 *
 * \return void
 */
void Section::calcInertia()
{
   real mass;
   Vec3 pos;

   // Initializing the body moment of inertia
   I_ = real(0);

   // Calculating the global moment of inertia
   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
   {
      mass = b->getMass();
      pos  = b->getPosition() - gpos_;

      I_ += b->getInertia();
      I_[0] += mass * ( pos[1]*pos[1] + pos[2]*pos[2] );
      I_[1] -= mass * pos[0] * pos[1];
      I_[2] -= mass * pos[0] * pos[2];
      I_[3] -= mass * pos[0] * pos[1];
      I_[4] += mass * ( pos[0]*pos[0] + pos[2]*pos[2] );
      I_[5] -= mass * pos[1] * pos[2];
      I_[6] -= mass * pos[0] * pos[2];
      I_[7] -= mass * pos[1] * pos[2];
      I_[8] += mass * ( pos[0]*pos[0] + pos[1]*pos[1] );
   }

   // Rotating the moment of inertia from the global frame of reference to the body frame of reference
   I_ = trans(R_) * I_ * R_;

   // Calculating the inverse of the body moment of inertia
   Iinv_ = I_.getInverse();
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a section.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the section output.
 * \return void
 */
void Section::print( std::ostream& os, const char* tab ) const
{
   os << tab << " Section containing rigid bodies";
   for( Bodies::ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      os << " " << b->getID();
   os << "\n";
   os << tab << "   Global position = " << gpos_ << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for sections.
 *
 * \param os Reference to the output stream.
 * \param s Reference to a constant section object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const Section& s )
{
   os << "--" << pe_BROWN << "SECTION PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------------\n";
   s.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
