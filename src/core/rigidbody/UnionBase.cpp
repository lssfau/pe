//=================================================================================================
/*!
 *  \file src/core/rigidbody/UnionBase.cpp
 *  \brief Base class for the union compound geometry
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

#include <pe/core/MPI.h>
#include <pe/core/rigidbody/UnionBase.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the UnionBase class.
 *
 * \param sid Unique system-specific ID for the union.
 * \param uid User-specific ID for the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 */
UnionBase::UnionBase( id_t sid, id_t uid, bool visible )
   : SuperBody( unionType, true, visible, sid, uid )  // Initializing the SuperBody base object
   , BodyManager()                                    // Initializing the BodyManager base object
   , bodies_()                                        // Rigid bodies contained in the union
   , links_()                                         // Links contained in the union
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the UnionBase class.
 *
 * Destroying a UnionBase also destroys any contained rigid bodies and links.
 */
UnionBase::~UnionBase()
{
   // Destroying all contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      delete *l;

   // Destroying all contained rigid bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      destroyBody( *b );

   // Clearing the bodies and links
   bodies_.clear();
   links_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  FORCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Removing all acting forces and torques from the contained bodies and links.
 *
 * \return void
 */
void UnionBase::resetForce()
{
   force_  = real(0);  // Resetting the force acting on the union
   torque_ = real(0);  // Resetting the torque acting on the union

   // Removing the force of the contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      b->RigidBody::resetForce();

   // Resetting the force of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->resetForce();
}
//*************************************************************************************************




//=================================================================================================
//
//  MPI FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the remote flag of the union and all contained bodies.
 *
 * \param remote \a true to declare the union remote, \a false declare it local.
 * \return void
 *
 * This function sets the remote flag of the union and all contained rigid bodies. Note that
 * this function should not be used explicitly, but is automatically called during the MPI
 * communication to set the remote status of a union within the simulation world. Using
 * this function explicitly may lead to simulation errors during a parallel simulation!
 */
void UnionBase::setRemote( bool remote )
{
   remote_ = remote;

   // Setting the remote flag of the contained rigid bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      b->setRemote( remote );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Clearing the union.
 *
 * \return void
 *
 * This function resets the union to its initial state and destroys all contained rigid bodies
 * and links.
 */
void UnionBase::clear()
{
   // Resetting the rigid body state
   finite_  = true;
   awake_   = true;
   mass_    = real(0);
   invMass_ = real(0);
   gpos_.reset();
   v_.reset();
   w_.reset();
   force_.reset();
   torque_.reset();
   I_.reset();
   Iinv_.reset();
   q_.reset();
   R_.reset();
   aabb_.reset();

   // Destroying all contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      delete *l;
   links_.clear();

   // Destroying all contained bodies
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      destroyBody( *b );
   bodies_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the bounding box of the union.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the union according to the current
 * position and orientation of the contained rigid bodies.
 */
void UnionBase::calcBoundingBox()
{
   // Setting the bounding box of an empty union
   if( bodies_.isEmpty() ) {
      aabb_[0] = gpos_[0] - real(0.01);
      aabb_[1] = gpos_[1] - real(0.01);
      aabb_[2] = gpos_[2] - real(0.01);
      aabb_[3] = gpos_[0] + real(0.01);
      aabb_[4] = gpos_[1] + real(0.01);
      aabb_[5] = gpos_[2] + real(0.01);
   }

   // Using the bounding box of the first contained bodies as initial bounding box
   // and merging it with the bounding boxes of all other bodies
   else {
      aabb_ = bodies_[0]->getAABB();
      for( ConstIterator b=bodies_.begin()+1; b!=bodies_.end(); ++b )
         aabb_ += b->getAABB();
   }

   pe_INTERNAL_ASSERT( aabb_.isValid()        , "Invalid bounding box detected" );
   pe_INTERNAL_ASSERT( aabb_.contains( gpos_ ), "Invalid bounding box detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the total mass and center of mass.
 *
 * \return void
 */
void UnionBase::calcCenterOfMass()
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );

   // Initializing the total mass and the inverse mass
   mass_    = real(0);
   invMass_ = real(0);

   // Don't calculate the center of mass of an empty union
   if( bodies_.isEmpty() ) return;

   // Calculating the center of mass of a single body
   if( bodies_.size() == 1 )
   {
      const BodyID body( bodies_[0] );
      gpos_ = body->getPosition();
      mass_ = body->getMass();
      if( !fixed_ && mass_ > real(0) )
         invMass_ = real(1) / mass_;
   }

   // Calculating the center of mass of a union containing several bodies
   else
   {
      // Resetting the center of mass
      gpos_.reset();

      // Calculating the center of mass of a finite union
      if( finite_ )
      {
         // Accumulating the mass of all contained rigid bodies
         for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
            pe_INTERNAL_ASSERT( b->isFinite(), "Invalid infinite body in finite union detected" );
            mass_ += b->getMass();
            gpos_ += b->getPosition() * b->getMass();
         }

         // Calculating the center of mass for unions with non-zero mass
         if( mass_ > real(0) ) {
            if( !fixed_ ) invMass_ = real(1) / mass_;
            gpos_ /= mass_;
         }

         // Calculating the center of mass for unions with a mass of zero
         else {
            size_t counter( 0 );
            gpos_.reset();

            for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
               gpos_ += b->getPosition();
               ++counter;
            }

            gpos_ /= counter;
         }
      }

      // Calculating the center of mass of an infinite union
      else {
         size_t counter( 0 );

         for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
            if( b->isFinite() ) continue;
            gpos_ += b->getPosition();
            ++counter;
         }

         gpos_ /= counter;
      }
   }

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the moment of inertia in reference to the body frame of the union.
 *
 * \return void
 */
void UnionBase::calcInertia()
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );

   // Initializing the body moment of inertia and the inverse moment of inertia
   I_ = real(0);
   Iinv_.reset();

   // Don't calculate the moment of inertia of an infinite or empty union
   if( !finite_ || bodies_.isEmpty() || mass_ == real(0) ) return;

   // Calculating the global moment of inertia
   real mass;
   Vec3 pos;

   for( ConstIterator b=bodies_.begin(); b!=bodies_.end(); ++b )
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
   if( !fixed_ ) Iinv_ = I_.getInverse();

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************

} // namespace pe
