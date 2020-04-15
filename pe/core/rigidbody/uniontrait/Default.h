//=================================================================================================
/*!
 *  \file pe/core/rigidbody/uniontrait/Default.h
 *  \brief Header file for the default implementation of the SphereTrait class template.
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

#ifndef _PE_CORE_RIGIDBODY_UNIONTRAIT_DEFAULT_H_
#define _PE_CORE_RIGIDBODY_UNIONTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/UnionBase.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Union customization class for the collision response.
 * \ingroup union
 *
 * The UnionTrait class template is a customization class for the union compound geometry.
 * Its main purpose is the customization of the Union class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class UnionTrait : public UnionBase
{
protected:
   //**Type definitions****************************************************************************
   typedef UnionBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit UnionTrait( id_t sid, id_t uid, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~UnionTrait() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void move( real dt );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the UnionTrait constructor.
 *
 * \param sid Unique system-specific ID for the union.
 * \param uid User-specific ID for the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 */
template< typename C >  // Type of the configuration
UnionTrait<C>::UnionTrait( id_t sid, id_t uid, bool visible )
   : Parent( sid, uid, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the UnionTrait destructor.
 */
template< typename C >  // Type of the configuration
UnionTrait<C>::~UnionTrait()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of a time step of \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * Calculating one single time step of size \a dt for the union. The global position, the
 * linear and angular velocity and the orientation of the union are changed depending on
 * the acting forces and the current velocities.
 */
template< typename C >  // Type of the configuration
void UnionTrait<C>::move( real dt )
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Moving the union according to the acting forces (don't move a sleeping union)
   if( awake_ ) {
      if( !fixed_ ) {
         // Calculating the linear acceleration by the equation
         //   force * m^(-1) + gravity
         const Vec3 vdot( force_ * invMass_ + Settings::gravity() );

         // Calculating the angular acceleration by the equation
         //   R * Iinv * R^T * torque
         // This calculation neglects any inertia changes changes due to the rotation of the capsule,
         // which would result in the equation R * Iinv * R^T * ( torque - w % ( R * I * R^T * w ) ).
         // Additionally, this calculation uses the assumption that the inertia tensor as well as the
         // inverse inertia tensor of the union are full matrices of the form
         //                            ( Ixx Ixy Ixz )
         //                            ( Iyx Iyy Iyz )
         //                            ( Izx Izy Izz )
         const Vec3 wdot( R_ * ( Iinv_ * ( trans(R_) * torque_ ) ) );

         // Calculating the forces and torques in each link
         for( LinkIterator l=links_.begin(); l!=links_.end(); ++l ) {
            l->calcForce( vdot, wdot );
         }

         // Updating the linear velocity
         v_ += vdot * dt;

         // Updating the angular velocity
         w_ += wdot * dt;
      }

      // Calculating the translational displacement
      gpos_ += v_ * dt;

      // Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      const Quat dq( phi, phi.length() );
      q_ = dq * q_;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Damping the movement
      if( Settings::damping() < real(1) ) {
         const real drag( std::pow( Settings::damping(), dt ) );
         v_ *= drag;
         w_ *= drag;
      }

      // Updating the contained bodies
      for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
         updateBody( *b, dq );

      // Updating the links
      for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
         l->update( dq );

      // Setting the axis-aligned bounding box
      UnionBase::calcBoundingBox();

      // Calculating the current motion of the capsule
      calcMotion();
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      UnionBase::resetForce();

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************

} // namespace pe   

#endif
