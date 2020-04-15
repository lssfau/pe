//=================================================================================================
/*!
 *  \file pe/core/rigidbody/spheretrait/Default.h
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

#ifndef _PE_CORE_RIGIDBODY_SPHERETRAIT_DEFAULT_H_
#define _PE_CORE_RIGIDBODY_SPHERETRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/SphereBase.h>
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
/*!\brief Sphere customization class for the collision response.
 * \ingroup sphere
 *
 * The SphereTrait class template is a customization class for the sphere geometry.
 * Its main purpose is the customization of the Sphere class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class SphereTrait : public SphereBase
{
protected:
   //**Type definitions****************************************************************************
   typedef SphereBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit SphereTrait( id_t sid, id_t uid, const Vec3& gpos,
                         real radius, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~SphereTrait() = 0;
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
/*!\brief Default implementation of the SphereTrait constructor.
 *
 * \param sid Unique system-specific ID for the sphere.
 * \param uid User-specific ID for the sphere.
 * \param gpos Global geometric center of the sphere.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 */
template< typename C >  // Type of the configuration
SphereTrait<C>::SphereTrait( id_t sid, id_t uid, const Vec3& gpos,
                             real radius, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, radius, material, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the SphereTrait destructor.
 */
template< typename C >  // Type of the configuration
SphereTrait<C>::~SphereTrait()
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
 * Calculating one single time step of size \a dt for the sphere. The global position, the
 * linear and angular velocity and the orientation of the sphere are changed depending on
 * the acting forces and the current velocities.
 */
template< typename C >  // Type of the configuration
void SphereTrait<C>::move( real dt )
{
   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected"       );
   pe_INTERNAL_ASSERT( !hasSuperBody()  , "Invalid superordinate body detected" );

   // Checking the moment of inertia
   pe_INTERNAL_ASSERT( I_.isDiagonal()   , "Non-diagonal inertia tensor detected"  );
   pe_INTERNAL_ASSERT( Iinv_.isDiagonal(), "Non-diagonal inverse inertia detected" );
   pe_INTERNAL_ASSERT( I_[0]==I_[4] && I_[4]==I_[8], "Invalid inertia detected"    );

   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Moving the sphere according to the acting forces (don't move a sleeping sphere)
   if( awake_ ) {
      if( !fixed_ ) {
         // Calculating the linear acceleration by the equation
         //   force * m^(-1) + gravity
         const Vec3 vdot( force_ * invMass_ + Settings::gravity() );

         // Calculating the angular acceleration by the equation
         //   R * Iinv * R^T * torque
         // This calculation neglects any inertia changes changes due to the rotation of the sphere,
         // which would result in the equation R * Iinv * R^T * ( torque - w % ( R * I * R^T * w ) ).
         // Additionally, this calculation uses the assumption that the inertia tensor as well as the
         // inverse inertia tensor of the sphere are scaled identity matrices of the form
         //                            ( I 0 0 )
         //                            ( 0 I 0 )
         //                            ( 0 0 I )
         const Vec3 wdot( Iinv_[0] * torque_ );

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
      q_ = Quat( phi, phi.length() ) * q_;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Damping the movement
      if( Settings::damping() < real(1) ) {
         const real drag( std::pow( Settings::damping(), dt ) );
         v_ *= drag;
         w_ *= drag;
      }

      // Setting the axis-aligned bounding box
      SphereBase::calcBoundingBox();

      // Calculating the current motion of the sphere
      calcMotion();
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
}
//*************************************************************************************************

} // namespace pe   

#endif
