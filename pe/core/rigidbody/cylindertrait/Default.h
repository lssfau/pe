//=================================================================================================
/*!
 *  \file pe/core/rigidbody/cylindertrait/Default.h
 *  \brief Header file for the default implementation of the CylinderTrait class template.
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

#ifndef _PE_CORE_RIGIDBODY_CYLINDERTRAIT_DEFAULT_H_
#define _PE_CORE_RIGIDBODY_CYLINDERTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/CylinderBase.h>
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
/*!\brief Cylinder customization class for the collision response.
 * \ingroup cylinder
 *
 * The CylinderTrait class template is a customization class for the cylinder geometry.
 * Its main purpose is the customization of the Cylinder class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class CylinderTrait : public CylinderBase
{
protected:
   //**Type definitions****************************************************************************
   typedef CylinderBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit CylinderTrait( id_t sid, id_t uid, const Vec3& gpos, real radius,
                           real length, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~CylinderTrait() = 0;
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
/*!\brief Default implementation of the CylinderTrait constructor.
 *
 * \param sid Unique system-specific ID for the cylinder.
 * \param uid User-specific ID for the cylinder.
 * \param gpos Global geometric center of the cylinder.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 */
template< typename C >  // Type of the configuration
CylinderTrait<C>::CylinderTrait( id_t sid, id_t uid, const Vec3& gpos, real radius,
                                 real length, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, radius, length, material, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the CylinderTrait destructor.
 */
template< typename C >  // Type of the configuration
CylinderTrait<C>::~CylinderTrait()
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
 * Calculating one single time step of size \a dt for the cylinder. The global position, the
 * linear and angular velocity and the orientation of the cylinder are changed depending on
 * the acting forces and the current velocities.
 */
template< typename C >  // Type of the configuration
void CylinderTrait<C>::move( real dt )
{
   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Checking the moment of inertia
   pe_INTERNAL_ASSERT( I_.isDiagonal()   , "Non-diagonal inertia tensor detected"  );
   pe_INTERNAL_ASSERT( Iinv_.isDiagonal(), "Non-diagonal inverse inertia detected" );

   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Moving the cylinder according to the acting forces (don't move a sleeping cylinder)
   if( awake_ ) {
      if( !fixed_ ) {
         // Calculating the linear acceleration by the equation
         //   force * m^(-1) + gravity
         const Vec3 vdot( force_ * invMass_ + Settings::gravity() );

         // Calculating the angular acceleration by the equation
         //   R * Iinv * R^T * torque
         // This calculation neglects any inertia changes changes due to the rotation of the cylinder,
         // which would result in the equation R * Iinv * R^T * ( torque - w % ( R * I * R^T * w ) ).
         // Additionally, this calculation uses the assumption that the inertia tensor as well as the
         // inverse inertia tensor of the cylinder are diagonal matrices of the form
         //                            ( Ix 0  0  )
         //                            ( 0  Iy 0  )
         //                            ( 0  0  Iz )
         const Vec3 tmp1( trans(R_) * torque_ );
         const Vec3 tmp2( Iinv_[0]*tmp1[0], Iinv_[4]*tmp1[1], Iinv_[8]*tmp1[2] );
         const Vec3 wdot( R_ * tmp2 );

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
      CylinderBase::calcBoundingBox();

      // Calculating the current motion of the cylinder
      calcMotion();
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
}
//*************************************************************************************************

} // namespace pe   

#endif
