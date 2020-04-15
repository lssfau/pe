//=================================================================================================
/*!
 *  \file pe/core/rigidbody/planetrait/Default.h
 *  \brief Header file for the default implementation of the PlaneTrait class template.
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

#ifndef _PE_CORE_RIGIDBODY_PLANETRAIT_DEFAULT_H_
#define _PE_CORE_RIGIDBODY_PLANETRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/PlaneBase.h>
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
/*!\brief Plane customization class for the collision response.
 * \ingroup plane
 *
 * The PlaneTrait class template is a customization class for the plane geometry.
 * Its main purpose is the customization of the Plane class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class PlaneTrait : public PlaneBase
{
protected:
   //**Type definitions****************************************************************************
   typedef PlaneBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit PlaneTrait( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal,
                        real d, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~PlaneTrait() = 0;
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
/*!\brief Default implementation of the PlaneTrait constructor.
 *
 * \param sid Unique system-specific ID for the plane.
 * \param uid User-specific ID for the plane.
 * \param gpos The global position (anchor point) of the plane.
 * \param normal The plane's normal in reference to the global world frame, \f$ |n| = 1 \f$.
 * \param d The displacement of the plane.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 */
template< typename C >  // Type of the configuration
PlaneTrait<C>::PlaneTrait( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal,
                           real d, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, normal, d, material, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the PlaneTrait destructor.
 */
template< typename C >  // Type of the configuration
PlaneTrait<C>::~PlaneTrait()
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
 * Calculating one single time step of size \a dt for the plane. The global position, the
 * linear and angular velocity and the orientation of the plane are changed depending on
 * the acting forces and the current velocities.
 */
template< typename C >  // Type of the configuration
void PlaneTrait<C>::move( real dt )
{
   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Note: Resetting the contact nodes and removing all attached contacts is not necessary here

   // Move the plane
   if( awake_ ) {
      // Note: Applied forces, torques or gravity never have an influence on planes

      // Calculating the translational displacement
      gpos_ += v_ * dt;

      // Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      q_ = Quat( phi, phi.length() ) * q_;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Updating secondary plane description (normal and displacement)
      normal_ = q_.rotate( Vec3( 0, 0, 1 ) );
      d_ = trans(normal_) * gpos_;

      // Damping the movement
      if( Settings::damping() < real(1) ) {
         const real drag( std::pow( Settings::damping(), dt ) );
         v_ *= drag;
         w_ *= drag;
      }

      // Setting the axis-aligned bounding box
      PlaneBase::calcBoundingBox();

      // Calculating the current motion of the box
      calcMotion();
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the plane
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid plane state detected" );
}
//*************************************************************************************************

} // namespace pe   

#endif
