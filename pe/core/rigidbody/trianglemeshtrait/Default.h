//=================================================================================================
/*!
 *  \file pe/core/rigidbody/trianglemeshtrait/Default.h
 *  \brief Header file for the default implementation of the TriangleMeshTrait class template.
 *
 *  Copyright (C) 2013-2014 Tobias Scharpff
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

#ifndef _PE_CORE_RIGIDBODY_TRIANGLEMESHTRAIT_DEFAULT_H_
#define _PE_CORE_RIGIDBODY_TRIANGLEMESHTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/TriangleMeshBase.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Triangle mesh customization class for the collision response.
 * \ingroup triangleMesh
 *
 * The TriangleMeshBaseTrait class template is a customization class for the triangle mesh geometry. Its main
 * purpose is the customization of the triangle mesh class for the selected collision response
 * algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class TriangleMeshTrait : public TriangleMeshBase
{
protected:
   //**Type definitions****************************************************************************
   typedef TriangleMeshBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit TriangleMeshTrait( id_t sid, id_t uid, const Vec3& gpos,
                         const Vertices& vertices, const IndicesLists& faceIndices,
                         MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~TriangleMeshTrait() = 0;
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
/*!\brief Default implementation of the TriangleMeshTrait constructor.
 *
 * \param sid Unique system-specific ID for the triangle mesh.
 * \param uid User-specific ID for the triangle mesh.
 * \param gpos Global geometric center of the triangle mesh.
 * \param vertices The list of vertices that build the surface
 * \param faceIndices The list of indices which assign three elements of vertices to one triangle
 * \param material The material of the triangle mesh.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 */
template< typename C >  // Type of the configuration
TriangleMeshTrait<C>::TriangleMeshTrait( id_t sid, id_t uid, const Vec3& gpos,
                                 const Vertices& vertices, const IndicesLists& faceIndices,
                                 MaterialID material, bool visible )
   : Parent( sid, uid, gpos, vertices, faceIndices, material, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the TriangleMeshTrait destructor.
 */
template< typename C >  // Type of the configuration
TriangleMeshTrait<C>::~TriangleMeshTrait()
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
 * Calculating one single time step of size \a dt for the triangle mesh. The global position, the
 * linear and angular velocity and the orientation of the triangle mesh are changed depending on
 * the acting forces and the current velocities.
 */
template< typename C >  // Type of the configuration
void TriangleMeshTrait<C>::move( real dt )
{
   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );
   std::cout << "foo" << std::endl;//TODO
   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Moving the triangle mesh according to the acting forces (don't move a sleeping triangle mesh)
   if( awake_ ) {
      if( !fixed_ ) {
         // Calculating the linear acceleration by the equation
         //   force * m^(-1) + gravity
         const Vec3 vdot( force_ * invMass_ + Settings::gravity() );

         // Calculating the angular acceleration by the equation
         //   R * Iinv * R^T * torque
         // This calculation neglects any inertia changes changes due to the rotation of the triangle mesh,
         // which would result in the equation R * Iinv * R^T * ( torque - w % ( R * I * R^T * w ) ).
         // Additionally, this calculation uses the assumption that the inertia tensor as well as the
         // inverse inertia tensor of the triangle mesh are full matrices of the form
         //                            ( Ixx Ixy Ixz )
         //                            ( Iyx Iyy Iyz )
         //                            ( Izx Izy Izz )
         const Vec3 wdot( R_ * ( Iinv_ * ( trans(R_) * torque_ ) ) );

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
      TriangleMeshBase::calcBoundingBox();

      // Calculating the current motion of the triangle mesh
      calcMotion();
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
