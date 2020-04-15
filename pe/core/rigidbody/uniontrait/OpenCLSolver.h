//=================================================================================================
/*!
 *  \file pe/core/rigidbody/uniontrait/OpenCLSolver.h
 *  \brief Specialization of the UnionTrait class template for the OpenCL solver
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

#ifndef _PE_CORE_RIGIDBODY_UNIONTRAIT_OPENCLSOLVER_H_
#define _PE_CORE_RIGIDBODY_UNIONTRAIT_OPENCLSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/response/Types.h>
#include <pe/core/Settings.h>
#include <pe/core/rigidbody/UnionBase.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/Invert.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the UnionTrait class template for the OpenCL solver.
 * \ingroup union
 *
 * This specialization of the SphereTrait class template adapts the union compound geometry to
 * the requirements of the OpenCL solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class UnionTrait< C<CD,FD,BG,response::OpenCLSolver> > : public UnionBase
{
protected:
   //**Type definitions****************************************************************************
   typedef UnionBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit UnionTrait( size_t sid, size_t uid, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~UnionTrait() = 0;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 vold_;  //!< The linear velocity before collision resolution.
   Vec3 wold_;  //!< The angular velocity before collision resolution.
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void integrateVelocity( real dt );
   virtual void integratePosition( real dt );
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
/*!\brief Constructor for the OpenCL specialization of the UnionTrait class template.
 *
 * \param sid Unique system-specific ID for the union.
 * \param uid User-specific ID for the union.
 * \param visible Specifies if the entire union is visible in a visualization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
UnionTrait< C<CD,FD,BG,response::OpenCLSolver> >::UnionTrait( size_t sid, size_t uid, bool visible )
   : Parent( sid, uid, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the OpenCLSolver specialization of the UnionTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
UnionTrait< C<CD,FD,BG,response::OpenCLSolver> >::~UnionTrait()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Integrate the body velocity by a time step of \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * The linear and angular velocity is updated according to the current acting forces and
 * gravity. The forces are reset afterwards.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void UnionTrait< C<CD,FD,BG,response::OpenCLSolver> >::integrateVelocity( real dt )
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Accelerate the union
   if( awake_ && !fixed_ )
   {
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

      // Backup the current linear and angular velocities to be able to perform link force calculations later on
      vold_ = v_;
      wold_ = w_;

      // Updating the linear velocity
      v_ += vdot * dt;

      // Updating the angular velocity
      w_ += wdot * dt;
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      UnionBase::resetForce();

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Integrate the body position by a time step of \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * The global position and the orientation of the body is updated according to the current
 * body velocity. Additionally the contact graph data structures are cleared, velocity
 * damping is performed, bounding boxes are updated and motion decay is detected. 
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void UnionTrait< C<CD,FD,BG,response::OpenCLSolver> >::integratePosition( real dt )
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Move the union
   if( awake_ )
   {
      const real dtinv( inv( dt ) );

      // Calculating the forces and torques in each link
      for( LinkIterator l=links_.begin(); l!=links_.end(); ++l ) {
         l->calcForce( ( v_ - vold_ ) * dtinv, ( w_ - wold_ ) * dtinv );
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

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
