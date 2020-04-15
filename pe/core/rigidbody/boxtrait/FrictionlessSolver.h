//=================================================================================================
/*!
 *  \file pe/core/rigidbody/boxtrait/FrictionlessSolver.h
 *  \brief Specialization of the BoxTrait class template for the frictionless solver
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

#ifndef _PE_CORE_RIGIDBODY_BOXTRAIT_FRICTIONLESSSOLVER_H_
#define _PE_CORE_RIGIDBODY_BOXTRAIT_FRICTIONLESSSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/BoxBase.h>
#include <pe/core/rigidbody/boxtrait/Default.h>
#include <pe/core/response/Types.h>
#include <pe/core/Settings.h>
#include <pe/core/Types.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/shims/Equal.h>
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
/*!\brief Specialization of the BoxTrait class template for the frictionless solver.
 * \ingroup box
 *
 * This specialization of the BoxTrait class template adapts the box geometry to the
 * requirements of the frictionless solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class BoxTrait< C<CD,FD,BG,response::FrictionlessSolver> > : public BoxBase
{
protected:
   //**Type definitions****************************************************************************
   typedef BoxBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit BoxTrait( id_t sid, id_t uid, const Vec3& gpos,
                      const Vec3& lengths, MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BoxTrait() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void move( real dt );  // Performing a time step of size dt
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
/*!\brief Constructor for the FrictionlessSolver specialization of the BoxTrait class template.
 *
 * \param sid Unique system-specific ID for the box.
 * \param uid User-specific ID for the box.
 * \param gpos Global geometric center of the box.
 * \param lengths Side lengths of the box \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
BoxTrait< C<CD,FD,BG,response::FrictionlessSolver> >::BoxTrait( id_t sid, id_t uid, const Vec3& gpos,
                                                                const Vec3& lengths, MaterialID material, bool visible )
   : Parent( sid, uid, gpos, lengths, material, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the FrictionlessSolver specialization of the BoxTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
BoxTrait< C<CD,FD,BG,response::FrictionlessSolver> >::~BoxTrait()
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
 * Calculating one single time step of size \a dt for the box. The global position, the
 * linear and angular velocity and the orientation of the box are changed depending on
 * the acting forces and the current velocities.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void BoxTrait< C<CD,FD,BG,response::FrictionlessSolver> >::move( real dt )
{
   // Checking the state of the object
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid object state detected" );
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superordinate body detected" );

   // Checking the moment of inertia
   pe_INTERNAL_ASSERT( I_.isDiagonal()   , "Non-diagonal inertia tensor detected"  );
   pe_INTERNAL_ASSERT( Iinv_.isDiagonal(), "Non-diagonal inverse inertia detected" );

   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Moving the object according to the acting forces unless object is sleeping
   if( awake_ ) {
      Vec3 phi;

      if( fixed_ ) {
         // Calculating the translational displacement
         gpos_ += v_ * dt;

         // Calculating the rotation angle
         phi = w_ * dt;
      }
      else {
         // Calculating the linear acceleration by the equation
         //   force * m^(-1) + gravity
         const Vec3 vdot( force_ * invMass_ + Settings::gravity() );

         // Calculating the angular acceleration by the equation
         //   R * Iinv * R^T * torque
         // This calculation neglects any inertia changes changes due to the rotation of the box,
         // which would result in the equation R * Iinv * R^T * ( torque - w % ( R * I * R^T * w ) ).
         // Additionally, this calculation uses the assumption that the inertia tensor as well as the
         // inverse inertia tensor of the box are diagonal matrices of the form
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

         // Calculating the translational displacement
         gpos_ += v_ * dt;

         // Calculating the rotation angle
         phi = w_ * dt;
      }

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
      BoxBase::calcBoundingBox();

      // Calculating the current motion of the object
      calcMotion();
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the object
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid object state detected" );
}
//*************************************************************************************************

} // namespace pe   

#endif
