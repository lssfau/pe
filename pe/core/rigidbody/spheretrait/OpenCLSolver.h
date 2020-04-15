//=================================================================================================
/*!
 *  \file pe/core/rigidbody/spheretrait/OpenCLSolver.h
 *  \brief Specialization of the SphereTrait class template for the OpenCL solver.
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

#ifndef _PE_CORE_RIGIDBODY_SPHERETRAIT_OPENCLSOLVER_H_
#define _PE_CORE_RIGIDBODY_SPHERETRAIT_OPENCLSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/SphereBase.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the SphereTrait class template for the OpenCL solver.
 * \ingroup sphere
 *
 * This specialization of the SphereTrait class template adapts the sphere geometry to the
 * requirements of the OpenCL solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class SphereTrait< C<CD,FD,BG,response::OpenCLSolver> > : public SphereBase
{
protected:
   //**Type definitions****************************************************************************
   typedef SphereBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit SphereTrait( size_t sid, size_t uid, const Vec3& gpos,
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
/*!\brief Constructor for the OpenCLSolver specialization of the SphereTrait class template.
 *
 * \param sid Unique system-specific ID for the sphere.
 * \param uid User-specific ID for the sphere.
 * \param gpos Global geometric center of the sphere.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
SphereTrait< C<CD,FD,BG,response::OpenCLSolver> >::SphereTrait( size_t sid, size_t uid, const Vec3& gpos,
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
/*!\brief Destructor for the OpenCLSolver specialization of the SphereTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
SphereTrait< C<CD,FD,BG,response::OpenCLSolver> >::~SphereTrait()
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
void SphereTrait< C<CD,FD,BG,response::OpenCLSolver> >::integrateVelocity( real dt )
{
   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected"       );
   pe_INTERNAL_ASSERT( !hasSuperBody()  , "Invalid superordinate body detected" );

   // Checking the moment of inertia
   pe_INTERNAL_ASSERT( I_.isDiagonal()   , "Non-diagonal inertia tensor detected"  );
   pe_INTERNAL_ASSERT( Iinv_.isDiagonal(), "Non-diagonal inverse inertia detected" );
   pe_INTERNAL_ASSERT( I_[0]==I_[4] && I_[4]==I_[8], "Invalid inertia detected"    );

   // Accelerate the sphere
   if( awake_ && !fixed_ )
   {
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
      pe_LOG_DEBUG_SECTION( log ) {
         log << "vdot " << vdot << "\n";
      }
   }
   else {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "sleeping or fixed\n";
      }
   }

   // Resetting the acting forces
   if( Settings::forceReset() )
      RigidBody::resetForce();

   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
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
void SphereTrait< C<CD,FD,BG,response::OpenCLSolver> >::integratePosition( real dt )
{
   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid object state detected"       );
   pe_INTERNAL_ASSERT( !hasSuperBody()  , "Invalid superordinate body detected" );

   // Checking the moment of inertia
   pe_INTERNAL_ASSERT( I_.isDiagonal()   , "Non-diagonal inertia tensor detected"  );
   pe_INTERNAL_ASSERT( Iinv_.isDiagonal(), "Non-diagonal inverse inertia detected" );
   pe_INTERNAL_ASSERT( I_[0]==I_[4] && I_[4]==I_[8], "Invalid inertia detected"    );

   // Resetting the contact node and removing all attached contacts
   resetNode();
   contacts_.clear();

   // Move the sphere
   if( awake_ )
   {
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

   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid object state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
