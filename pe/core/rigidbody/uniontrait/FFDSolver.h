//=================================================================================================
/*!
 *  \file pe/core/rigidbody/uniontrait/FFDSolver.h
 *  \brief Specialization of the UnionTrait class template for the fast frictional dynamics solver
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

#ifndef _PE_CORE_RIGIDBODY_UNIONTRAIT_FFDSOLVER_H_
#define _PE_CORE_RIGIDBODY_UNIONTRAIT_FFDSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/uniontrait/Default.h>
#include <pe/core/response/Types.h>
#include <pe/core/Settings.h>
#include <pe/core/Types.h>
#include <pe/core/rigidbody/UnionBase.h>
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
/*!\brief Specialization of the UnionTrait class template for the fast frictional dynamics solver.
 * \ingroup union
 *
 * This specialization of the UnionTrait class template adapts the union compound geometry to
 * the requirements of the fast frictional dynamics solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class UnionTrait< C<CD,FD,BG,response::FFDSolver> > : public UnionBase
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
   virtual void firstPositionHalfStep ( real dt );
   virtual void secondPositionHalfStep( real dt );
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
/*!\brief Constructor for the FFDSolver specialization of the UnionTrait class template.
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
UnionTrait< C<CD,FD,BG,response::FFDSolver> >::UnionTrait( id_t sid, id_t uid, bool visible )
   : Parent( sid, uid, visible )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the FFDSolver specialization of the UnionTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
UnionTrait< C<CD,FD,BG,response::FFDSolver> >::~UnionTrait()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the first position half step of size \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function performs the first position half step of size \a dt. The global position, the
 * orientation/rotation of the union and all contained bodies and links are changed depending
 * on the current linear and angular velocity.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void UnionTrait< C<CD,FD,BG,response::FFDSolver> >::firstPositionHalfStep( real dt )
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants()      , "Invalid union state detected"        );
   pe_INTERNAL_ASSERT( !hasSuperBody()        , "Invalid superordinate body detected" );
   pe_INTERNAL_ASSERT( !hasContacts()         , "Invalid contacts detected"           );
   pe_INTERNAL_ASSERT( !this->hasConstraints(), "Invalid constraints detected"        );

   // Position update of the union according to the current velocity
   // (don't update a sleeping, fixed, or remote union)
   if( awake_ && !fixed_ && !remote_ )
   {
      //Calculating the translational displacement
      gpos_ += v_ * dt;

      //Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      // The calculation of the new orientation is performed such that the contained rigid bodies
      // are updated in exactly the same manner as during an update via the setOrientation()
      // function. This removes discrepancies in the orientation of the contained bodies in MPI
      // parallel simulations.
      const Quat dq1( phi, phi.length() );      // The change in orientation due to angular velocity
      const Quat tmp( dq1 * q_ );               // The new final orientation of the union
      const Quat dq2( tmp * q_.getInverse() );  // The change between the old and new orientation
      q_ = tmp;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Updating the contained bodies
      for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
         updateBody( *b, dq2 );

      // Updating the links
      for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
         l->update( dq2 );

      // Setting the axis-aligned bounding box
      UnionBase::calcBoundingBox();
   }

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the second position half step of size \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function performs the second position half step of size \a dt. The global position, the
 * orientation/rotation of the union and all contained bodies and links are changed depending
 * on the current linear and angular velocity. This function also removes all currently acting
 * external forces.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void UnionTrait< C<CD,FD,BG,response::FFDSolver> >::secondPositionHalfStep( real dt )
{
   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected"        );
   pe_INTERNAL_ASSERT( !hasSuperBody()  , "Invalid superordinate body detected" );

   // Removing all attached contacts
   contacts_.clear();

   // Resetting the contact constraints
   this->resetConstraints();

   // Position update of the union according to the current velocity
   // (don't update a sleeping, fixed, or remote union)
   if( awake_ && !fixed_ && !remote_ )
   {
      //Calculating the translational displacement
      gpos_ += v_ * dt;

      //Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      // The calculation of the new orientation is performed such that the contained rigid bodies
      // are updated in exactly the same manner as during an update via the setOrientation()
      // function. This removes discrepancies in the orientation of the contained bodies in MPI
      // parallel simulations.
      const Quat dq1( phi, phi.length() );      // The change in orientation due to angular velocity
      const Quat tmp( dq1 * q_ );               // The new final orientation of the union
      const Quat dq2( tmp * q_.getInverse() );  // The change between the old and new orientation
      q_ = tmp;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Updating the contained bodies
      for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
         updateBody( *b, dq2 );

      // Updating the links
      for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
         l->update( dq2 );

      // Setting the axis-aligned bounding box
      UnionBase::calcBoundingBox();
   }

   // Resetting the acting forces
   if( Settings::forceReset() || remote_ )
      UnionBase::resetForce();

   // Checking the state of the union
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid union state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
