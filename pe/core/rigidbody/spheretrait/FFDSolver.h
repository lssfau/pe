//=================================================================================================
/*!
 *  \file pe/core/rigidbody/spheretrait/FFDSolver.h
 *  \brief Specialization of the SphereTrait class template for the fast frictional dynamics solver
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

#ifndef _PE_CORE_RIGIDBODY_SPHERETRAIT_FFDSOLVER_H_
#define _PE_CORE_RIGIDBODY_SPHERETRAIT_FFDSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/spheretrait/Default.h>
#include <pe/core/response/Types.h>
#include <pe/core/rigidbody/SphereBase.h>
#include <pe/core/Settings.h>
#include <pe/core/Types.h>
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
/*!\brief Specialization of the SphereTrait class template for the fast frictional dynamics solver.
 * \ingroup sphere
 *
 * This specialization of the SphereTrait class template adapts the sphere geometry to the
 * requirements of the fast frictional dynamics solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class SphereTrait< C<CD,FD,BG,response::FFDSolver> > : public SphereBase
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
/*!\brief Constructor for the FFDSolver specialization of the SphereTrait class template.
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
SphereTrait< C<CD,FD,BG,response::FFDSolver> >::SphereTrait( id_t sid, id_t uid, const Vec3& gpos,
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
/*!\brief Destructor for the FFDSolver specialization of the SphereTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
SphereTrait< C<CD,FD,BG,response::FFDSolver> >::~SphereTrait()
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
 * This function performs the first position half step of size \a dt. This includes an update
 * of the global position of the sphere according to the current linear velocity, an update of
 * the orientation/rotation of the sphere according to the current angular velocity and a
 * recalculation of the axis-aligned bounding box.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void SphereTrait< C<CD,FD,BG,response::FFDSolver> >::firstPositionHalfStep( real dt )
{
   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants()      , "Invalid sphere state detected"       );
   pe_INTERNAL_ASSERT( !hasSuperBody()        , "Invalid superordinate body detected" );
   pe_INTERNAL_ASSERT( !hasContacts()         , "Invalid contacts detected"           );
   pe_INTERNAL_ASSERT( !this->hasConstraints(), "Invalid constraints detected"        );

   // Position update of the sphere according to the current velocity
   // (don't update a sleeping, fixed, or remote sphere)
   if( awake_ && !fixed_ && !remote_ )
   {
      //Calculating the translational displacement
      gpos_ += v_ * dt;

      //Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      q_ = Quat( phi, phi.length() ) * q_;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Setting the axis-aligned bounding box
      SphereBase::calcBoundingBox();
   }

   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the second position half step of size \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function performs the second position half step of size \a dt. The includes an update
 * of the global position of the sphere according to the current linear velocity, an update of
 * the orientation/rotation of the sphere according to the current angular velocity and a
 * recalculation of the axis-aligned bounding box. This function also removes all currently
 * acting external forces.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void SphereTrait< C<CD,FD,BG,response::FFDSolver> >::secondPositionHalfStep( real dt )
{
   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected"       );
   pe_INTERNAL_ASSERT( !hasSuperBody()  , "Invalid superordinate body detected" );

   // Removing all attached contacts
   contacts_.clear();

   // Resetting the contact constraints
   this->resetConstraints();

   // Position update of the sphere according to the current velocity
   // (don't update a sleeping , fixed, or remote sphere)
   if( awake_ && !fixed_ && !remote_ )
   {
      //Calculating the translational displacement
      gpos_ += v_ * dt;

      //Calculating the rotation angle
      const Vec3 phi( w_ * dt );

      // Calculating the new orientation
      q_ = Quat( phi, phi.length() ) * q_;
      R_ = q_.toRotationMatrix();
      pe_INTERNAL_ASSERT( equal( R_.getDeterminant(), real(1) ), "Corrupted rotation matrix determinant" );

      // Setting the axis-aligned bounding box
      SphereBase::calcBoundingBox();
   }

   // Resetting the acting forces
   if( Settings::forceReset() || remote_ )
      RigidBody::resetForce();

   // Checking the state of the sphere
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid sphere state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
