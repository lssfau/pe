//=================================================================================================
/*!
 *  \file pe/core/rigidbody/cylindertrait/FFDSolver.h
 *  \brief Specialization of the CylinderTrait class template for the fast frictional dynamics solver
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

#ifndef _PE_CORE_RIGIDBODY_CYLINDERTRAIT_FFDSOLVER_H_
#define _PE_CORE_RIGIDBODY_CYLINDERTRAIT_FFDSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/CylinderBase.h>
#include <pe/core/rigidbody/cylindertrait/Default.h>
#include <pe/core/response/Types.h>
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
/*!\brief Specialization of the CylinderTrait class template for the fast frictional dynamics solver.
 * \ingroup cylinder
 *
 * This specialization of the CylinderTrait class template adapts the cylinder geometry to the
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
class CylinderTrait< C<CD,FD,BG,response::FFDSolver> > : public CylinderBase
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
/*!\brief Constructor for the FFDSolver specialization of the CylinderTrait class template.
 *
 * \param sid Unique system-specific ID for the cylinder.
 * \param uid User-specific ID for the cylinder.
 * \param gpos Global geometric center of the cylinder.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
CylinderTrait< C<CD,FD,BG,response::FFDSolver> >::CylinderTrait( id_t sid, id_t uid, const Vec3& gpos, real radius,
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
/*!\brief Destructor for the FFDSolver specialization of the CylinderTrait class template.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
CylinderTrait< C<CD,FD,BG,response::FFDSolver> >::~CylinderTrait()
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
 * of the global position of the cylinder according to the current linear velocity, an update of
 * the orientation/rotation of the cylinder according to the current angular velocity and a
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
void CylinderTrait< C<CD,FD,BG,response::FFDSolver> >::firstPositionHalfStep( real dt )
{
   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants()      , "Invalid cylinder state detected"     );
   pe_INTERNAL_ASSERT( !hasSuperBody()        , "Invalid superordinate body detected" );
   pe_INTERNAL_ASSERT( !hasContacts()         , "Invalid contacts detected"           );
   pe_INTERNAL_ASSERT( !this->hasConstraints(), "Invalid constraints detected"        );

   // Position update of the cylinder according to the current velocity
   // (don't update a sleeping, fixed, or remote cylinder)
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
      CylinderBase::calcBoundingBox();
   }

   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the second position half step of size \a dt.
 *
 * \param dt Time step size.
 * \return void
 *
 * This function performs the second position half step of size \a dt. The includes an update
 * of the global position of the cylinder according to the current linear velocity, an update
 * of the orientation/rotation of the cylinder according to the current angular velocity and
 * a recalculation of the axis-aligned bounding box. This function also removes all currently
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
void CylinderTrait< C<CD,FD,BG,response::FFDSolver> >::secondPositionHalfStep( real dt )
{
   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected"     );
   pe_INTERNAL_ASSERT( !hasSuperBody()  , "Invalid superordinate body detected" );

   // Removing all attached contacts
   contacts_.clear();

   // Resetting the contact constraints
   this->resetConstraints();

   // Position update of the cylinder according to the current velocity
   // (don't update a sleeping, fixed, or remote cylinder)
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
      CylinderBase::calcBoundingBox();
   }

   // Resetting the acting forces
   if( Settings::forceReset() || remote_ )
      RigidBody::resetForce();

   // Checking the state of the cylinder
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid cylinder state detected" );
}
//*************************************************************************************************

} // namespace pe

#endif
