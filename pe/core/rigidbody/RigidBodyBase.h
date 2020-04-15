//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyBase.h
 *  \brief Header file for the RigidBodyBase class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODYBASE_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODYBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/batches/BodyTrait.h>
#include <pe/core/Configuration.h>
#include <pe/core/detection/coarse/BodyTrait.h>
#include <pe/core/detection/fine/BodyTrait.h>
#include <pe/core/response/BodyTrait.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Types.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/Vector3.h>
#include <pe/system/BodyExtension.h>
#include <pe/system/Precision.h>
#include <pe/system/SleepMode.h>
#include <pe/util/NonCopyable.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Basic rigid body properties.
 * \ingroup rigid_body
 *
 * The RigidBodyBase class represents the basic properties of rigid bodies.
 */
class RigidBodyBase : public detection::coarse::BodyTrait<Config>
               , public detection::fine::BodyTrait<Config>
               , public batches::BodyTrait<Config>
               , public response::BodyTrait<Config>
               , public BodyExtension
               , private NonCopyable
{
protected:
   //**Type definitions****************************************************************************
   //! Abbreviation for the coarse collision detection body trait base class.
   /*! This base class configures the type of the axis-aligned bounding box of the rigid body. */
   typedef detection::coarse::BodyTrait<Config>  CCDBT;

   //! Abbreviation for the fine collision detection body trait base class.
   /*! This base class configures rigid bodies for the requirements of the selected fine
       collision detection algorithm. */
   typedef detection::fine::BodyTrait<Config>  FCDBT;

   //! Abbreviation for the batch generation body trait base class.
   /*! This base class configures rigid bodies for the requirements of the selected batch
       generation algorithm. */
   typedef batches::BodyTrait<Config>  BGBT;

   //! Abbreviation for the collision response body trait base class.
   /*! This base class adapts the rigid body to the requirements of the selected collision
       response algorithm. */
   typedef response::BodyTrait<Config>  CRBT;

   //! Abbreviation for the user-specific body extension base class.
   /*! This base class can be used to adapt rigid bodies to user-specific requirements. */
   typedef BodyExtension  BE;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit RigidBodyBase( BodyID body );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~RigidBodyBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Sleep mode functions************************************************************************
   /*!\name Sleep mode functions */
   //@{
   inline void wake();
   //@}
   //**********************************************************************************************

protected:

   //**Sleep mode functions************************************************************************
   /*!\name Sleep mode functions */
   //@{
   inline void calcMotion();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool fixed_;      //!< Fixation flag.
                     /*!< The flag value indicates if the rigid body is fixed/stationary
                          (fixed = \a true). */
   bool awake_;      //!< Sleep mode flag.
                     /*!< The flag value indicates if the rigid body is currently awake
                          (\a true) or in sleep mode (\a false). */
   real mass_;       //!< The total mass of the rigid body.
   real invMass_;    //!< The inverse total mass of the rigid body.
   real motion_;     //!< The current motion of the rigid body.
                     /*!< If this value drops under the specified sleep threshold, the
                          rigid body will be put to sleep. */
   Vec3 gpos_;       //!< The global position of the center of mass of this rigid body.
   Vec3 rpos_;       //!< The relative position within the body frame of the superordinate body.
                     /*!< If the body is contained within a superordinate Union the relative
                          position gives the position of the body's center of mass within the
                          body frame of the superordinate body. If the body is not contained
                          in a Union, the relative position is 0. */
   mutable Vec3 v_;  //!< The linear velocity of this rigid body.
   mutable Vec3 w_;  //!< Angular velocity of this rigid body.
   Vec3 force_;      //!< Total force (external+contact) acting in the body's center of mass.
   Vec3 torque_;     //!< Total torque (external+contact) acting in the body's center of mass.
   Mat3 I_;          //!< The moment of inertia in reference to the body's own body frame.
                     /*!< The moment of inertia quantifies the rotational inertia of a rigid
                          body, i.e. its inertia with respect to rotational motion, in a manner
                          somewhat analogous to how mass quantifies the inertia of a body with
                          respect to translational motion. The naming convention of the tensor
                          elements is
                                         \f[\left(\begin{array}{*{3}{c}}
                                         I_{xx} & I_{xy} & I_{xz} \\
                                         I_{yx} & I_{yy} & I_{yz} \\
                                         I_{zx} & I_{zy} & I_{zz} \\
                                         \end{array}\right)\f] */
   Mat3 Iinv_;       //!< The inverse moment of inertia within the body frame.
   Quat q_;          //!< The orientation of the body frame in the global world frame.
   Rot3 R_;          //!< The rotation in reference to the global frame of reference.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  SLEEP MODE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Waking the rigid body and ending the sleep mode.
 *
 * \return void
 *
 * This function wakes a rigid body from sleep mode. Note that this function has no effect if
 * it is called on a subordinate rigid body, i.e. a body contained in another rigid body.
 */
inline void RigidBodyBase::wake()
{
   awake_ = true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the current motion of a rigid body.
 *
 * \return void
 *
 * This function calculates the current motion of a rigid body. The motion is a scalar value,
 * consisting of the current linear and angular velocity and is calculated by the following
 * equation:

   \f[ \Phi = \mbox{bias} \cdot \Phi + (1-\mbox{bias}) \cdot ( \vec{v}^2 + \vec{w}^2 ), \f]

 * where \f$ \Phi \f$ is the motion, \f$ \vec{v} \f$ is the linear velocity, \f$ \vec{w} \f$
 * is the angular velocity and \a bias is the weighting factor for the recency-weighted average
 * between the new motion value and the motion value from the last time frame (see pe::sleepBias).
 * If the motion drops below pe::sleepThreshold, the rigid body is put to sleep.
 */
inline void RigidBodyBase::calcMotion()
{
   pe_INTERNAL_ASSERT( sleepThreshold >= real(0), "Invalid sleepThreshold value" );
   pe_INTERNAL_ASSERT( sleepBias >= real(0) && sleepBias <= real(1), "Invalid sleepBias value" );

   // Calculating the current motion of the rigid body (recency-weighted average)
   motion_ = ( sleepBias * motion_ ) + ( real(1) - sleepBias )*( v_.sqrLength() + w_.sqrLength() );

   // Activating the sleep mode of the rigid body
   if( motion_ < sleepThreshold ) {
      awake_ = false;
      v_.reset();
      w_.reset();
   }
}
//*************************************************************************************************

} // namespace pe

#endif
