//=================================================================================================
/*!
 *  \file pe/core/joint/FixedJoint.h
 *  \brief Header file for the FixedJoint class
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

#ifndef _PE_CORE_JOINT_FIXEDJOINT_H_
#define _PE_CORE_JOINT_FIXEDJOINT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/joint/Joint.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/TimeStep.h>
#include <pe/core/Types.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/Vector3.h>
#include <pe/math/VectorN.h>
#include <pe/math/Quaternion.h>
#include <pe/system/Precision.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup fixed_joint Fixed joint
 * \ingroup joints
 *
 * The fixed joint module combines all necessary functionality for the setup, use, and
 * destruction of fixed joints. A detailed description of the fixed joint can be found
 * with the class FixedJoint. This description also contains detailed examples for the
 * setup and destruction of a fixed joint.
 */
/*!\brief FixedJoint between two connected primitives.
 * \ingroup fixed_joint
 *
 * \section fixed_joint_general General
 *
 * A FixedJoint represents a currently active fixed joint between two rigid bodies in the
 * rigid body simulation. The class is derived from the Joint base class, which makes the
 * FixedJoint a motion constraint, which is considered during the collision resolution.\n
 * The fixed joint is the simplest possible joint and represents a fixed connection between
 * two rigid bodies (comparable with a solid rod in-between the two bodies). It therefore
 * constraints both the linear as well as the rotational movement of the two attached rigid
 * bodies.
 *
 * \image html fixedJoint.png
 * \image latex fixedJoint.eps "Fixed Joint" width=400pt
 *
 *
 * \section fixed_joint_setup Creating and destroying a fixed joint
 *
 * In order to create a fixed joint, one of the following fixed joint creation functions can
 * be used:
 *
 * - pe::attachFixedJoint( BodyID body1, BodyID body2, real scale );
 * - pe::attachFixedJoint( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real scale );
 *
 * In order to destroy a specific fixed joint, the following function can be used:
 *
 * - pe::detach( JointID joint )
 *
 * The following example demonstrates the creation and destruction of a fixed joint:

   \code
   // Creating the iron sphere 1 with a radius of 2.5 at the global position (2,3,4).
   SphereID sphere = createSphere( 1, Vec3(2,3,4), 2.5, iron );

   // Creating the granite box 2 with side lengths (6,5,4) at the global position (10,3,4)
   BoxID box = createBox( 2, Vec3(10,3,4), Vec3(6,5,4), granite );

   // Creating a fixed joint between the sphere and the box primitive
   // Per default, the centers of mass of both attached bodies are used as anchor points.
   // In all active visualizations, the scaling factor of 0.5 will be used to draw the
   // fixed joint. Note that the fixed joint is automatically added to the simulation
   // world and is immediately part of the entire simulation. The function returns a
   // handle to the newly created fixed joint, which can be used to for instance query
   // the anchor points of the joint.
   FixedJointID joint = attachFixedJoint( sphere, box, 0.5 );

   // Rigid body simulation
   // ...

   // Destroying (detaching) the fixed joint
   detach( joint );
   \endcode

 * Note that currently it is not possible to use fixed joints in combination with the
 * PolyhedralFrictionSolver or the FFDSolver (i.e., in MPI parallel simulations). The
 * attempt to create a fixed joint when any of these constraint solvers is selected
 * (see pe::pe_CONSTRAINT_SOLVER) results in a \a std::logic_error exception.
 */
class FixedJoint : public Joint
{
protected:
   //**Type definitions****************************************************************************
   typedef Joint  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit FixedJoint( id_t sid, BodyID body1, const Vec3& anchor1,
                        BodyID body2, const Vec3& anchor2, real scale );
   //@}
  //***********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~FixedJoint();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline        real  getCurLength() const;
   inline        real  getIniLength() const;
   inline  const Vec3& getAnchor1BF() const;
   inline  const Vec3  getAnchor1WF() const;
   inline  const Vec3& getAnchor2BF() const;
   inline  const Vec3  getAnchor2WF() const;
   inline  const Vec3  getMid1BF   () const;
   inline  const Vec3  getMid1WF   () const;
   inline  const Vec3  getMid2BF   () const;
   inline  const Vec3  getMid2WF   () const;
   inline  const Quat& getIniOrient() const;
   inline  Quat        getErrOrient() const;

   virtual real        getCorParam () const;
   //@}
   //**********************************************************************************************

   //**Calculation functions***********************************************************************
   /*!\name Calculation functions */
   //@{
   virtual void calcMat     ( MatN& JLin1, MatN& JAng1, MatN& JLin2, MatN& JAng2 ) const;
   virtual void calcErrorVec( VecN& bError )                                       const;
   //@}
   //**********************************************************************************************

   //**Limit functions*****************************************************************************
   /*!\name Limit functions */
   //@{
   inline bool isViolatedLow   ()  const;
   inline bool isViolatedHigh  ()  const;
          void calcLimitMat    ( MatN& JLin1L, MatN& JAng1L, MatN& JLin2L, MatN& JAng2L ) const;
   inline real calcLimitErrorLo()  const;
   inline real calcLimitErrorHi()  const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Member Variables****************************************************************************
   /*!\name Member variables */
   //@{
   real length_;   //!< The constant distance between the two rigid bodies \f$ (0..\infty) \f$.
   Vec3 anchor1_;  //!< The first body's anchor point in body relative coordinates.
   Vec3 anchor2_;  //!< The second body's anchor point in body relative coordinates.
   Vec3 mid1_;     //!< The midpoint between the two bodies wrt. the first body.
   Vec3 mid2_;     //!< The midpoint between the two bodies wrt. the second body.
   Quat qini_;     //!< The initial relative orientation of the two bodies.
   //@}
   //**********************************************************************************************

private:
   //**Fixed Joint setup functions*****************************************************************
   /*!\cond PE_INTERNAL */
   friend FixedJointID attachFixedJoint( BodyID body1, const Vec3& anchor1,
                                         BodyID body2, const Vec3& anchor2, real scale );
   /*!\endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current constant distance between the two rigid bodies.
 *
 * \return The current constant distance between the two rigid bodies.
 */
inline real FixedJoint::getCurLength() const
{
   const Vec3 dist( body2_->getPosition() - body1_->getPosition() );
   return dist.length();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the initial constant distance between the two rigid bodies.
 *
 * \return The initial constant distance between the two rigid bodies.
 */
inline real FixedJoint::getIniLength() const
{
   return length_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first body's anchor point in body relative coordinates.
 *
 * \return The first body's anchor point.
 *
 * This function returns the first body's anchor point in body relative coordinates.
 */
inline const Vec3& FixedJoint::getAnchor1BF() const
{
   return anchor1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first body's anchor point in global coordinates.
 *
 * \return The first body's anchor point.
 *
 * This function returns the first body's anchor point in global coordinates.
 */
inline const Vec3 FixedJoint::getAnchor1WF() const
{
   return body1_->pointFromBFtoWF( anchor1_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second body's anchor point in body relative coordinates.
 *
 * \return The second body's anchor point.
 *
 * This function returns the second body's anchor point in body relative coordinates.
 */
inline const Vec3& FixedJoint::getAnchor2BF() const
{
   return anchor2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second body's anchor point in global coordinates.
 *
 * \return The second body's anchor point.
 *
 * This function returns the second body's anchor point in global coordinates.
 */
inline const Vec3 FixedJoint::getAnchor2WF() const
{
   return body2_->pointFromBFtoWF( anchor2_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the midpoint between the two bodies in relative coordinates wrt. the first body.
 *
 * \return The midpoint between the two bodies wrt. the first body.
 *
 * This function returns the midpoint between the two bodies in body relative coordinates
 * wrt. the first body.
 */
inline const Vec3 FixedJoint::getMid1BF() const
{
   return mid1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the midpoint between the two bodies in global coordinates wrt. the first body.
 *
 * \return The midpoint between the two bodies wrt. the first body.
 *
 * This function returns the midpoint between the two bodies in global coordinates wrt. the
 * first body.
 */
inline const Vec3 FixedJoint::getMid1WF() const
{
   return body1_->pointFromBFtoWF( mid1_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the midpoint between the two bodies in relative coordinates wrt. the second body.
 *
 * \return The midpoint between the two bodies wrt. the second body.
 *
 * This function returns the midpoint between the two bodies in body relative coordinates
 * wrt. the second body.
 */
inline const Vec3 FixedJoint::getMid2BF() const
{
   return mid2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the midpoint between the two bodies in global coordinates wrt. the second body.
 *
 * \return The midpoint between the two bodies wrt. the second body.
 *
 * This function returns the midpoint between the two bodies in global coordinates wrt. the
 * second body.
 */
inline const Vec3 FixedJoint::getMid2WF() const
{
   return body2_->pointFromBFtoWF( mid2_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the initial relative orientation of the two bodies.
 *
 * \return The initial relative orientation of the two bodies.
 */
inline const Quat& FixedJoint::getIniOrient() const
{
   return qini_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the error of the relative orientation of body 2 wrt. body 1.
 *
 * \return The error of the relative orientation of body 2 wrt. body 1.
 */
inline Quat FixedJoint::getErrOrient() const
{
   const Quat cur( body2_->getQuaternion().getInverse() * body1_->getQuaternion() );
   return ( cur * qini_.getInverse() );
}
//*************************************************************************************************




//=================================================================================================
//
//  FIXED JOINT SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Fixed joint setup functions */
//@{
inline FixedJointID attachFixedJoint( BodyID body1, BodyID body2, real scale=real(1) );
       FixedJointID attachFixedJoint( BodyID body1, const Vec3& anchor1,
                                      BodyID body2, const Vec3& anchor2, real scale=real(1) );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new fixed joint between two rigid bodies.
 * \ingroup joint
 *
 * \param body1 The first body to which the joint is attached.
 * \param body2 The second body to which the joint is attached.
 * \param scale Visualization parameter for the scaling of the fixed joint \f$[0..1] \f$.
 * \return Handle for the new fixed joint.
 * \exception std::logic_error Cannot create fixed joint in MPI parallel simulation.
 * \exception std::logic_error Selected constraint solver is incapable of handling fixed joints.
 * \exception std::invalid_argument Invalid scaling parameter.
 *
 * This function creates a new fixed joint between the two rigid bodies \a body1 and \a body2,
 * which prevents any relative motion between the two bodies. The centers of mass are used as
 * anchor points for both attached rigid bodies. The \a scale parameter specifies the size of
 * the fixed joint in all active visualization systems (the default is 1).
 *
 * \image html fixedJoint.png
 * \image latex fixedJoint.eps "Fixed Joint" width=400pt
 *
 * The following code example illustrates the setup of a fixed joint between a sphere and
 * a box primitive:

   \code
   // Creating the iron sphere 1 with a radius of 2.5 at the global position (2,3,4).
   SphereID sphere = createSphere( 1, Vec3(2,3,4), 2.5, iron );

   // Creating the granite box 2 with side lengths (6,5,4) at the global position (10,3,4)
   BoxID box = createBox( 2, Vec3(10,3,4), Vec3(6,5,4), granite );

   // Creating a fixed joint between the sphere and the box primitive
   // Per default, the centers of mass of both attached bodies are used as anchor points.
   // In all active visualizations, the scaling factor of 0.5 will be used to draw the
   // fixed joint.
   FixedJointID joint = attachFixedJoint( sphere, box, 0.5 );
   \endcode

 * \b Note: Currently it is not possible to use fixed joints in MPI parallel simulations (i.e.,
 * in case the simulation uses more than a single process). The attempt to create a fixed joint
 * in a parallel simulation results results in a \a std::logic_error exception. Additionally,
 * it is not possible to use fixed joints in combination with the PolyhedralFrictionSolver or
 * the FFDSolver. The attempt to create a fixed joint in case any of these constraint solvers
 * is selected (see pe::pe_CONSTRAINT_SOLVER) results in a \a std::logic_error exception.
 */
inline FixedJointID attachFixedJoint( BodyID body1, BodyID body2, real scale )
{
   return attachFixedJoint( body1, Vec3(0,0,0), body2, Vec3(0,0,0), scale );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Fixed joint operators */
//@{
std::ostream& operator<<( std::ostream& os, const FixedJoint& joint );
std::ostream& operator<<( std::ostream& os, ConstFixedJointID joint );
//@}
//*************************************************************************************************

}//namespace pe

#endif
