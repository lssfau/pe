//=================================================================================================
/*!
 *  \file pe/core/configuration/FFDSolver.h
 *  \brief Specialization of the Configuration class template for the FFD solver
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

#ifndef _PE_CORE_CONFIGURATION_FFDSOLVER_H_
#define _PE_CORE_CONFIGURATION_FFDSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/configuration/Default.h>


namespace pe {

//=================================================================================================
//
//  SPECIALIZATION FOR THE FAST FRICTIONAL DYNAMICS SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Specialization of the configuration for the fast frictional dynamics solver.
 * \ingroup core
 *
 * This specialization of the Configuration class handles the special case that the fast
 * frictional dynamics solver doesn't require a batch generator. Therefore the SingleBatch
 * algorithm is specifically chosen as batch generation algorithm, since this algorithm does
 * not impose additional data members or functionality to both rigid bodies and contacts.
 */
template< template<typename> class CD    // Type of the coarse collision detection algorithm
        , typename FD                    // Type of the fine collision detection algorithm
        , template<typename> class BG >  // Type of the batch generation algorithm
struct Configuration<CD,FD,BG,response::FFDSolver>
{
public:
   //**Type definitions****************************************************************************
   typedef Configuration<CD,FD,batches::SingleBatch,response::FFDSolver>  Config;  //!< Type of this configuration.

   typedef RigidBody             BodyType;            //!< Type of the rigid bodies.
   typedef RigidBody*            BodyID;              //!< Handle for a rigid body.
   typedef const RigidBody*      ConstBodyID;         //!< Handle for a constant rigid body.

   typedef Box                   BoxType;             //!< Type of the box geometric primitive.
   typedef Box*                  BoxID;               //!< Handle for a box primitive.
   typedef const Box*            ConstBoxID;          //!< Handle for a constant box primitive.

   typedef Capsule               CapsuleType;         //!< Type of the capsule geometric primitive.
   typedef Capsule*              CapsuleID;           //!< Handle for a capsule primitive.
   typedef const Capsule*        ConstCapsuleID;      //!< Handle for a constant capsule primitive.

   typedef Cylinder              CylinderType;        //!< Type of the cylinder geometric primitive.
   typedef Cylinder*             CylinderID;          //!< Handle for a cylinder primitive.
   typedef const Cylinder*       ConstCylinderID;     //!< Handle for a constant cylinder primitive.

   typedef Plane                 PlaneType;           //!< Type of the plane geometric primitive.
   typedef Plane*                PlaneID;             //!< Handle for a plane primitive.
   typedef const Plane*          ConstPlaneID;        //!< Handle for a constant plane primitive.

   typedef Sphere                SphereType;          //!< Type of the sphere geometric primitive.
   typedef Sphere*               SphereID;            //!< Handle for a sphere primitive.
   typedef const Sphere*         ConstSphereID;       //!< Handle for a constant sphere primitive.

   typedef Union                 UnionType;           //!< Type of the union compound geometry.
   typedef Union*                UnionID;             //!< Handle for a union.
   typedef const Union*          ConstUnionID;        //!< Handle for a constant union.

   typedef Attachable            AttachableType;      //!< Type of the attachables.
   typedef Attachable*           AttachableID;        //!< Handle for an attachable.
   typedef const Attachable*     ConstAttachableID;   //!< Handle for a constant attachable.

   typedef Gravity               GravityType;         //!< Type of the gravity force generators.
   typedef Gravity*              GravityID;           //!< Handle for a gravity force generator.
   typedef const Gravity*        ConstGravityID;      //!< Handle for a constant gravity force generator.

   typedef Spring                SpringType;          //!< Type of the spring force generators.
   typedef Spring*               SpringID;            //!< Handle for a spring force generator.
   typedef const Spring*         ConstSpringID;       //!< Handle for a constant spring force generator.

   typedef Contact               ContactType;         //!< Type of the contacts.
   typedef Contact*              ContactID;           //!< Handle for a contact.
   typedef const Contact*        ConstContactID;      //!< Handle for a constant contact.

   typedef Joint                 JointType;           //!< Type of the joints.
   typedef Joint*                JointID;             //!< Handle for a joint.
   typedef const Joint*          ConstJointID;        //!< Handle for a constant joint.

   typedef SliderJoint           SliderJointType;     //!< Type of the slider joint.
   typedef SliderJoint*          SldierJointID;       //!< Handle for a slider joint.
   typedef const SliderJoint*    ConstSliderJointID;  //!< Handle for a constant slider joint.

   typedef HingeJoint            HingeJointType;      //!< Type of the hinge joint.
   typedef HingeJoint*           HingeJointID;        //!< Handle for a hinge joint.
   typedef const HingeJoint*     ConstHingeJointID;   //!< Handle for a constant hinge joint.

   typedef BallJoint             BallJointType;       //!< Type of the ball joint.
   typedef BallJoint*            BallJointID;         //!< Handle for a ball joint.
   typedef const BallJoint*      ConstBallJointID;    //!< Handle for a constant ball joint.

   typedef Process               ProcessType;         //!< Type of the remote processes.
   typedef Process*              ProcessID;           //!< Handle for a remote process.
   typedef const Process*        ConstProcessID;      //!< Handle for a constant remote process.

   //! Type of the coarse collision detection algorithm.
   /*! The type of the coarse collision detection algorithm is selected by the setting of
       the pe::pe_COARSE_COLLISION_DETECTOR macro. */
   typedef CD<Config>  CoarseDetector;

   //! Type of the fine collision detector.
   /*! The type of the fine collision detection algorithm is selected by the setting of
       the pe::pe_FINE_COLLISION_DETECTOR macro. */
   typedef FD  FineDetector;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
