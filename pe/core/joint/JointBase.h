//=================================================================================================
/*!
 *  \file pe/core/joint/JointBase.h
 *  \brief Header file for the JointBase class
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

#ifndef _PE_CORE_JOINT_JOINTBASE_H_
#define _PE_CORE_JOINT_JOINTBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/Attachable.h>
#include <pe/core/batches/JointTrait.h>
#include <pe/core/Configuration.h>
#include <pe/core/detection/coarse/JointTrait.h>
#include <pe/core/detection/fine/JointTrait.h>
#include <pe/core/joint/JointType.h>
#include <pe/core/response/JointTrait.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/Types.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/VectorN.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Joint between two connected primitives.
 * \ingroup joints
 *
 * The JointBase class is the base class for all types of joints between rigid bodies in the
 * simulation system.
 */
class PE_PROTECTED JointBase : public detection::coarse::JointTrait<Config>
                , public detection::fine::JointTrait<Config>
                , public batches::JointTrait<Config>
                , public response::JointTrait<Config>
//                 , public Attachable  TODO!!
{
private:
   //**Type definitions****************************************************************************
   //! Abbreviation for the coarse collision detection joint trait base.
   /*! This base class configures the JointBase class for the requirements of the selected
       coarse collision detection algorithm. */
   typedef detection::coarse::JointTrait<Config>  CCDJT;

   //! Abbreviation for the fine collision detection joint trait base class.
   /*! This base class configures the JointBase class for the requirements of the selected
       fine collision detection algorithm. */
   typedef detection::fine::JointTrait<Config>  FCDJT;

   //! Abbreviation for the batch generation joint trait base.
   /*! This base class configures the JointBase class for the requirements of the selected
       batch generation algorithm. */
   typedef batches::JointTrait<Config>  BJT;

   //! Abbreviation for the collision response joint trait base.
   /*! This base class configures the JointBase class for the requirements of the selected
       collision response algorithm. */
   typedef response::JointTrait<Config>  RJT;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit JointBase( JointType type, size_t rows, BodyID body1, BodyID body2, real scale, id_t sid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~JointBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline  bool        isVisible()   const;
   inline  bool        isActive()    const;

   inline  size_t      getIndex()    const;
   inline  BodyID      getBody1();
   inline  ConstBodyID getBody1()    const;
   inline  BodyID      getBody2();
   inline  ConstBodyID getBody2()    const;
   inline  JointType   getType()     const;
   inline  size_t      getRows()     const;
   inline  real        getScale()    const;
   inline  id_t        getSystemID() const;

   virtual real        getCorParam() const = 0;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
          void setScale( real scale   );
   inline void setIndex( size_t index );
   //@}
   //**********************************************************************************************

   //**Calculation functions***********************************************************************
   /*!\name Calculation functions */
   //@{
   virtual void calcMat     ( MatN& JLin1, MatN& JAng1, MatN& JLin2, MatN& JAng2 ) const = 0;
   virtual void calcErrorVec( VecN& bError )                                       const = 0;
   //@}
   //**********************************************************************************************

   //**Limit functions*****************************************************************************
   /*!\name Limit functions */
   //@{
   virtual bool isViolatedLow   ()  const = 0;
   virtual bool isViolatedHigh  ()  const = 0;
   virtual void calcLimitMat    ( MatN& JLin1L, MatN& JAng1L, MatN& JLin2L, MatN& JAng2L ) const = 0;
   virtual real calcLimitErrorLo()  const = 0;
   virtual real calcLimitErrorHi()  const = 0;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os ) const = 0;
   //@}
   //**********************************************************************************************

protected:
   //**Member variables ***************************************************************************
   /*!\name Member variables */
   //@{
   const JointType type_;   //!< The type of the joint.
   const size_t    rows_;   //!< Number of rows the joint occupies in Jacobian matrices.
   id_t            sid_;    //!< The unique system-specific joint ID.
   size_t          index_;  //!< The current batch index of the joint.
                            /*!< During the setup of a constraint graph, the joint is
                                 assigned to a batch of constraints. The batch index
                                 represents the index within this constraint batch. The index
                                 is first assigned during the batch setup. */
   BodyID          body1_;  //!< The first constrained rigid body.
   BodyID          body2_;  //!< The second constrained rigid body.
   real            scale_;  //!< Scaling parameter for visualization purposes \f$[0..1] \f$.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the joint is visible or not.
 *
 * \return \a true in case of a visible joint, \a false in case of an invisible joint.
 *
 * This function returns whether the joint is visible in all visualizations or not. In case
 * the scale of a joint has been set to zero, the function returns \a false. Otherwise it
 * returns \a true.
 */
inline bool JointBase::isVisible() const
{
   if( scale_ == real(0) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the joint is active or not.
 *
 * \return \a true if the joint is active, \a false if it inactive.
 *
 * This function returns whether the joint is active or not. A joint is considered active
 * at least one of the attached rigid bodies is active.
 */
inline bool JointBase::isActive() const
{
   return body1_->isActive() || body2_->isActive();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current batch index of the joint.
 *
 * \return The current batch index.
 */
inline size_t JointBase::getIndex() const
{
   return index_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first constrained rigid body.
 *
 *\return The first constrained rigid body.
 */
inline BodyID JointBase::getBody1()
{
   return body1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first constrained rigid body.
 *
 *\return The first constrained rigid body.
 */
inline ConstBodyID JointBase::getBody1() const
{
   return body1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second constrained rigid body.
 *
 *\return The second constrained rigid body.
 */
inline BodyID JointBase::getBody2()
{
   return body2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second constrained rigid body.
 *
 *\return The second constrained rigid body.
 */
inline ConstBodyID JointBase::getBody2() const
{
   return body2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the type of the joint.
 *
 * This function returns the type of the joint.
 *\return The type of the joint.
 */
inline JointType JointBase::getType() const
{
   return type_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the unique system-specific ID of the joint.
 *
 * \return The system-specific ID.
 */
inline id_t JointBase::getSystemID() const
{
   return sid_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rows of the Jacobian matrices
 *
 * \return The number of rows of the Jacobian matrices.
 */
inline size_t JointBase::getRows() const
{
   return rows_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the visualization parameter.
 *
 * \return The visualization parameter.
 */
inline real JointBase::getScale() const
{
   return scale_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the batch index of the joint.
 *
 * \param index The new batch index of the joint.
 * \return void
 *
 * This function sets the batch index of the joint, i.e., the identification number of the
 * joint within a batch.
 */
inline void JointBase::setIndex( size_t index )
{
   index_ = index;
}
//*************************************************************************************************

} // namespace pe

#endif
