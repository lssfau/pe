//=================================================================================================
/*!
 *  \file pe/core/attachable/SpringBase.h
 *  \brief Header file for the SpringBase class
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

#ifndef _PE_CORE_ATTACHABLE_SPRINGBASE_H_
#define _PE_CORE_ATTACHABLE_SPRINGBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/ForceGenerator.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for the spring force generator.
 * \ingroup force_generator
 *
 * The SpringBase class represents the base class for the spring force generator. It
 * provides the basic functionality of a spring force generator. For a full description
 * of spring force generators, see the Spring class description.
 */
class SpringBase : public ForceGenerator
{
private:
   //**Type definitions****************************************************************************
   typedef ForceGenerator  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit SpringBase( id_t sid, BodyID body1, BodyID body2, const Vec3& anchor1, const Vec3& anchor2,
                        real stiffness, real damping, real length, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~SpringBase();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline BodyID      getBody1();
   inline ConstBodyID getBody1()      const;
   inline BodyID      getBody2();
   inline ConstBodyID getBody2()      const;
   inline const Vec3& getAnchor1BF()  const;
   inline const Vec3  getAnchor1WF()  const;
   inline const Vec3& getAnchor2BF()  const;
   inline const Vec3  getAnchor2WF()  const;
   inline real        getStiffness()  const;
   inline real        getDamping()    const;
   inline real        getRestLength() const;
   inline real        getLength()     const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setStiffness( real stiffness );
   void setDamping  ( real damping   );
   void setLength   ( real length    );
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 anchor1_;    //!< The first body's anchor point in body relative coordinates.
   Vec3 anchor2_;    //!< The second body's anchor point in body relative coordinates.
   real stiffness_;  //!< Stiffness of the spring \f$ (0..\infty) \f$.
   real damping_;    //!< Damping of the spring \f$ [0..\infty) \f$.
   real length_;     //!< The length in non-deformed state \f$ (0..\infty) \f$.
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
/*!\brief Returns the first rigid body attached to the spring.
 *
 * \return The first body anchored to the spring.
 */
inline BodyID SpringBase::getBody1()
{
   return bodies_[0];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first rigid body attached to the spring.
 *
 * \return The first body anchored to the spring.
 */
inline ConstBodyID SpringBase::getBody1() const
{
   return bodies_[0];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second rigid body attached to the spring.
 *
 * \return The second body anchored to the spring.
 */
inline BodyID SpringBase::getBody2()
{
   return bodies_[1];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second rigid body attached to the spring.
 *
 * \return The second body anchored to the spring.
 */
inline ConstBodyID SpringBase::getBody2() const
{
   return bodies_[1];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first body's anchor point.
 *
 * \return The first body's anchor point.
 *
 * \b Note: This function returns the first body's anchor point in body relative coordinates!
 */
inline const Vec3& SpringBase::getAnchor1BF() const
{
   return anchor1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first body's anchor point.
 *
 * \return The first body's anchor point.
 *
 * \b Note: This function returns the first body's anchor point in global coordinates!
 */
inline const Vec3 SpringBase::getAnchor1WF() const
{
   return bodies_[0]->pointFromBFtoWF( anchor1_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second body's anchor point.
 *
 * \return The second body's anchor point.
 *
 * \b Note: This function returns the second body's anchor point in body relative coordinates!
 */
inline const Vec3& SpringBase::getAnchor2BF() const
{
   return anchor2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second body's anchor point.
 *
 * \return The second body's anchor point.
 *
 * \b Note: This function returns the second body's anchor point in global coordinates!
 */
inline const Vec3 SpringBase::getAnchor2WF() const
{
   return bodies_[1]->pointFromBFtoWF( anchor2_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the stiffness of the spring.
 *
 * \return The stiffness of the spring.
 */
inline real SpringBase::getStiffness() const
{
   return stiffness_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the damping of the spring.
 *
 * \return The damping of the spring.
 */
inline real SpringBase::getDamping() const
{
   return damping_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the rest length of the spring.
 *
 * \return The rest lengths of the spring.
 */
inline real SpringBase::getRestLength() const
{
   return length_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current length of the spring.
 *
 * \return The current lengths of the spring.
 */
inline real SpringBase::getLength() const
{
   return ( bodies_[0]->pointFromBFtoWF( anchor1_ ) - bodies_[1]->pointFromBFtoWF( anchor2_ ) ).length();
}
//*************************************************************************************************

} // namespace pe

#endif
