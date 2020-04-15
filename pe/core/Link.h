//=================================================================================================
/*!
 *  \file pe/core/Link.h
 *  \brief Header file for the Link class
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

#ifndef _PE_CORE_LINK_H_
#define _PE_CORE_LINK_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/Section.h>
#include <pe/core/Types.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Vector3.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup link Link
 * \ingroup core
 */
/*!\brief Fixed link between two connected rigid bodies.
 * \ingroup link
 *
 * A link between two rigid bodies represents a point of interest within a union. During
 * each time step, the forces and torques acting in each specified link are calculated.
 * In order to be able to calculate the forces and torques, the two bodies have to touch
 * each other and the structure of the superordinate union must be separable in two distinct
 * sections that are only connected to each other at the specified link. Links within disjoint
 * unions or links within cyclic structures cannot be handled. Additionally, links in infinite
 * unions (i.e. union containing at least a single infinite rigid body) cannot be handled. In
 * these cases the link is invalidated and no force and torque calculation is performed. Note,
 * that it is also not possible to calculate the forces and torques in nested unions!\n
 * The example shows a union consisting of two spheres. The link between the two spheres can
 * be seen as a reading point for contact forces and torques.
 *
 * \image html link.png
 * \image latex link.eps "Link between two spheres" width=200pt
 */
class Link : private NonCopyable
{
private:
   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class Union;
   template< typename C > friend class UnionTrait;
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   //! Rigid body container for the setup of a link.
   typedef PtrVector<RigidBody,NoDelete>  Bodies;
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline bool         isValid()        const;
   inline id_t         getID()          const;
   inline UnionID      getUnion();
   inline ConstUnionID getUnion()       const;
          const Vec3   getNormal()      const;
   inline const Vec3&  getPosition()    const;
   inline const Vec3&  getRelPosition() const;

   inline BodyID       getBody1();
   inline ConstBodyID  getBody1()       const;
   inline BodyID       getBody2();
   inline ConstBodyID  getBody2()       const;
   //@}
   //**********************************************************************************************

   //**Force functions*****************************************************************************
   /*!\name Force functions */
   //@{
   inline const Vec3& getForce1()      const;
   inline const Vec3& getForce2()      const;
   inline const Vec3& getTorque1()     const;
   inline const Vec3& getTorque2()     const;
   inline const Vec3& getNormal1()     const;
   inline const Vec3& getNormal2()     const;
   inline const Vec3& getTangential1() const;
   inline const Vec3& getTangential2() const;
   inline void        resetForce();
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

private:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Link( id_t id, UnionID sb, BodyID b1, BodyID b2 );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Setup functions*****************************************************************************
   /*!\name Setup functions */
   //@{
   void setupLink( Bodies& bodies );
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   void update( const Vec3& dp );  // Translation update of the link
   void update( const Quat& dq );  // Rotation update of the link
   void calcForce( const Vec3& vdot, const Vec3& wdot );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool valid_;    //!< Flag value that indicates the validity of the link.
   id_t id_;       //!< The user-specific link ID.
   UnionID sb_;    //!< The superordinate union containing this link.
   BodyID b1_;     //!< The first directly attached rigid body.
   BodyID b2_;     //!< The second directly attached rigid body.
   Section sec1_;  //!< The first attached section.
                   /*!< The first rigid body is contained in the first section and is the
                        only point of contact to the second section. */
   Section sec2_;  //!< The second attached section.
                   /*!< The second rigid body is contained in the second section and is the
                        only point of contact to the first section. */
   Vec3 gPos_;     //!< Global position of this link.
   Vec3 rPos_;     //!< Relative position within the body frame of the superordinate union.
   Vec3 normal_;   //!< Relative body frame normal of the link (from body 2 to body 1 )

   Vec3 force1_;   //!< Total force in the link acting on section 1.
   Vec3 torque1_;  //!< Total torque in the link acting on section 1.
   Vec3 nForce1_;  //!< Total normal force in the link acting on section 1.
   Vec3 tForce1_;  //!< Total tangential force in the link acting on section 1.

   Vec3 force2_;   //!< Total force in the link acting on section 2.
   Vec3 torque2_;  //!< Total torque in the link acting on section 2.
   Vec3 nForce2_;  //!< Total normal force in the link acting on section 2.
   Vec3 tForce2_;  //!< Total tangential force in the link acting on section 2.
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
/*!\brief Returns the valid status of the link.
 *
 * \return \a true if the link is valid, \a false if it is not.
 */
inline bool Link::isValid() const
{
   return valid_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the user-specific ID of the link.
 *
 * \return The user-specific ID.
 */
inline id_t Link::getID() const
{
   return id_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the superordinate union in which the link is contained..
 *
 * \return The superordinate union.
 */
inline UnionID Link::getUnion()
{
   return sb_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the superordinate union in which the link is contained..
 *
 * \return The superordinate union.
 */
inline ConstUnionID Link::getUnion() const
{
   return sb_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the global position of the link.
 *
 * \return The global position of the link.
 */
inline const Vec3& Link::getPosition() const
{
   return gPos_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the relative position of the link within the superordinate union.
 *
 * \return The relative position of the link.
 */
inline const Vec3& Link::getRelPosition() const
{
   return rPos_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first attached rigid body.
 *
 * \return The first attached rigid body.
 */
inline BodyID Link::getBody1()
{
   return b1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first attached rigid body.
 *
 * \return The first attached rigid body.
 */
inline ConstBodyID Link::getBody1() const
{
   return b1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second attached rigid body.
 *
 * \return The second attached rigid body.
 */
inline BodyID Link::getBody2()
{
   return b2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second attached rigid body.
 *
 * \return The second attached rigid body.
 */
inline ConstBodyID Link::getBody2() const
{
   return b2_;
}
//*************************************************************************************************




//=================================================================================================
//
//  FORCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the link force acting on section 1.
 *
 * \return The link force on section 1.
 */
inline const Vec3& Link::getForce1() const
{
   return force1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the link force acting on section 2.
 *
 * \return The link force on section 2.
 */
inline const Vec3& Link::getForce2() const
{
   return force2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the link torque acting on section 1.
 *
 * \return The link torque on section 1.
 */
inline const Vec3& Link::getTorque1() const
{
   return torque1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the link torque acting on section 2.
 *
 * \return The link torque on section 2.
 */
inline const Vec3& Link::getTorque2() const
{
   return torque2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the link normal force acting on section 1.
 *
 * \return The link normal force on section 1.
 */
inline const Vec3& Link::getNormal1() const
{
   return nForce1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the link normal force acting on section 2.
 *
 * \return The link normal force on section 2.
 */
inline const Vec3& Link::getNormal2() const
{
   return nForce2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the link tangential force acting on section 1.
 *
 * \return The link tangential force on section 1.
 */
inline const Vec3& Link::getTangential1() const
{
   return tForce1_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the link tangential force acting on section 2.
 *
 * \return The link tangential force on section 2.
 */
inline const Vec3& Link::getTangential2() const
{
   return tForce2_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing the force from the two attached sections.
 *
 * \return void
 *
 * The function resets the force in both attached sections. The calculated forces in the link
 * will not be reset and therefore continue to hold the calculated values from the last time
 * step.
 */
inline void Link::resetForce()
{
   sec1_.resetForce();
   sec2_.resetForce();
}
//*************************************************************************************************




//=================================================================================================
//
//  LINK SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Link setup functions */
//@{
LinkID createLink( UnionID u, id_t id, BodyID b1, BodyID b2 );
void   destroy( LinkID link );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Link operators */
//@{
std::ostream& operator<<( std::ostream& os, const Link& c );
std::ostream& operator<<( std::ostream& os, ConstLinkID c );
//@}
//*************************************************************************************************

} // namespace pe

#endif
