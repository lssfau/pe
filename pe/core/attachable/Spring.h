//=================================================================================================
/*!
 *  \file pe/core/attachable/Spring.h
 *  \brief Header file for the Spring class
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

#ifndef _PE_CORE_ATTACHABLE_SPRING_H_
#define _PE_CORE_ATTACHABLE_SPRING_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/Configuration.h>
#include <pe/core/response/SpringTrait.h>
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
/*!\brief Spring force generator.
 * \ingroup force_generator
 *
 * The Spring class represents a force generator to estimate and exert the reaction force of
 * a spring-damper system on the two anchored rigid bodies. The acting forces are calculated
 * according to the following two formulas:

         \f[ F = -k \cdot x - \delta \cdot v \f]
         \f[ m \cdot \ddot{x} + \delta \cdot \dot{x} + k \cdot x = 0, \f]

 * where \a F is the acting force, \a x is the displacement or the anchor points from the rest
 * length of the spring, \a v is the relative velocity between the two anchor points, \a k is
 * the stiffness of the spring and \f$ \delta \f$ is the damping factor.\n
 * In order to create a spring force generator, one of the following spring creation functions
 * can be used:
 *
 * -# pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, bool visible );
 * -# pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, bool visible );
 * -# pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, real length, bool visible );
 * -# pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, real length, bool visible );
 *
 * Example:

   \code
   using namespace pe;

   // Creating two spherical rigid bodies
   SphereID sphere1 = createSphere( 1, -4.0, 0.0, 0.0, 1.0, fir    );
   SphereID sphere2 = createSphere( 2,  4.0, 0.0, 0.0, 1.0, copper );

   // Attaching a spring force generator to the two spheres
   SpringID spring = attachSpring( sphere1, sphere2, 1000, 100 );

   // Simulation code
   ...

   // Destroying the first sphere. Note that this additionally destroys the attached spring force
   // generator. Any further use of active handles to the force generator is invalid!
   destroy( sphere1 );
   \endcode

 * The following image illustrates the simulation of the well known Newton's cradle scenario,
 * where springs are used as mountings for the oscillating spheres.
 *
 * \image html spring.png
 * \image latex spring.eps "Newton's cradle" width=200pt
 */
class Spring : public SpringTrait<Config>
{
private:
   //**Type definitions****************************************************************************
   typedef SpringTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Spring( id_t sid, BodyID body1, BodyID body2, const Vec3& anchor1, const Vec3& anchor2,
                    real stiffness, real damping, real length, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Spring();
   //@}
   //**********************************************************************************************

public:
   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setVisible( bool visible );
   //@}
   //**********************************************************************************************

   //**MPI functions*******************************************************************************
   /*!\name MPI functions */
   //@{
   virtual bool isRemote() const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Force functions*****************************************************************************
   /*!\name Force functions */
   //@{
   virtual void applyForce();
   //@}
   //**********************************************************************************************

private:
   //**Spring setup functions*************************************************************************
   /*! \cond PE_INTERNAL */
   friend SpringID attachSpring( BodyID body1, const Vec3& anchor1,
                                 BodyID body2, const Vec3& anchor2,
                                 real stiffness, real damping, real length, bool visible );
   friend SpringID instantiateSpring( id_t sid, BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2,
                                      real stiffness, real damping, real length, bool visible );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  SPRING SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Spring setup functions */
//@{
inline SpringID attachSpring( BodyID body1, BodyID body2,
                              real stiffness, real damping, bool visible=true );

inline SpringID attachSpring( BodyID body1, const Vec3& anchor1,
                              BodyID body2, const Vec3& anchor2,
                              real stiffness, real damping, bool visible=true );

inline SpringID attachSpring( BodyID body1, BodyID body2,
                              real stiffness, real damping, real length, bool visible=true );

       SpringID attachSpring( BodyID body1, const Vec3& anchor1,
                              BodyID body2, const Vec3& anchor2,
                              real stiffness, real damping, real length, bool visible );
                              
       SpringID attachSpring( BodyID body1, const Vec3& anchor1,
                              BodyID body2, const Vec3& anchor2,
                              real stiffness, real damping, real length );

       SpringID instantiateSpring( id_t sid, BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2,
                                   real stiffness, real damping, real length, bool visible );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new spring between two rigid bodies.
 * \ingroup force_generator
 *
 * \param body1 The first rigid body anchored to the spring.
 * \param body2 The second rigid body anchored to the spring.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param visible Specifies if the spring is visible in a visualization.
 * \return Handle for the new spring.
 * \exception std::logic_error Invalid setup of spring inside a global section.
 * \exception std::invalid_argument Cannot attach spring to global rigid body.
 * \exception std::invalid_argument Invalid anchor point for body 1.
 * \exception std::invalid_argument Invalid anchor point for body 2.
 * \exception std::invalid_argument Invalid spring stiffness.
 * \exception std::invalid_argument Invalid spring damping.
 * \exception std::invalid_argument Invalid rest length.
 *
 * This function creates an initially unstressed spring between the two rigid bodies \a body1
 * and \a body2. The centers of mass are used as anchor points for both attached rigid bodies.
 * \a stiffness specifies the stiffness of the spring, whereas \a damping sets the damping
 * factor of the spring-damper system.\n
 *
 * \b Note: Springs cannot be created within a pe_GLOBAL_SECTION. The attempt to create a global
 * spring will result in a \a std::logic_error exception! Additionally, springs cannot be attached
 * to global rigid bodies. The attempt to attach a spring to a global rigid body result in a
 * \a std::invalid_argument exception!
 */
inline SpringID attachSpring( BodyID body1, BodyID body2,
                              real stiffness, real damping, bool visible )
{
   const Vec3 l( body1->getPosition() - body2->getPosition() );
   return attachSpring( body1, Vec3(0,0,0), body2, Vec3(0,0,0),
                        stiffness, damping, l.length(), visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new spring between two rigid bodies.
 * \ingroup force_generator
 *
 * \param body1 The first rigid body anchored to the spring.
 * \param anchor1 The first body's anchor point in body relative coordinates.
 * \param body2 The second rigid body anchored to the spring.
 * \param anchor2 The second body's anchor point in body relative coordinates.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param visible Specifies if the spring is visible in a visualization.
 * \return Handle for the new spring.
 * \exception std::logic_error Invalid setup of spring inside a global section.
 * \exception std::invalid_argument Cannot attach spring to global rigid body.
 * \exception std::invalid_argument Invalid anchor point for body 1.
 * \exception std::invalid_argument Invalid anchor point for body 2.
 * \exception std::invalid_argument Invalid spring stiffness.
 * \exception std::invalid_argument Invalid spring damping.
 * \exception std::invalid_argument Invalid rest length.
 *
 * This function creates an initially unstressed spring between the two rigid bodies \a body1
 * and \a body2. \a anchor1 is the anchor point for \a body1, given in coordinates relative to
 * the body frame of \a body1. \a anchor2 is the anchor point for \a body2 and specified in
 * coordinates relative to the body frame of \a body2. \a stiffness specifies the stiffness of
 * the spring, whereas \a damping sets the damping factor of the spring-damper system.
 *
 * \b Note: Springs cannot be created within a pe_GLOBAL_SECTION. The attempt to create a global
 * spring will result in a \a std::logic_error exception! Additionally, springs cannot be attached
 * to global rigid bodies. The attempt to attach a spring to a global rigid body result in a
 * \a std::invalid_argument exception!
 */
inline SpringID attachSpring( BodyID body1, const Vec3& anchor1,
                              BodyID body2, const Vec3& anchor2,
                              real stiffness, real damping, bool visible )
{
   const Vec3 l( body1->pointFromBFtoWF( anchor1 ) - body2->pointFromBFtoWF( anchor2 ) );
   return attachSpring( body1, anchor1, body2, anchor2,
                        stiffness, damping, l.length(), visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new spring between two rigid bodies.
 * \ingroup force_generator
 *
 * \param body1 The first rigid body anchored to the spring.
 * \param body2 The second rigid body anchored to the spring.
 * \param stiffness The stiffness of the spring \f$ (0..\infty) \f$.
 * \param damping The damping of the spring \f$ [0..\infty) \f$.
 * \param length The rest length of the spring \f$ (0..\infty) \f$.
 * \param visible Specifies if the spring is visible in a visualization.
 * \return Handle for the new spring.
 * \exception std::logic_error Invalid setup of spring inside a global section.
 * \exception std::invalid_argument Cannot attach spring to global rigid body.
 * \exception std::invalid_argument Invalid anchor point for body 1.
 * \exception std::invalid_argument Invalid anchor point for body 2.
 * \exception std::invalid_argument Invalid spring stiffness.
 * \exception std::invalid_argument Invalid spring damping.
 * \exception std::invalid_argument Invalid rest length.
 *
 * This function creates a spring between the two rigid bodies \a body1 and \a body2. The centers
 * of mass are used as anchor points for both attached rigid bodies. \a stiffness specifies the
 * stiffness of the spring, whereas \a damping sets the damping factor of the spring-damper system.
 * \a length specifies the rest length of the spring. In case the centers of mass of the two
 * attached rigid bodies have a different distance, the spring is initially stressed.
 *
 * \b Note: Springs cannot be created within a pe_GLOBAL_SECTION. The attempt to create a global
 * spring will result in a \a std::logic_error exception! Additionally, springs cannot be attached
 * to global rigid bodies. The attempt to attach a spring to a global rigid body result in a
 * \a std::invalid_argument exception!
 */
inline SpringID attachSpring( BodyID body1, BodyID body2,
                              real stiffness, real damping, real length, bool visible )
{
   return attachSpring( body1, Vec3(0,0,0), body2, Vec3(0,0,0),
                        stiffness, damping, length, visible );
}
//*************************************************************************************************

} // namespace pe

#endif
