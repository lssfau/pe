//=================================================================================================
/*!
 *  \file pe/core/attachable/ForceGenerator.h
 *  \brief Interface for force generators
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

#ifndef _PE_CORE_ATTACHABLE_FORCEGENERATOR_H_
#define _PE_CORE_ATTACHABLE_FORCEGENERATOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/attachable/Attachable.h>
#include <pe/core/Types.h>
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
/*!\defgroup force_generator Force generators
 * \ingroup core
 *
 * Force generators offer the functionality to stimulate rigid bodies once each time step by
 * adding a certain amount of force to it. The \b pe engine currently offers two different
 * force generators: \ref gravity and \ref spring.\n\n
 *
 *
 * \section gravity Gravity force generators
 *
 * The Gravity force generator exerts a certain acceleration force to a single rigid body.
 * This force generator can for example be used to apply an additional or special gravity to
 * a specific rigid body. Gravity force generators are created by the following functions:
 *
 * - pe::attachGravity( BodyID body, real gx, real gy, real gz );
 * - pe::attachGravity( BodyID body, const Vec3& gravity );
 *
 * Example:

   \code
   using namespace pe;

   // Creating a spherical rigid body
   SphereID sphere = createSphere( 1, 2.4, -1.4, 5.0, 0.9, oak );

   // Attaching a gravity force generator to the sphere
   GravityID gravity = attachGravity( sphere, Vec3( 0.0, 0.0, -9.81 ) );

   // Simulation code
   ...

   // Destroying the sphere. Note that this additionally destroys the gravity force generator.
   // Any further use of active handles to the force generator is invalid!
   destroy( sphere );
   \endcode

 * \b Note: A Gravity force generator applies an additional force to a rigid body. The body
 * is still affected by the global gravity!
 *
 *
 * \section spring Spring force generators
 *
 * The Spring force generator estimates and exert the reaction force of a spring-damper system
 * on the two anchored rigid bodies. The acting forces are calculated according to the following
 * two formulas:

         \f[ F = -k \cdot x - \delta \cdot v \f]
         \f[ m \cdot \ddot{x} + \delta \cdot \dot{x} + k \cdot x = 0, \f]

 * where \a F is the acting force, \a x is the displacement or the anchor points from the rest
 * length of the spring, \a v is the relative velocity between the two anchor points, \a k is
 * the stiffness of the spring and \f$ \delta \f$ is the damping factor.\n
 * In order to create a spring force generator, one of the following spring creation functions
 * can be used:
 *
 * - pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, bool visible );
 * - pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, bool visible );
 * - pe::attachSpring( BodyID body1, BodyID body2, real stiffness, real damping, real length, bool visible );
 * - pe::attachSpring( BodyID body1, const Vec3& anchor1, BodyID body2, const Vec3& anchor2, real stiffness, real damping, real length, bool visible );
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
 */
/*!\brief Base class for force generators.
 * \ingroup force_generator
 *
 * The ForceGenerator class is the base class for all attachable force generators of the physics
 * engine. It provides the common data members and common functionality for all force generators.
 */
class ForceGenerator : public Attachable
{
private:
   //**Type definitions****************************************************************************
   //! Vector for currently active force generators.
   typedef PtrVector<ForceGenerator,NoDelete>  Generators;
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit ForceGenerator( AttachableType type, id_t sid, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~ForceGenerator() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline bool isVisible() const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setVisible( bool visible=true ) = 0;
   //@}
   //**********************************************************************************************

   //**MPI functions*******************************************************************************
   /*!\name MPI functions */
   //@{
   virtual bool isRemote() const = 0;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os ) const = 0;
   //@}
   //**********************************************************************************************

protected:
   //**Force functions*****************************************************************************
   /*!\name Force functions */
   //@{
   virtual void applyForce() = 0;
   //@}
   //**********************************************************************************************

private:
   //**Force functions*****************************************************************************
   /*!\name Force functions */
   //@{
   static void applyForces();
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   bool visible_;  //!< Visibility flag.
                   /*!< If the force generator is visible, it is automatically registered
                        for all active visualizations. */
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static Generators generators_;  //!< The currently active force generators.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template<class T> friend void boost::checked_delete( T* x );
   friend class World;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the force generator is visible or not.
 *
 * \return \a true in case the force generator is visible, \a false in case it is not.
 */
inline bool ForceGenerator::isVisible() const
{
   return visible_;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Force generator operators */
//@{
std::ostream& operator<<( std::ostream& os, const ForceGenerator& fg );
std::ostream& operator<<( std::ostream& os, ConstForceGeneratorID fg );
//@}
//*************************************************************************************************

} // namespace pe

#endif
