//=================================================================================================
/*!
 *  \file pe/core/attachable/Gravity.h
 *  \brief Header file for the Gravity class
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

#ifndef _PE_CORE_ATTACHABLE_GRAVITY_H_
#define _PE_CORE_ATTACHABLE_GRAVITY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/Configuration.h>
#include <pe/core/response/GravityTrait.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Gravity force generator.
 * \ingroup force_generator
 *
 * The Gravity force generator exerts a certain acceleration force to a single rigid body.
 * This force generator can for example be used to apply a special gravity in addition to
 * the global gravity to a specific rigid body. In order to set up a gravity force generator,
 * one of the following setup functions can be used:
 *
 * - pe::attachGravity( BodyID body, real gx, real gy, real gz );
 * - pe::attachGravity( BodyID body, const Vec3& gravity );
 *
 * In order to destroy a specific gravity force generator, the detach() function can be used.
 * It detaches the gravity force generator from its rigid body and automatically destroys it.
 *
 * - pe::detach( AttachableID attachable )
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
 */
class Gravity : public GravityTrait<Config>
{
private:
   //**Type definitions****************************************************************************
   typedef GravityTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Gravity( id_t sid, BodyID body, const Vec3& gravity );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Gravity();
   //@}
   //**********************************************************************************************

public:
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
   friend GravityID attachGravity( BodyID body, const Vec3& gravity );
   friend GravityID instantiateGravity( id_t sid, BodyID body, const Vec3& gravity );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GRAVITY SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Gravity setup functions */
//@{
inline GravityID attachGravity( BodyID body, real gx, real gy, real gz );
       GravityID attachGravity( BodyID body, const Vec3& gravity );
       GravityID instantiateGravity( id_t sid, BodyID body, const Vec3& gravity );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new gravity force generator.
 * \ingroup force_generator
 *
 * \param body The rigid body the force generator is attached to.
 * \param gx The x-component of the exerted gravity (in global coordinates).
 * \param gy The y-component of the exerted gravity (in global coordinates).
 * \param gz The z-component of the exerted gravity (in global coordinates).
 * \return Handle for the new gravity force generator.
 * \exception std::logic_error Invalid setup of gravity force generator inside a global section.
 * \exception std::invalid_argument Cannot attach gravity force generator to global rigid body.
 *
 * \b Note: Gravity force generators cannot be created within a pe_GLOBAL_SECTION. The attempt
 * to create a global gravity force generator will result in a \a std::logic_error exception!
 * Additionally, gravity force generators cannot be attached to global rigid bodies. The attempt
 * to attach a spring to a global rigid body result in a \a std::invalid_argument exception!
 */
inline GravityID attachGravity( BodyID body, real gx, real gy, real gz )
{
   // Creating a new gravity force generator
   return attachGravity( body, Vec3(gx,gy,gz) );
}
//*************************************************************************************************

} // namespace pe

#endif
