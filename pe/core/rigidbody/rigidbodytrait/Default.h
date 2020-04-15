//=================================================================================================
/*!
 *  \file pe/core/rigidbody/rigidbodytrait/Default.h
 *  \brief Header file for the default implementation of the RigidBodyTrait class template.
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_DEFAULT_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/RigidBodyBase.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rigid body customization class for the collision response.
 * \ingroup rigid_body
 *
 * The RigidBodyTrait class template is a customization class for rigid bodies in general.
 * Its main purpose is the customization of the RigidBody class for the selected collision
 * response algorithm (see pe::pe_CONSTRAINT_SOLVER).
 */
template< typename C >  // Type of the configuration
class RigidBodyTrait : public RigidBodyBase
{
private:
   //**Type definitions****************************************************************************
   typedef RigidBodyBase Parent;  //!< The type of the parent class.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit RigidBodyTrait( BodyID body );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~RigidBodyTrait() = 0;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the RigidBodyTrait constructor.
 *
 * \param body The body ID of this rigid body.
 */
template< typename C >  // Type of the configuration
RigidBodyTrait<C>::RigidBodyTrait( BodyID body )
   : Parent( body )  // Initialization of the parent class
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the RigidBodyTrait destructor.
 */
template< typename C >  // Type of the configuration
RigidBodyTrait<C>::~RigidBodyTrait()
{}
//*************************************************************************************************

} // namespace pe

#endif
