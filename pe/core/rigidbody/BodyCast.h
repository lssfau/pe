//=================================================================================================
/*!
 *  \file pe/core/rigidbody/BodyCast.h
 *  \brief Cast operators for rigid bodies
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

#ifndef _PE_CORE_RIGIDBODY_BODYCAST_H_
#define _PE_CORE_RIGIDBODY_BODYCAST_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/constraints/RigidBody.h>


namespace pe {

//=================================================================================================
//
//  RIGID BODY CAST OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Rigid body cast operators */
//@{
template< typename To, typename From > inline To* static_body_cast( From* body );
template< typename To, typename From > inline To* dynamic_body_cast( From* body );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Static cast for rigid bodies.
 *
 * \param body The rigid body to be cast.
 * \return The casted body.
 *
 * The static_body_cast function is used exactly as the built-in static_cast operator but
 * for rigid bodies.

   \code
   BodyID   body   = createSphere( 1, 0.0, 0.0, 0.0, 1.0, iron );
   SphereID sphere = static_body_cast<Sphere>( body );
   \endcode
 */
template< typename To, typename From >
inline To* static_body_cast( From* body )
{
   pe_CONSTRAINT_MUST_BE_RIGID_BODY_TYPE( From );
   pe_CONSTRAINT_MUST_BE_RIGID_BODY_TYPE( To   );
   return static_cast<To*>( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Dynamic cast for rigid bodies.
 *
 * \param body The rigid body to be cast.
 * \return The casted body.
 *
 * The dynamic_body_cast function is used exactly as the built-in dynamic_cast operator but
 * for rigid bodies.

   \code
   BodyID   body;
   SphereID sphere = dynamic_body_cast<Sphere>( body );
   \endcode
 */
template< typename To, typename From >
inline To* dynamic_body_cast( From* body )
{
   pe_CONSTRAINT_MUST_BE_RIGID_BODY_TYPE( From );
   pe_CONSTRAINT_MUST_BE_RIGID_BODY_TYPE( To   );
   return dynamic_cast<To*>( body );
}
//*************************************************************************************************

} // namespace pe

#endif
