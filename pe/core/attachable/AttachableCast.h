//=================================================================================================
/*!
 *  \file pe/core/attachable/AttachableCast.h
 *  \brief Cast operators for attachables
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

#ifndef _PE_CORE_ATTACHABLE_ATTACHABLECAST_H_
#define _PE_CORE_ATTACHABLE_ATTACHABLECAST_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/constraints/Attachable.h>


namespace pe {

//=================================================================================================
//
//  ATTACHABLE CAST OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Attachable cast operators */
//@{
template< typename To, typename From > inline To* static_attachable_cast( From* attachable );
template< typename To, typename From > inline To* dynamic_attachable_cast( From* attachable );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Static cast for attachables.
 *
 * \param attachable The attachable to be cast.
 * \return The casted attachable.
 *
 * The static_attachable_cast function is used exactly as the built-in static_cast operator
 * but for attachables.

   \code
   SphereID     sphere     = createSphere ( 1, 0.0, 0.0, 0.0, 1.0, iron );
   AttachableID attachable = attachGravity( sphere, 0.0, 0.0, -9.81 );
   GravityID    gravity    = static_attachable_cast<Gravity>( attachable );
   \endcode
 */
template< typename To, typename From >
inline To* static_attachable_cast( From* attachable )
{
   pe_CONSTRAINT_MUST_BE_ATTACHABLE_TYPE( From );
   pe_CONSTRAINT_MUST_BE_ATTACHABLE_TYPE( To   );
   return static_cast<To*>( attachable );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Dynamic cast for attachables.
 *
 * \param attachable The attachable to be cast.
 * \return The casted attachable.
 *
 * The dynamic_attachable_cast function is used exactly as the built-in dynamic_cast operator
 * but for attachables.

   \code
   AttachableID attachable;
   GravityID gravity = dynamic_attachable_cast<Gravity>( attachable );
   \endcode
 */
template< typename To, typename From >
inline To* dynamic_attachable_cast( From* attachable )
{
   pe_CONSTRAINT_MUST_BE_ATTACHABLE_TYPE( From );
   pe_CONSTRAINT_MUST_BE_ATTACHABLE_TYPE( To   );
   return dynamic_cast<To*>( attachable );
}
//*************************************************************************************************

} // namespace pe

#endif
