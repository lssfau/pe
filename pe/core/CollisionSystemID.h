//=================================================================================================
/*!
 *  \file pe/core/CollisionSystemID.h
 *  \brief Implementation of a smart CollisionSystem handle
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

#ifndef _PE_CORE_COLLISIONSYSTEMID_H_
#define _PE_CORE_COLLISIONSYSTEMID_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/shared_ptr.hpp>
#include <pe/core/Configuration.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename > class CollisionSystem;




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Handle for the collision system.
 * \ingroup core
 */
typedef boost::shared_ptr< CollisionSystem<Config> >  CollisionSystemID;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Handle for the constant collision system.
 * \ingroup core
 */
typedef boost::shared_ptr< const CollisionSystem<Config> >  ConstCollisionSystemID;
//*************************************************************************************************

} // namespace pe

#endif
