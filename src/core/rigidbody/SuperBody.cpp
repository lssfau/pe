//=================================================================================================
/*!
 *  \file src/core/rigidbody/SuperBody.cpp
 *  \brief Source file for the SuperBody class
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/MPI.h>
#include <pe/core/rigidbody/SuperBody.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the SuperBody class.
 *
 * \param type The geometry type of the superordinate body.
 * \param finite Specifies if the superordinate body is finite or not.
 * \param visible Specifies if the superordinate body is visible or not.
 * \param sid The system-specific ID of the superordinate body.
 * \param uid The user-specific ID of the superordinate body.
 */
SuperBody::SuperBody( GeomType type, bool finite, bool visible, id_t sid, id_t uid )
   : RigidBody(type,finite,visible,sid,uid)  // Initializing the base object
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the SuperBody class.
 */
SuperBody::~SuperBody()
{}
//*************************************************************************************************

} // namespace pe
