//=================================================================================================
/*!
 *  \file src/core/attachable/AttachableBase.cpp
 *  \brief Source file for the AttachableBase class
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/AttachableBase.h>
#include <pe/core/MPI.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the AttachableBase class.
 *
 * \param type The type of the rigid body.
 * \param sid The unique system-specific ID of the attachable.
 */
AttachableBase::AttachableBase( AttachableType type, id_t sid )
   : type_  ( type )  // The type code of the attachable
   , sid_   ( sid  )  // The unique system-specific attachable ID
   , bodies_()        // Vector of attached rigid bodies
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the AttachableBase class.
 */
AttachableBase::~AttachableBase()
{}
//*************************************************************************************************

} // namespace pe
