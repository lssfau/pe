//=================================================================================================
/*!
 *  \file src/core/attachable/Attachable.cpp
 *  \brief Source file for the Attachable class
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

#include <pe/core/attachable/Attachable.h>
#include <pe/core/attachable/AttachableStorage.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/Configuration.h>
#include <pe/core/MPI.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the Attachable class.
 *
 * \param type The type of the rigid body.
 * \param sid The unique system-specific ID of the attachable.
 */
Attachable::Attachable( AttachableType type, id_t sid )
   : Parent( type, sid )  // Initialization of the parent class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Attachable class.
 */
Attachable::~Attachable()
{
}
//*************************************************************************************************




//=================================================================================================
//
//  MPI FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\fn bool Attachable::isRemote() const
 * \brief Returns whether the attachable is remote or not.
 *
 * \return \a true in case the attachable is remote, \a false if not.
 *
 * This function returns whether the attachable is remote or not. In case the attachable is
 * attached to at least a single local rigid body the function returns \a false. Otherwise
 * it returns \a true.
 */
//*************************************************************************************************




//=================================================================================================
//
//  ATTACHABLE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Detaches the given attachable.
 * \ingroup core
 *
 * \param attachable The detachable to be detached.
 */
PE_PUBLIC void detach( AttachableID attachable )
{
   typedef AttachableStorage<Config> AS;

   // WARNING: Using friend relationship to get non-constant reference of attachable storage.
   AS& attachablestorage( theCollisionSystem()->attachablestorage_ );

   // Removing the attachable from the attachable storage
   const AS::Iterator pos( attachablestorage.find( attachable ) );
   attachablestorage.remove( pos );

   delete attachable;
}
//*************************************************************************************************

} // namespace pe
