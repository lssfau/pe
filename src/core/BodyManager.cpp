//=================================================================================================
/*!
 *  \file src/core/BodyManager.cpp
 *  \brief Source file for the BodyManager class
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

#include <pe/core/BodyManager.h>
#include <pe/core/MPI.h>
#include <pe/core/World.h>
#include <pe/util/Null.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

ManagerID BodyManager::default_( NULL );




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the BodyManager class.
 */
BodyManager::~BodyManager()
{
   // In case the currently active default manager is destroyed,
   // the default manager is reset to NULL
   if( default_ == this )
      default_ = NULL;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BODY MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a handle to the currently active default body manager.
 * \ingroup core
 *
 * \return Handle to the default body manager.
 *
 * This function returns a handle to the currently active default body manager. Per default,
 * all newly created rigid bodies are added to this manager.
 */
ManagerID theDefaultManager()
{
   if( BodyManager::default_ == NULL )
      BodyManager::default_ = theWorld().get();
   return BodyManager::default_;
}
//*************************************************************************************************

} // namespace pe
