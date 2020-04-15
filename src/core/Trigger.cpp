//=================================================================================================
/*!
 *  \file src/core/Trigger.cpp
 *  \brief Trigger interface class
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

#include <algorithm>
#include <pe/core/Trigger.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

Trigger::Triggers Trigger::triggers_;




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the Trigger class.
 */
Trigger::Trigger()
{
   triggers_.pushBack( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the Trigger class.
 */
Trigger::~Trigger()
{
   Triggers::Iterator pos( std::find( triggers_.begin(), triggers_.end(), this ) );
   pe_INTERNAL_ASSERT( pos != triggers_.end(), "Trigger object is not registered!" );
   triggers_.erase( pos );
}
//*************************************************************************************************

} // namespace pe
