//=================================================================================================
/*!
 *  \file pe/core/attachable/AttachableType.h
 *  \brief Header file for the attachable types
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

#ifndef _PE_CORE_ATTACHABLE_ATTACHABLETYPE_H_
#define _PE_CORE_ATTACHABLE_ATTACHABLETYPE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>


namespace pe {

//=================================================================================================
//
//  ATTACHABLE TYPES
//
//=================================================================================================

//*************************************************************************************************
//! Type codes of the attachables.
enum AttachableType {
   gravityType  = 1,  //!< Code for the Gravity force generator.
   springType   = 2   //!< Code for the Spring force generator.
};
//*************************************************************************************************




//=================================================================================================
//
//  ATTACHABLE TYPE UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Attachable type utility functions */
//@{
std::ostream& operator<<( std::ostream& os, AttachableType type );
//@}
//*************************************************************************************************

} // namespace pe

#endif
