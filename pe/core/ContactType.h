//=================================================================================================
/*!
 *  \file pe/core/ContactType.h
 *  \brief Header file for the contact types
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

#ifndef _PE_CORE_CONTACTTYPE_H_
#define _PE_CORE_CONTACTTYPE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>


namespace pe {

//=================================================================================================
//
//  CONTACT TYPES
//
//=================================================================================================

//*************************************************************************************************
//! Classification of contacts.
/*! Contacts between rigid bodies are classified depending on the relative velocity between
 *  the touching rigid bodies:
 *
 *   - \f$ v_{rel} \f$ > 0: separating contact
 *   - \f$ v_{rel} \f$ = 0: resting contact
 *   - \f$ v_{rel} \f$ < 0: colliding contact\n
 *
 * (in order to classify the contact, pe::collisionThreshold is used as tolerance level).
 */
enum ContactType {
   colliding  = 0,  //!< Colliding contacts (vrel < 0).
   resting    = 1,  //!< Resting contacts (vrel = 0).
   separating = 2   //!< Separating contacts (vrel > 0).
};
//*************************************************************************************************




//=================================================================================================
//
//  CONTACT TYPE UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Contact type utility functions */
//@{
std::ostream& operator<<( std::ostream& os, ContactType type );
//@}
//*************************************************************************************************

} // namespace pe

#endif
