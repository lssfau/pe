//=================================================================================================
/*!
 *  \file pe/core/rigidbody/CapsuleVector.h
 *  \brief Implementation of a vector for capsule handles
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

#ifndef _PE_CORE_RIGIDBODY_CAPSULEVECTOR_H_
#define _PE_CORE_RIGIDBODY_CAPSULEVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

class Capsule;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for capsule handles.
 * \ingroup core
 *
 * The CapsuleVector class is a special PtrVector for capsule handles (pe::CapsuleID). It offers
 * easy and fast access to the contained capsules:

   \code
   pe::CapsuleVector capsules;

   // Calculating the total number of capsules contained in the capsule vector.
   pe::CapsuleVector::SizeType num = capsules.size();

   // Loop over all capsules contained in the capsule vector.
   pe::CapsuleVector::Iterator begin = capsules.begin();
   pe::CapsuleVector::Iterator end   = capsules.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<Capsule,NoDelete>  CapsuleVector;
//*************************************************************************************************

} // namespace pe

#endif
