//=================================================================================================
/*!
 *  \file pe/core/rigidbody/UnionVector.h
 *  \brief Implementation of a vector for union handles
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

#ifndef _PE_CORE_RIGIDBODY_UNIONVECTOR_H_
#define _PE_CORE_RIGIDBODY_UNIONVECTOR_H_


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

class Union;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for union handles.
 * \ingroup core
 *
 * The UnionVector class is a special PtrVector for union handles (pe::UnionID). It offers
 * easy and fast access to the contained unions:

   \code
   pe::UnionVector unions;

   // Calculating the total number of unions contained in the union vector.
   pe::UnionVector::SizeType num = unions.size();

   // Loop over all unions contained in the union vector.
   pe::UnionVector::Iterator begin = unions.begin();
   pe::UnionVector::Iterator end   = unions.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<Union,NoDelete>  UnionVector;
//*************************************************************************************************

} // namespace pe

#endif
