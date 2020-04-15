//=================================================================================================
/*!
 *  \file pe/core/rigidbody/BoxVector.h
 *  \brief Implementation of a vector for box handles
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

#ifndef _PE_CORE_RIGIDBODY_BOXVECTOR_H_
#define _PE_CORE_RIGIDBODY_BOXVECTOR_H_


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

class Box;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for box handles.
 * \ingroup core
 *
 * The BoxVector class is a special PtrVector for box handles (pe::BoxID). It offers easy and
 * fast access to the contained boxes:

   \code
   pe::BoxVector boxes;

   // Calculating the total number of boxes contained in the box vector.
   pe::BoxVector::SizeType num = boxes.size();

   // Loop over all boxes contained in the box vector.
   pe::BoxVector::Iterator begin = boxes.begin();
   pe::BoxVector::Iterator end   = boxes.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<Box,NoDelete>  BoxVector;
//*************************************************************************************************

} // namespace pe

#endif
