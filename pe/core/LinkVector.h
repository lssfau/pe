//=================================================================================================
/*!
 *  \file pe/core/LinkVector.h
 *  \brief Implementation of a vector for link handles
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

#ifndef _PE_CORE_LINKVECTOR_H_
#define _PE_CORE_LINKVECTOR_H_


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

class Link;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for link handles.
 * \ingroup core
 *
 * The LinkVector class is a container class for link handles (pe::LinkID). It offers easy and
 * fast access to the contained links:

   \code
   pe::LinkVector links;

   // Calculating the total number of links contained in the link vector.
   pe::LinkVector::SizeType num = links.size();

   // Loop over all links contained in the link vector.
   pe::LinkVector::Iterator begin = links.begin();
   pe::LinkVector::Iterator end   = links.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<Link,NoDelete>  LinkVector;
//*************************************************************************************************

} // namespace pe

#endif
