//=================================================================================================
/*!
 *  \file pe/core/rigidbody/PlaneVector.h
 *  \brief Implementation of a vector for plane handles
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

#ifndef _PE_CORE_RIGIDBODY_PLANEVECTOR_H_
#define _PE_CORE_RIGIDBODY_PLANEVECTOR_H_


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

class Plane;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for plane handles.
 * \ingroup core
 *
 * The PlaneVector class is a special PtrVector for plane handles (pe::PlaneID). It offers
 * easy and fast access to the contained planes:

   \code
   pe::PlaneVector planes;

   // Calculating the total number of planes contained in the plane vector.
   pe::PlaneVector::SizeType num = planes.size();

   // Loop over all planes contained in the plane vector.
   pe::PlaneVector::Iterator begin = planes.begin();
   pe::PlaneVector::Iterator end   = planes.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<Plane,NoDelete>  PlaneVector;
//*************************************************************************************************

} // namespace pe

#endif
