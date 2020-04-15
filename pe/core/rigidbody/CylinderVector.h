//=================================================================================================
/*!
 *  \file pe/core/rigidbody/CylinderVector.h
 *  \brief Implementation of a vector for cylinder handles
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

#ifndef _PE_CORE_RIGIDBODY_CYLINDERVECTOR_H_
#define _PE_CORE_RIGIDBODY_CYLINDERVECTOR_H_


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

class Cylinder;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for cylinder handles.
 * \ingroup core
 *
 * The CylinderVector class is a special PtrVector for cylinder handles (pe::CylinderID). It
 * offers easy and fast access to the contained cylinders:

   \code
   pe::CylinderVector cylinders;

   // Calculating the total number of cylinders contained in the cylinder vector.
   pe::CylinderVector::SizeType num = cylinders.size();

   // Loop over all cylinders contained in the cylinder vector.
   pe::CylinderVector::Iterator begin = cylinders.begin();
   pe::CylinderVector::Iterator end   = cylinders.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<Cylinder,NoDelete>  CylinderVector;
//*************************************************************************************************

} // namespace pe

#endif
