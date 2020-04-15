//=================================================================================================
/*!
 *  \file pe/core/rigidbody/SphereVector.h
 *  \brief Implementation of a vector for sphere handles
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

#ifndef _PE_CORE_RIGIDBODY_SPHEREVECTOR_H_
#define _PE_CORE_RIGIDBODY_SPHEREVECTOR_H_


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

class Sphere;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a vector for sphere handles.
 * \ingroup core
 *
 * The SphereVector class is a special PtrVector for sphere handles (pe::SphereID). It offers
 * easy and fast access to the contained spheres:

   \code
   pe::SphereVector spheres;

   // Calculating the total number of spheres contained in the sphere vector.
   pe::SphereVector::SizeType num = spheres.size();

   // Loop over all spheres contained in the sphere vector.
   pe::SphereVector::Iterator begin = spheres.begin();
   pe::SphereVector::Iterator end   = spheres.end();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<Sphere,NoDelete>  SphereVector;
//*************************************************************************************************

} // namespace pe

#endif
