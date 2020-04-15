//=================================================================================================
/*!
 *  \file pe/core/rigidbody/GeomVector.h
 *  \brief Implementation of a vector for geometric primitives
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

#ifndef _PE_CORE_RIGIDBODY_GEOMVECTOR_H_
#define _PE_CORE_RIGIDBODY_GEOMVECTOR_H_


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

class GeomPrimitive;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of a vector for geometric primitives.
 * \ingroup core
 *
 * The GeomVector class is a specialization of the PtrVector class for geometric primitives.
 * These specializations for the geometric types of the core (Sphere, Box, Capsule, Plane)
 * guarantee the maximum performance for the templated version of the size function and both
 * cast iterators (CastIterator and ConstCastIterator) when used with one of the geometric
 * core types.

   \code
   pe::GeomVector primitives;

   // Calculating the total number of spheres contained in the primitive vector.
   // Due to the specializations of GeomVector, this functions is much faster than the
   // standard version of size.
   pe::GeomVector::SizeType spheres = primitives.size<Sphere>();

   // Loop over all spheres contained in the geometric primitive vector.
   // Additionally the CastIterator and ConstCastIterator of GeomVector are optimized for
   // the use with the geometric types of the core module (Sphere, Box, Capsule, Plane).
   pe::GeomVector::CastIterator<Sphere> begin = primitives.begin<Sphere>();
   pe::GeomVector::CastIterator<Sphere> end   = primitives.end<Sphere>();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<GeomPrimitive,NoDelete>  GeomVector;
//*************************************************************************************************

} // namespace pe

#endif
