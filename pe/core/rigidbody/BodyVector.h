//=================================================================================================
/*!
 *  \file pe/core/rigidbody/BodyVector.h
 *  \brief Implementation of a vector for (polymorphic) rigid body pointers
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

#ifndef _PE_CORE_RIGIDBODY_BODYVECTOR_H_
#define _PE_CORE_RIGIDBODY_BODYVECTOR_H_


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

class RigidBody;




//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of a vector for (polymorphic) rigid body pointers.
 * \ingroup core
 *
 * The BodyVector class is a specialization of the PtrVector class for polymorphic rigid body
 * pointers. These specializations for the geometric types of the core (Sphere, Box, Capsule,
 * Plane, Union) guarantee the maximum performance for the templated version of the size
 * function and both cast iterators (CastIterator and ConstCastIterator) when used with one
 * of the geometric core types.

   \code
   pe::BodyVector bodies;

   // Calculating the total number of spheres contained in the body vector.
   // Due to the specializations of BodyVector, this functions is much faster than the
   // standard version of size.
   pe::BodyVector::SizeType spheres = bodies.size<Sphere>();

   // Loop over all spheres contained in the body vector.
   // Additionally the CastIterator and ConstCastIterator of BodyVector are optimized for
   // the use with the geometric types of the core module (Sphere, Box, Capsule, Plane, Union).
   pe::BodyVector::CastIterator<Sphere> begin = bodies.begin<Sphere>();
   pe::BodyVector::CastIterator<Sphere> end   = bodies.end<Sphere>();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
typedef PtrVector<RigidBody,NoDelete>  BodyVector;
//*************************************************************************************************

} // namespace pe

#endif
