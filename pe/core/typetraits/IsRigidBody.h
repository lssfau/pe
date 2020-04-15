//=================================================================================================
/*!
 *  \file pe/core/typetraits/IsRigidBody.h
 *  \brief Header file for the IsRigidBody type trait
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

#ifndef _PE_CORE_TYPETRAITS_ISRIGIDBODY_H_
#define _PE_CORE_TYPETRAITS_ISRIGIDBODY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_base_of.hpp>
#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

class RigidBody;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsRigidBody type trait.
 * \ingroup core_type_traits
 */
template< typename T >
struct IsRigidBodyHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of< RigidBody, T >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for rigid body types.
 * \ingroup core_type_traits
 *
 * This type trait tests whether or not the given template parameter is a rigid body type (i.e.,
 * is inheritance related to the RigidBody class, as for instance Sphere, Box, Capsule, ...). In
 * case the type is a (possibly cv-qualified) rigid body type, the \a value member enumeration
 * is set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsRigidBody< Sphere >::value        // Evaluates to 1
   pe::IsRigidBody< const Box >::Type      // Results in TrueType
   pe::IsRigidBody< volatile Union >       // Is derived from TrueType
   pe::IsRigidBody< Vec3 >::value          // Evaluates to 0
   pe::IsRigidBody< const Contact >::Type  // Results in FalseType
   pe::IsRigidBody< volatile World >       // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsRigidBody : public IsRigidBodyHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsRigidBodyHelper<T>::value };
   typedef typename IsRigidBodyHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
