//=================================================================================================
/*!
 *  \file pe/core/typetraits/IsUnion.h
 *  \brief Header file for the IsUnion type trait
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

#ifndef _PE_CORE_TYPETRAITS_ISUNION_H_
#define _PE_CORE_TYPETRAITS_ISUNION_H_


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

class Union;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsUnion type trait.
 * \ingroup core_type_traits
 */
template< typename T >
struct IsUnionHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of< Union, T >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for union compound geometry types.
 * \ingroup core_type_traits
 *
 * This type trait tests whether or not the given template parameter is a union compound
 * geometry type (i.e., is inheritance related to class Union). In case the type is a
 * (possibly cv-qualified) union type, the \a value member enumeration is set to 1, the
 * nested type definition \a Type is \a TrueType, and the class derives from \a TrueType.
 * Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives from
 * \a FalseType.

   \code
   pe::IsUnion< Union >::value       // Evaluates to 1
   pe::IsUnion< const Union >::Type  // Results in TrueType
   pe::IsUnion< volatile Union >     // Is derived from TrueType
   pe::IsUnion< Sphere >::value      // Evaluates to 0
   pe::IsUnion< const Plane >::Type  // Results in FalseType
   pe::IsUnion< volatile Box >       // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsUnion : public IsUnionHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsUnionHelper<T>::value };
   typedef typename IsUnionHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
