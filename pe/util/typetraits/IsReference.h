//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsReference.h
 *  \brief Header file for the IsReference type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISREFERENCE_H_
#define _PE_UTIL_TYPETRAITS_ISREFERENCE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_reference.hpp>
#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsReference type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsReferenceHelper
{
   //**********************************************************************************************
   enum { value = boost::is_reference<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time type check.
 * \ingroup type_traits
 *
 * This class tests whether the given template parameter \a T is a reference type (including
 * references to functions). If it is a reference type, the \a value member enumeration is
 * set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsReference<int&>::value             // Evaluates to 1
   pe::IsReference<int const&>::Type        // Results in TrueType
   pe::IsReference<int (&)(long)>           // Is derived from TrueType
   pe::IsReference<int>::value              // Evaluates to 0
   pe::IsReference<double*>::Type           // Results in FalseType
   pe::IsReference<int (MyClass::*)(long)>  // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsReference : public IsReferenceHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsReferenceHelper<T>::value };
   typedef typename IsReferenceHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
