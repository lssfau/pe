//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsPointer.h
 *  \brief Header file for the IsPointer type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISPOINTER_H_
#define _PE_UTIL_TYPETRAITS_ISPOINTER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_pointer.hpp>
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
/*!\brief Auxiliary helper struct for the IsPointer type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsPointerHelper
{
   //**********************************************************************************************
   enum { value = boost::is_pointer<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time type check.
 * \ingroup type_traits
 *
 * This class tests whether the given template parameter is a pointer type (including function
 * pointers, but excluding pointers to members) or not. If it is a pointer type, the \a value
 * member enumeration is set to 1, the nested type definition \a Type is \a TrueType, and the
 * class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType,
 * and the class derives from \a FalseType.

   \code
   pe::IsPointer<char* const>::value      // Evaluates to 1
   pe::IsPointer<volatile float*>::Type   // Results in TrueType
   pe::IsPointer<int (*)(long)>           // Is derived from TrueType
   pe::IsPointer<int>::value              // Evaluates to 0
   pe::IsPointer<int MyClass::*>::Type    // Results in FalseType
   pe::IsPointer<int (MyClass::*)(long)>  // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsPointer : public IsPointerHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsPointerHelper<T>::value };
   typedef typename IsPointerHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
