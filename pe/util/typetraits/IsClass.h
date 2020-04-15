//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsClass.h
 *  \brief Header file for the IsClass type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISCLASS_H_
#define _PE_UTIL_TYPETRAITS_ISCLASS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_class.hpp>
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
/*!\brief Auxiliary helper struct for the IsClass type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsClassHelper
{
   //**********************************************************************************************
   enum { value = boost::is_class<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time type check.
 * \ingroup type_traits
 *
 * The IsClass type trait tests whether or not the given template parameter is a (possibly
 * cv-qualified) class type. In case the given data type is a class type, the \a value member
 * enumeration is set to 1, the nested type definition \a Type is set to \a TrueType, and the
 * class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType
 * and the class derives from \a FalseType.

   \code
   class MyClass {};

   pe::IsClass<MyClass>::value        // Evaluates to 1
   pe::IsClass<MyClass const>::Type   // Results in TrueType
   pe::IsClass<std::string volatile>  // Is derived from TrueType
   pe::IsClass<int>::value            // Evaluates to 0 (int is a built-in data type)
   pe::IsClass<MyClass&>::Type        // Results in FalseType
   pe::IsClass<MyClass*>              // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsClass : public IsClassHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsClassHelper<T>::value };
   typedef typename IsClassHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
