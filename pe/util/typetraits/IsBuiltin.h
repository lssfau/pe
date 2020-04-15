//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsBuiltin.h
 *  \brief Header file for the IsBuiltin type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISBUILTIN_H_
#define _PE_UTIL_TYPETRAITS_ISBUILTIN_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_fundamental.hpp>
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
/*!\brief Auxiliary helper struct for the IsBuiltin type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsBuiltinHelper
{
   //**********************************************************************************************
   enum { value = boost::is_fundamental<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for built-in data types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is a built-in/fundamental
 * data type. In case the type is a built-in type, the \a value member enumeration is set to 1,
 * the nested type definition \a Type is \a TrueType, and the class derives from \a TrueType.
 * Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives from
 * \a FalseType.

   \code
   pe::IsBuiltin<void>::value         // Evaluates to 1
   pe::IsBuiltin<float const>::Type   // Results in TrueType
   pe::IsBuiltin<short volatile>      // Is derived from TrueType
   pe::IsBuiltin<std::string>::value  // Evaluates to 0
   pe::IsBuiltin<int*>::Type          // Results in FalseType
   pe::IsBuiltin<int&>                // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsBuiltin : public IsBuiltinHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsBuiltinHelper<T>::value };
   typedef typename IsBuiltinHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
