//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsNumeric.h
 *  \brief Header file for the IsNumeric type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISNUMERIC_H_
#define _PE_UTIL_TYPETRAITS_ISNUMERIC_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>
#include <pe/util/typetraits/IsBoolean.h>
#include <pe/util/typetraits/IsBuiltin.h>
#include <pe/util/typetraits/IsVoid.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsNumeric type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsNumericHelper
{
   //**********************************************************************************************
   enum { value = IsBuiltin<T>::value && !IsBoolean<T>::value && !IsVoid<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for numeric types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is a numeric type (integral
 * or floating point). In case the type is a numeric type, the \a value member enumeration is
 * set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsNumeric<int>::value         // Evaluates to 1 (int is a numeric data type)
   pe::IsNumeric<const float>::Type  // Results in TrueType (float is a numeric data type)
   pe::IsNumeric<volatile double>    // Is derived from TrueType (double is a numeric data type)
   pe::IsNumeric<void>::value        // Evaluates to 0 (void is not a numeric data type)
   pe::IsNumeric<bool>::Type         // Results in FalseType (bool is not a numeric data type)
   pe::IsNumeric<const bool>         // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsNumeric : public IsNumericHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsNumericHelper<T>::value };
   typedef typename IsNumericHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
