//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsArithmetic.h
 *  \brief Header file for the IsArithmetic type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISARITHMETIC_H_
#define _PE_UTIL_TYPETRAITS_ISARITHMETIC_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_arithmetic.hpp>
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
/*!\brief Auxiliary helper struct for the IsArithmetic type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsArithmeticHelper
{
   //**********************************************************************************************
   enum { value = boost::is_arithmetic<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for arithmetic data types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is a (possibly cv-qualified)
 * arithmetic (integral or floating point) data type. In case the type is an arithmetic  type, the
 * \a value member enumeration is set to 1, the nested type definition \a Type is \a TrueType, and
 * the class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType
 * and the class derives from \a FalseType.

   \code
   class MyClass {};

   pe::IsArithmetic<int>::value         // Evaluates to 1
   pe::IsArithmetic<float const>::Type  // Results in TrueType
   pe::IsArithmetic<short volatile>     // Is derived from TrueType
   pe::IsArithmetic<void>::value        // Evaluates to 0
   pe::IsArithmetic<int*>::value        // Evaluates to 0
   pe::IsArithmetic<int&>::Type         // Results in FalseType
   pe::IsArithmetic<MyClass>            // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsArithmetic : public IsArithmeticHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsArithmeticHelper<T>::value };
   typedef typename IsArithmeticHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
