//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsConst.h
 *  \brief Header file for the IsConst type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISCONST_H_
#define _PE_UTIL_TYPETRAITS_ISCONST_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_const.hpp>
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
/*!\brief Auxiliary helper struct for the IsConst type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsConstHelper
{
   //**********************************************************************************************
   enum { value = boost::is_const<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for constant data types.
 * \ingroup type_traits
 *
 * The IsConst type trait tests whether or not the given template parameter is a (top level)
 * const-qualified data type. In case the given data type is const-qualified, the \a value
 * member enumeration is set to 1, the nested type definition \a Type is \a TrueType, and
 * the class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType,
 * and the class derives from \a FalseType.

   \code
   pe::IsConst<const int>::value          // Evaluates to 1
   pe::IsConst<const volatile int>::Type  // Results in TrueType
   pe::IsConst<int* const>                // Is derived from TrueType
   pe::IsConst<int>::value                // Evaluates to 0
   pe::IsConst<const int*>::Type          // Results in FalseType (the const-qualifier is not at the top level)
   pe::IsConst<const int* volatile>       // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsConst : public IsConstHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsConstHelper<T>::value };
   typedef typename IsConstHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
