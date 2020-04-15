//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsIntegral.h
 *  \brief Header file for the IsIntegral type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISINTEGRAL_H_
#define _PE_UTIL_TYPETRAITS_ISINTEGRAL_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_integral.hpp>
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
/*!\brief Auxiliary helper struct for the IsIntegral type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsIntegralHelper
{
   //**********************************************************************************************
   enum { value = boost::is_integral<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for integral data types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is an integral data
 * type. In case the type is an integral data type, the \a value member enumeration is set
 * to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class
 * derives from \a FalseType.

   \code
   pe::IsIntegral<int>::value            // Evaluates to 1
   pe::IsIntegral<const char>::Type      // Results in TrueType (char is an integral data type)
   pe::IsIntegral<volatile short>        // Is derived from TrueType
   pe::IsIntegral<float>::value          // Evaluates to 0
   pe::IsIntegral<const double>::Type    // Results in FalseType
   pe::IsIntegral<volatile long double>  // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsIntegral : public IsIntegralHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsIntegralHelper<T>::value };
   typedef typename IsIntegralHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
