//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsSigned.h
 *  \brief Header file for the IsSigned type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISSIGNED_H_
#define _PE_UTIL_TYPETRAITS_ISSIGNED_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_signed.hpp>
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
/*!\brief Auxiliary helper struct for the IsSigned type trait.
 * \ingroup type_traits
 */
template< typename T >
struct IsSignedHelper
{
   //**********************************************************************************************
   enum { value = boost::is_signed<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\struct pe::IsSigned
 * \brief Compile time check for signed data types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is a signed integral data
 * type. In case the type is a signed (possibly cv-qualified) data type, the \a value member
 * enumeration is set to 1, the nested type definition \a Type is \a TrueType, and the class
 * derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and
 * the class derives from \a FalseType.

   \code
   class MyClass {};

   pe::IsSigned<short>::value         // Evaluates to 1
   pe::IsSigned<const int>::type      // Results in TrueType
   pe::IsSigned<volatile long>        // Is derived from TrueType
   pe::IsSigned<unsigned int>::value  // Evaluates to 0
   pe::IsSigned<const float>::type    // Results in FalseType
   pe::IsSigned<MyClass>              // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsSigned : public IsSignedHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsSignedHelper<T>::value };
   typedef typename IsSignedHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
