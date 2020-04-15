//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsInteger.h
 *  \brief Header file for the IsInteger type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISINTEGER_H_
#define _PE_UTIL_TYPETRAITS_ISINTEGER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/FalseType.h>
#include <pe/util/TrueType.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time check for integer types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is an integer type (i.e.,
 * either (signed) int or unsigned int, possibly cv-qualified). In case the type is an integer
 * type (ignoring the cv-qualifiers), the \a value member enumeration is set to 1, the nested
 * type definition \a Type is \a TrueType, and the class derives from \a TrueType. Otherwise
 * \a value is set to 0, \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   pe::IsInteger<int>::value                 // Evaluates to 1
   pe::IsInteger<const unsigned int>::Type   // Results in TrueType
   pe::IsInteger<const volatile signed int>  // Is derived from TrueType
   pe::IsInteger<unsigned short>::value      // Evaluates to 0
   pe::IsInteger<const long>::Type           // Results in FalseType
   pe::IsInteger<volatile float>             // Is derived from FalseType
   \endcode

 * Note the difference between the IsInteger and IsIntegral type traits: Whereas the IsInteger
 * type trait specifically tests whether the given data type is either int or unsigned int
 * (possibly cv-qualified), the IsIntegral type trait tests whether the given template argument
 * is an integral data type (char, short, int, long, etc.).
 */
template< typename T >
struct IsInteger : public FalseType
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = 0 };
   typedef FalseType  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for the plain 'int' type.
template<>
struct IsInteger<int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for 'const int'.
template<>
struct IsInteger<const int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for 'volatile int'.
template<>
struct IsInteger<volatile int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for 'const volatile int'.
template<>
struct IsInteger<const volatile int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for the plain 'unsigned int' type.
template<>
struct IsInteger<unsigned int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for 'const unsigned int'.
template<>
struct IsInteger<const unsigned int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for 'volatile unsigned int'.
template<>
struct IsInteger<volatile unsigned int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsInteger type trait for 'const volatile unsigned int'.
template<>
struct IsInteger<const volatile unsigned int> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
