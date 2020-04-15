//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsCharacter.h
 *  \brief Header file for the IsCharacter type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISCHARACTER_H_
#define _PE_UTIL_TYPETRAITS_ISCHARACTER_H_


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
/*!\brief Compile time check for character types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is a character type
 * (i.e., either char, signed char, unsigned char, or wchar_t, possibly cv-qualified). In
 * case the type is a character type (ignoring the cv-qualifiers), the \a value member
 * enumeration is set to 1, the nested type definition \a Type is \a TrueType, and the
 * class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType,
 * and the class derives from \a FalseType.

   \code
   pe::IsCharacter<char>::value                // Evaluates to 1
   pe::IsCharacter<const unsigned char>::Type  // Results in TrueType
   pe::IsCharacter<const volatile wchar_t>     // Is derived from TrueType
   pe::IsCharacter<unsigned short>::value      // Evaluates to 0
   pe::IsCharacter<const int>::Type            // Results in FalseType
   pe::IsCharacter<volatile long>              // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsCharacter : public FalseType
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
//! Specialization of the IsCharacter type trait for the plain 'char' type.
template<>
struct IsCharacter<char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const char'.
template<>
struct IsCharacter<const char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'volatile char'.
template<>
struct IsCharacter<volatile char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const volatile char'.
template<>
struct IsCharacter<const volatile char> : public TrueType
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
//! Specialization of the IsCharacter type trait for the plain 'signed char' type.
template<>
struct IsCharacter<signed char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const signed char'.
template<>
struct IsCharacter<const signed char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'volatile signed char'.
template<>
struct IsCharacter<volatile signed char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const volatile signed char'.
template<>
struct IsCharacter<const volatile signed char> : public TrueType
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
//! Specialization of the IsCharacter type trait for the plain 'unsigned char' type.
template<>
struct IsCharacter<unsigned char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const unsigned char'.
template<>
struct IsCharacter<const unsigned char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'volatile unsigned char'.
template<>
struct IsCharacter<volatile unsigned char> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const volatile unsigned char'.
template<>
struct IsCharacter<const volatile unsigned char> : public TrueType
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
//! Specialization of the IsCharacter type trait for the plain 'wchar_t' type.
template<>
struct IsCharacter<wchar_t> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const wchar_t'.
template<>
struct IsCharacter<const wchar_t> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'volatile wchar_t'.
template<>
struct IsCharacter<volatile wchar_t> : public TrueType
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
//! Specialization of the IsCharacter type trait for 'const volatile wchar_t'.
template<>
struct IsCharacter<const volatile wchar_t> : public TrueType
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
