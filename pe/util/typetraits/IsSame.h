//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsSame.h
 *  \brief Header file for the IsSame and IsStrictlySame type traits
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

#ifndef _PE_UTIL_TYPETRAITS_ISSAME_H_
#define _PE_UTIL_TYPETRAITS_ISSAME_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

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
/*!\brief Auxiliary helper struct for the IsSame type trait.
 * \ingroup type_traits
 */
template< typename A, typename B >
struct IsSameHelper
{
private:
   //**********************************************************************************************
   class No {};
   class Yes { No no[2]; };

   static A&  createA();
   static B&  createB();

   static Yes testA( const volatile B& );
   static No  testA( ... );
   static Yes testB( const volatile A& );
   static No  testB( ... );
   //**********************************************************************************************

public:
   //**********************************************************************************************
   enum { value = ( sizeof( testA( createA() ) ) == sizeof( Yes ) ) &&
                  ( sizeof( testB( createB() ) ) == sizeof( Yes ) ) };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Type relationship analysis.
 * \ingroup type_traits
 *
 * This class tests if the two data types \a A and \a B are equal. For this type comparison,
 * the cv-qualifiers of both data types are ignored. If \a A and \a B are the same data type
 * (ignoring the cv-qualifiers), then the \a value member enumeration is set to 1, the nested
 * type definition \a Type is \a TrueType, and the class derives from \a TrueType. Otherwise
 * \a value is set to 0, \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   pe::IsSame<int,int>::value               // Evaluates to 1
   pe::IsSame<int,const int>::Type          // Results in TrueType
   pe::IsSame<float,volatile float>         // Is derived from TrueType
   pe::IsSame<char,wchar_t>::value          // Evaluates to 0
   pe::IsSame<char,volatile float>::Type    // Results in FalseType
   pe::IsSame<int,double>                   // Is derived from FalseType
   \endcode
 */
template< typename A, typename B >
struct IsSame : public IsSameHelper<A,B>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsSameHelper<A,B>::value };
   typedef typename IsSameHelper<A,B>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time type relationship analysis.
 * \ingroup type_traits
 *
 * This class tests if the two data types \a A and \a B are equal. For this type comparison,
 * the cv-qualifiers of both data types are not ignored. If \a A and \a B are the same data
 * type, then the \a value member enumeration is set to 1, the nested type definition \a Type
 * is \a TrueType, and the class derives from \a TrueType. Otherwise \a value is set to 0,
 * \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   pe::IsStrictlySame<int,int>::value                   // Evaluates to 1
   pe::IsStrictlySame<const double,const double>::Type  // Results in TrueType
   pe::IsStrictlySame<volatile float,volatile float>    // Is derived from TrueType
   pe::IsStrictlySame<char,wchar_t>::value              // Evaluates to 0
   pe::IsStrictlySame<int,const int>::Type              // Results in FalseType
   pe::IsStrictlySame<float,volatile float>             // Is derived from FalseType
   \endcode
 */
template< typename A, typename B >
struct IsStrictlySame : public FalseType
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
//! Specialization of the IsStrictlySame class template for a single, matching data type.
template< typename T >
struct IsStrictlySame<T,T> : public TrueType
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
