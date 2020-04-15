//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsDouble.h
 *  \brief Header file for the IsDouble type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISDOUBLE_H_
#define _PE_UTIL_TYPETRAITS_ISDOUBLE_H_


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
/*!\brief Compile time check for double precision floating point types.
 * \ingroup type_traits
 *
 * This type trait tests whether or not the given template parameter is of double type. In
 * case the type is double (ignoring the cv-qualifiers), the \a value member enumeration is
 * set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class
 * derives from \a FalseType.

   \code
   pe::IsDouble<double>::value          // Evaluates to 1
   pe::IsDouble<const double>::Type     // Results in TrueType
   pe::IsDouble<const volatile double>  // Is derived from TrueType
   pe::IsDouble<float>::value           // Evaluates to 0
   pe::IsDouble<const int>::Type        // Results in FalseType
   pe::IsDouble<volatile short>         // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsDouble : public FalseType
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
//! Specialization of the IsDouble type trait for the plain 'double' type.
template<>
struct IsDouble<double> : public TrueType
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
//! Specialization of the IsDouble type trait for 'const double'.
template<>
struct IsDouble<const double> : public TrueType
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
//! Specialization of the IsDouble type trait for 'volatile double'.
template<>
struct IsDouble<volatile double> : public TrueType
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
//! Specialization of the IsDouble type trait for 'const volatile double'.
template<>
struct IsDouble<const volatile double> : public TrueType
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
