//=================================================================================================
/*!
 *  \file pe/util/typetraits/HasSize.h
 *  \brief Header file for the HasSize type trait
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

#ifndef _PE_UTIL_TYPETRAITS_HASSIZE_H_
#define _PE_UTIL_TYPETRAITS_HASSIZE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time size check.
 * \ingroup type_traits
 *
 * This class offers the possibility to test the size of a type at compile time. If the type
 * \a T is exactly \a Size bytes large, the \a value member enumeration is set to 1, the nested
 * type definition \a Type is \a TrueType, and the class derives from \a TrueType. Otherwise
 * \a value is set to 0, \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   pe::HasSize<int,4>::value              // Evaluates to 1 (on most architectures)
   pe::HasSize<float,4>::Type             // Results in TrueType (on most architectures)
   pe::HasSize<const double,8>            // Is derived from TrueType (on most architectures)
   pe::HasSize<volatile double,2>::value  // Evaluates to 0
   pe::HasSize<const char,8>::Type        // Results in FalseType
   pe::HasSize<unsigned char,4>           // Is derived from FalseType
   \endcode
 */
template< typename T, size_t Size >
class HasSize : public SelectType< sizeof( T ) == Size, TrueType, FalseType >::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = ( sizeof( T ) == Size ) };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Partial specialization of the compile time size constraint.
 * \ingroup type_traits
 *
 * This class ia a partial specialization of the HasSize template for the type \a void. This
 * specialization assumes that an object of type \a void has a size of 0. Therefore \a value
 * is set to 1, \a Type is \a TrueType, and the class derives from \a TrueType only if the
 * \a Size template argument is 0. Otherwise \a value is set to 0, \a Type is \a FalseType,
 * and the class derives from \a FalseType.
 */
template< size_t Size >
class HasSize<void,Size> : public SelectType< 0 == Size, TrueType, FalseType >::Type
{
public:
   //**********************************************************************************************
   enum { value = ( 0 == Size ) };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Partial specialization of the compile time size constraint.
 * \ingroup type_traits
 *
 * This class ia a partial specialization of the HasSize template for constant \a void. This
 * specialization assumes that an object of type \a void has a size of 0. Therefore \a value
 * is set to 1, \a Type is \a TrueType, and the class derives from \a TrueType only if the
 * \a Size template argument is 0. Otherwise \a value is set to 0, \a Type is \a FalseType,
 * and the class derives from \a FalseType.
 */
template< size_t Size >
class HasSize<const void,Size> : public SelectType< 0 == Size, TrueType, FalseType >::Type
{
public:
   //**********************************************************************************************
   enum { value = ( 0 == Size ) };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Partial specialization of the compile time size constraint.
 * \ingroup type_traits
 *
 * This class ia a partial specialization of the HasSize template for volatile \a void. This
 * specialization assumes that an object of type \a void has a size of 0. Therefore \a value
 * is set to 1, \a Type is \a TrueType, and the class derives from \a TrueType only if the
 * \a Size template argument is 0. Otherwise \a value is set to 0, \a Type is \a FalseType,
 * and the class derives from \a FalseType.
 */
template< size_t Size >
class HasSize<volatile void,Size> : public SelectType< 0 == Size, TrueType, FalseType >::Type
{
public:
   //**********************************************************************************************
   enum { value = ( 0 == Size ) };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Partial specialization of the compile time size constraint.
 * \ingroup type_traits
 *
 * This class ia a partial specialization of the HasSize template for constant volatile \a void.
 * This specialization assumes that an object of type \a void has a size of 0. Therefore \a value
 * is set to 1, \a Type is \a TrueType, and the class derives from \a TrueType only if the \a Size
 * template argument is 0. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class
 * derives from \a FalseType.
 */
template< size_t Size >
class HasSize<const volatile void,Size> : public SelectType< 0 == Size, TrueType, FalseType >::Type
{
public:
   //**********************************************************************************************
   enum { value = ( 0 == Size ) };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
