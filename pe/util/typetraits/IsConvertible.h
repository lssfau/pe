//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsConvertible.h
 *  \brief Header file for the IsConvertible type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISCONVERTIBLE_H_
#define _PE_UTIL_TYPETRAITS_ISCONVERTIBLE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_convertible.hpp>
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
/*!\brief Auxiliary helper struct for the IsConvertible type trait.
 * \ingroup type_traits
 */
template< typename From, typename To >
struct IsConvertibleHelper
{
   //**********************************************************************************************
   enum { value = boost::is_convertible<From,To>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time pointer relationship constraint.
 * \ingroup type_traits
 *
 * This type traits tests whether the first given template argument can be converted to the
 * second template argument via copy construction. If the first argument can be converted to
 * the second argument, the \a value member enumeration is set to 1, the nested type definition
 * \a type is \a TrueType, and the class derives from \a TrueType. Otherwise \a value is set
 * to 0, \a type is \a FalseType, and the class derives from \a FalseType.

   \code
   struct A {};
   struct B : public A {};

   struct C {};
   struct D {
      D( const C& c ) {}
   };

   pe::IsConvertible<int,unsigned int>::value    // Evaluates to 1
   pe::IsConvertible<float,const double>::value  // Evaluates to 1
   pe::IsConvertible<B,A>::Type                  // Results in TrueType
   pe::IsConvertible<B*,A*>::Type                // Results in TrueType
   pe::IsConvertible<C,D>                        // Is derived from TrueType
   pe::IsConvertible<char*,std::string>          // Is derived from TrueType
   pe::IsConvertible<std::string,char*>::value   // Evaluates to 0
   pe::IsConvertible<A,B>::Type                  // Results in FalseType
   pe::IsConvertible<A*,B*>                      // Is derived from FalseType
   \endcode
 */
template< typename From, typename To >
struct IsConvertible : public IsConvertibleHelper<From,To>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsConvertibleHelper<From,To>::value };
   typedef typename IsConvertibleHelper<From,To>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
