//=================================================================================================
/*!
 *  \file pe/util/typetraits/IsBaseOf.h
 *  \brief Header file for the IsBaseOf type trait
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

#ifndef _PE_UTIL_TYPETRAITS_ISBASEOF_H_
#define _PE_UTIL_TYPETRAITS_ISBASEOF_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/remove_cv.hpp>
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
/*!\brief Auxiliary helper struct for the IsBaseOf type trait.
 * \ingroup type_traits
 */
template< typename Base, typename Derived >
struct IsBaseOfHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of<typename boost::remove_cv<Base>::type,
                                    typename boost::remove_cv<Derived>::type>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time analysis of an inheritance relationship.
 * \ingroup type_traits
 *
 * This type trait tests for an inheritance relationship between the two types \a Base and
 * \a Derived. If \a Derived is a type derived from \a Base or the same type as \a Base the
 * \a value member enumeration is set to 1, the nested type definition \a Type is \a TrueType,
 * and the class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is
 * \a FalseType, and the class derives from \a FalseType.

   \code
   class A { ... };
   class B : public A { ... };
   class C { ... };

   pe::IsBaseOf<A,B>::value  // Evaluates to 1
   pe::IsBaseOf<A,B>::Type   // Results in TrueType
   pe::IsBaseOf<A,B>         // Is derived from TrueType
   pe::IsBaseOf<A,C>::value  // Evaluates to 0
   pe::IsBaseOf<B,A>::Type   // Results in FalseType
   pe::IsBaseOf<B,A>         // Is derived from FalseType
   \endcode
 */
template< typename Base, typename Derived >
class IsBaseOf : public IsBaseOfHelper<Base,Derived>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsBaseOfHelper<Base,Derived>::value };
   typedef typename IsBaseOfHelper<Base,Derived>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
