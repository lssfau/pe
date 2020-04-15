//=================================================================================================
/*!
 *  \file pe/util/constraints/Subscriptable.h
 *  \brief Constraint on the data type
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

#ifndef _PE_UTIL_CONSTRAINTS_SUBSCRIPTABLE_H_
#define _PE_UTIL_CONSTRAINTS_SUBSCRIPTABLE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/ConstraintTest.h>
#include <pe/util/Suffix.h>


namespace pe {

//=================================================================================================
//
//  MUST_BE_SUBSCRIBTABLE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the given type
 * the \a constraint function can be called or not. If the given type is not subscriptable
 * (the subscript operator can not be used on the type) a compilation error is created.
 */
template< typename T >
struct CONSTRAINT_MUST_BE_SUBSCRIBTABLE_FAILED
{
private:
   //**********************************************************************************************
   static T createT();
   //**********************************************************************************************

public:
   //**********************************************************************************************
   enum { T_is_not_subscriptable = sizeof( createT()[0] ),
          value = 1 };
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup constraints
 *
 * In case the given data type is not subscriptable, a compilation error is created.
 */
#define pe_CONSTRAINT_MUST_BE_SUBSCRIPTABLE(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_SUBSCRIBTABLE_FAILED< T >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_SUBSCRIPTABLE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_BE_SUBSCRIBTABLE_AS_DECAYABLE_POINTER CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the given type
 * the \a constraint function can be called or not. If the given type is not a subscriptable
 * pointer a compilation error is created.
 */
template< typename T >
struct CONSTRAINT_MUST_BE_SUBSCRIBTABLE_AS_DECAYABLE_POINTER_FAILED
{
private:
   //**********************************************************************************************
   static T createT();
   //**********************************************************************************************

public:
   //**********************************************************************************************
   enum { T_is_not_subscriptable = sizeof( 0[createT()] ),
          value = 1 };
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup constraints
 *
 * In case the given data type is not a subscriptable pointer, a compilation error is created.
 */
#define pe_CONSTRAINT_MUST_BE_SUBSCRIPTABLE_AS_DECAYABLE_POINTER(T) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_SUBSCRIBTABLE_AS_DECAYABLE_POINTER_FAILED< T >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_SUBSCRIBTABLE_AS_DECAYABLE_POINTER_TYPEDEF, __LINE__ )
//*************************************************************************************************

} // namespace pe

#endif
