//=================================================================================================
/*!
 *  \file pe/util/Algorithm.h
 *  \brief Headerfile for generic algorithms
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

#ifndef _PE_UTIL_ALGORITHM_H_
#define _PE_UTIL_ALGORITHM_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/DerivedFrom.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  POLYMORPHIC COUNT
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Counts the pointer to objects with dynamic type \a D.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of objects with dynamic type \a D.
 *
 * This function traverses the range \f$ [first,last) \f$ of pointers to objects with static
 * type \a S and counts all polymorphic pointers to objects of dynamic type \a D. Note that
 * in case \a D is not a type derived from \a S, a compile time error is created!
 */
template< typename D    // Dynamic type of the objects
        , typename S >  // Static type of the objects
inline size_t polymorphicCount( S *const * first, S *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( D, S );

   size_t count( 0 );
   for( S *const * it=first; it!=last; ++it )
      if( dynamic_cast<D*>( *it ) ) ++count;
   return count;
}
//*************************************************************************************************




//=================================================================================================
//
//  POLYMORPHIC FIND
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Finds the next pointer to an object with dynamic type \a D.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to an object with dynamic type \a D.
 *
 * This function traverses the range \f$ [first,last) \f$ of pointers to objects with static
 * type \a S until it finds the next polymorphic pointer to an object of dynamic type \a D.
 * Note that in case \a D is not a type derived from \a S, a compile time error is created!
 */
template< typename D    // Dynamic type of the objects
        , typename S >  // Static type of the objects
inline S *const * polymorphicFind( S *const * first, S *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( D, S );

   while( first != last && !dynamic_cast<D*>( *first ) ) ++first;
   return first;
}
//*************************************************************************************************

} // namespace pe

#endif
