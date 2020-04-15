//=================================================================================================
/*!
 *  \file pe/core/Algorithm.h
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

#ifndef _PE_CORE_ALGORITHM_H_
#define _PE_CORE_ALGORITHM_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <functional>
#include <pe/util/constraints/Pointer.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  ALGORITHM HELPER CLASSES
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Helper class for the findUserID() function
 * \ingroup algorithm
 *
 * The CompareUserID class is a binary functor required for the findUserID() function.
 */
template< typename BodyType >
struct CompareUserID : public std::binary_function<BodyType,size_t,bool>
{
   //**Binary function call operator***************************************************************
   /*!\name Binary function call operator */
   //@{
   bool operator()( BodyType body, size_t uid ) const {
      return body->getID() == uid;
   }
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Helper class for the findSystemID() function
 * \ingroup algorithm
 *
 * The CompareSystemID class is a binary functor required for the findSystemID() function.
 */
template< typename BodyType >
struct CompareSystemID : public std::binary_function<BodyType,ularge_t,bool>
{
   //**Binary function call operator***************************************************************
   /*!\name Binary function call operator */
   //@{
   bool operator()( BodyType body, ularge_t sid ) const {
      return body->getSystemID() == sid;
   }
   //@}
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  GENERIC FIND FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Finding the first rigid body with a certain user-specific ID.
 * \ingroup algorithm
 *
 * \param begin The beginning of the element range.
 * \param end The end of the element range.
 * \param uid The user-specific ID for the search.
 *
 * This function finds the first rigid body with the user-specific ID \a uid in the range
 * [\a begin, \a end). The following example illustrates the use of the findUserID() function:

   \code
   pe::WorldID world = pe::theWorld();

   // Setup of two boxes with user-specific IDs of 1 and 2
   pe::createBox( 1, -1.0,-1.0,-1.0, 0.5, 0.5, 0.5, iron );
   pe::createBox( 2, -2.0,-2.0,-2.0, 0.5, 0.5, 0.5, iron );

   // Setup of two spheres with user-specific IDs of 1 and 2
   pe::createSphere( 1, 1.0, 1.0, 1.0, 0.1, iron );
   pe::createSphere( 2, 2.0, 2.0, 2.0, 0.1, iron );

   // This call of the findUserID() function returns an iterator to the first rigid body with
   // user-specific ID 2. In this example, the iterator will refer to the second box, since it
   // was created before the sphere.
   pe::World::Iterator b2 = pe::findUserID( world->begin(), world->end(), 2 );

   // This function call returns an iterator to the first sphere with user-specific ID 2. In this
   // example, the iterator will refer to the second sphere.
   pe::World::Bodies::CastIterator<Sphere> s2 = pe::findUserID( world->begin<Sphere>(), world->end<Sphere>(), 2 );
   \endcode
 */
template< typename Iterator >  // Type of the iterators
inline Iterator findUserID( Iterator begin, Iterator end, size_t uid )
{
   typedef typename Iterator::value_type  value_type;
   pe_CONSTRAINT_MUST_BE_POINTER_TYPE( value_type );
   return std::find_if( begin, end, std::bind2nd( CompareUserID<value_type>(), uid ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  DOXYGEN DOCUMENTATION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup algorithm Algorithms */
//*************************************************************************************************

} // namespace pe

#endif
