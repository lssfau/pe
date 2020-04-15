//=================================================================================================
/*!
 *  \file pe/util/DimensionOf.h
 *  \brief Compile time evaluation of array sizes
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

#ifndef _PE_UTIL_DIMENSIONOF_H_
#define _PE_UTIL_DIMENSIONOF_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/Byte.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  DIMENSIONOF FUNCTIONALITY
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Helper class for the dimensionof function.
 * \ingroup util
 *
 * The Array class is a helper class for the dimensionof function. It provides a public array
 * member of exactly N bytes.
 */
template< unsigned int N >
struct Array {
   byte array[N];
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Static evaluation of array dimensions.
 * \ingroup util
 *
 * \param a Reference to a static array of type T and size N.
 * \return Dimension of the static array.
 *
 * The dimensionof function is a safe way to evaluate the size of an array. The function only
 * works for array arguments and fails for pointers and user-defined class types.

   \code
   int              ai[ 42 ];
   int*             pi( ai );
   std::vector<int> vi( 42 );

   dimensionof( ai );  // Returns the size of the integer array (42)
   dimensionof( pi );  // Fails to compile!
   dimensionof( vi );  // Fails to compile!
   \endcode
 */
template< typename T, unsigned int N >
inline size_t dimensionof( T(&a)[N] )
{
   return sizeof( Array<N> );
}
//*************************************************************************************************

} // namespace pe

#endif
