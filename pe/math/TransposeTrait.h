//=================================================================================================
/*!
 *  \file pe/math/TransposeTrait.h
 *  \brief Header file for the transpose trait
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

#ifndef _PE_MATH_TRANSPOSETRAIT_H_
#define _PE_MATH_TRANSPOSETRAIT_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base template for the TransposeTrait class.
 * \ingroup math
 *
 * The TransposeTrait class template provides information about the transpose data type of a
 * particular scalar, vector or matrix data type. Once instantiated, the according transpose
 * data type can be accessed by the nested type definition \a Type.\n
 * The following examples give an impression of the use of the TransposeTrait class template:

   \code
   TransposeTrait< int >::Type                                    // results in int
   TransposeTrait< double >::Type                                 // results in double
   TransposeTrait< Vector2<int,false> >::Type                     // results in Vector2<int,true>
   TransposeTrait< Vector3<double,true> >::Type                   // results in Vector3<double,false>
   TransposeTrait< VectorN< Vector3<double,false>, false >::Type  // results in VectorN< Vector3<double,true>, true >
   TransposeTrait< VectorN< VectorN<double,true>, false >::Type   // results in VectorN< VectorN<double,false>, true >
   \endcode
 */
template< typename T >
struct TransposeTrait
{
   typedef T  Type;  //!< Default return type of the transpose trait.
};
//*************************************************************************************************

} // namespace pe

#endif
