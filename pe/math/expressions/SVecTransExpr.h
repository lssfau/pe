//=================================================================================================
/*!
 *  \file pe/math/expressions/SVecTransExpr.h
 *  \brief Header file for the sparse vector transpose expression
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

#ifndef _PE_MATH_EXPRESSIONS_SVECTRANSEXPR_H_
#define _PE_MATH_EXPRESSIONS_SVECTRANSEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/constraints/SparseVector.h>
#include <pe/math/expressions/SparseVector.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the transpose of the given sparse vector.
 * \ingroup sparse_vector
 *
 * \param sv The sparse vector to be transposed.
 * \return The transpose of the sparse vector.
 *
 * This function returns an expression representing the transpose of the given sparse vector:

   \code
   pe::SVecN  a;
   pe::SVecNT b;
   // ... Resizing and initialization
   b = trans( a );
   \endcode
 */
template< typename VT  // Type of the sparse vector
        , bool TF >    // Transposition flag
inline const typename VT::TransposeType& trans( const SparseVector<VT,TF>& sv )
{
   typedef typename VT::TransposeType  TransposeType;
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( TransposeType );
   return reinterpret_cast< const typename VT::TransposeType& >( sv );
}
//*************************************************************************************************

} // namespace pe

#endif
