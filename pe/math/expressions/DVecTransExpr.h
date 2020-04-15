//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecTransExpr.h
 *  \brief Header file for the dense vector transpose expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECTRANSEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECTRANSEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/constraints/DenseVector.h>
#include <pe/math/expressions/DenseVector.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the transpose of the given dense vector.
 * \ingroup dense_vector
 *
 * \param dv The dense vector to be transposed.
 * \return The transpose of the dense vector.
 *
 * This function returns an expression representing the transpose of the given dense vector:

   \code
   pe::VecN  a;
   pe::VecNT b;
   // ... Resizing and initialization
   b = trans( a );
   \endcode
 */
template< typename VT  // Type of the dense vector
        , bool TF >    // Transposition flag
inline const typename VT::TransposeType& trans( const DenseVector<VT,TF>& dv )
{
   typedef typename VT::TransposeType  TransposeType;
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( TransposeType );
   return reinterpret_cast< const TransposeType& >( ~dv );
}
//*************************************************************************************************

} // namespace pe

#endif
