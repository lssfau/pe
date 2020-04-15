//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatScalarDivExpr.h
 *  \brief Header file for the sparse matrix/scalar division expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATSCALARDIVEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATSCALARDIVEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsFloatingPoint.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Division operator for the division of a sparse matrix by a scalar value (\f$ A=B/s \f$).
 * \ingroup sparse_matrix
 *
 * \param mat The left-hand side sparse matrix for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result matrix.
 *
 * This operator represents the division of a sparse matrix by a scalar value:

   \code
   pe::SMatN A, B;
   // ... Resizing and initialization
   B = A / 0.24;
   \endcode

 * The operator returns a sparse matrix of the higher-order element type of the involved data
 * types \a T1::ElementType and \a T2. Note that this operator only works for scalar values
 * of built-in data type.
 *
 * \b Note: A division by zero is only checked by a user assert.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side scalar
inline const typename MathTrait<typename T1::ResultType,T2,IsNumeric<T2>::value>::DivType
   operator/( const SparseMatrix<T1>& mat, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   typedef typename T1::ResultType             R1;        // Result type of the left-hand side sparse matrix expression
   typedef typename MathTrait<R1,T2>::DivType  DivType;   // Division result type
   typedef typename DivType::ElementType       ET;        // Element type of the result type
   typedef typename DivType::Iterator          Iterator;  // Iterator type of the result type

   // Data type of the inverse scalar element
   typedef typename SelectType<IsFloatingPoint<ET>::value,ET,T2>::Type  Inv;

   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( DivType );
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE      ( T2 );

   DivType tmp( mat );

   // Depending on the two involved element data types, an integer division is applied
   // or a floating point division is selected.
   if( IsNumeric<ET>::value && IsFloatingPoint<ET>::value ) {
      const Inv iscalar( Inv(1)/static_cast<Inv>( scalar ) );
      for( size_t i=0; i<tmp.rows(); ++i ) {
         for( Iterator element=tmp.begin(i); element!=tmp.end(i); ++element )
            element->value() *= iscalar;
      }
   }
   else {
      for( size_t i=0; i<tmp.rows(); ++i )
         for( Iterator element=tmp.begin(i); element!=tmp.end(i); ++element )
            element->value() /= scalar;
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
