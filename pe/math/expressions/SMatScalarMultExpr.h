//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatScalarMultExpr.h
 *  \brief Header file for the sparse matrix/scalar multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATSCALARMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATSCALARMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a sparse matrix and a scalar value
 *        (\f$ A=B*s \f$).
 * \ingroup sparse_matrix
 *
 * \param mat The left-hand side sparse matrix for the multiplication.
 * \param scalar The right-hand side scalar value for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a sparse matrix and a scalar value:

   \code
   pe::SMatN A, B;
   // ... Resizing and initialization
   B = A * 1.25;
   \endcode

 * The operator returns a sparse matrix of the higher-order element type of the involved data
 * types \a T1::ElementType and \a T2. Note that this operator only works for scalar values
 * of built-in data type.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side scalar
inline const typename MathTrait<typename T1::ResultType,T2,IsNumeric<T2>::value>::MultType
   operator*( const SparseMatrix<T1>& mat, T2 scalar )
{
   typedef typename T1::ResultType              R1;        // Result type of the left-hand side sparse matrix expression
   typedef typename MathTrait<R1,T2>::MultType  MultType;  // Multiplication result type
   typedef typename MultType::Iterator          Iterator;  // Iterator type of the result type

   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( MultType );
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE      ( T2 );

   MultType tmp( mat );

   for( size_t i=0; i<tmp.rows(); ++i ) {
      const Iterator end( tmp.end(i) );
      for( Iterator element=tmp.begin(i); element<end; ++element )
         element->value() *= scalar;
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a scalar value and a sparse matrix
 *        (\f$ A=s*B \f$).
 * \ingroup sparse_matrix
 *
 * \param scalar The left-hand side scalar value for the multiplication.
 * \param mat The right-hand side sparse matrix for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a scalar value and a sparse matrix:

   \code
   pe::SMatN A, B;
   // ... Resizing and initialization
   B = 1.25 * A;
   \endcode

 * The operator returns a sparse matrix of the higher-order element type of the involved data
 * types \a T1 and \a T2::ElementType. Note that this operator only works for scalar values
 * of built-in data type.
 */
template< typename T1    // Type of the left-hand side scalar
        , typename T2 >  // Type of the right-hand side sparse matrix
inline const typename MathTrait<T1,typename T2::ResultType,IsNumeric<T1>::value>::MultType
   operator*( T1 scalar, const SparseMatrix<T2>& mat )
{
   typedef typename T2::ResultType              R2;        // Result type of the left-hand side sparse matrix expression
   typedef typename MathTrait<T1,R2>::MultType  MultType;  // Multiplication result type
   typedef typename MultType::Iterator          Iterator;  // Iterator type of the result type

   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE      ( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T2 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( MultType );

   MultType tmp( mat );

   for( size_t i=0; i<tmp.rows(); ++i ) {
      const Iterator end( tmp.end(i) );
      for( Iterator elem=tmp.begin(i); elem<end; ++elem )
         elem->value() *= scalar;
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
