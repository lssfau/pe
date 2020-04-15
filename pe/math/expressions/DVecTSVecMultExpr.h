//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecTSVecMultExpr.h
 *  \brief Header file for the dense vector/sparse vector outer product expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECTSVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECTSVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the outer product of a dense vector and a sparse vector
 *        (\f$ A=\vec{b}*\vec{c}^T \f$).
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side dense vector for the outer product.
 * \param rhs The right-hand side transpose sparse vector for the outer product.
 * \return The resulting sparse matrix.
 *
 * This operator represents the outer product between a dense vector and a transpose sparse
 * vector:

   \code
   pe::VecN a;
   pe::SVecN b;
   pe::MatN A;
   // ... Resizing and initialization
   A = a * trans(b);
   \endcode

 * The operator returns a sparse matrix of the higher-order element type of the two involved
 * element types \a T1::ElementType and \a T2::ElementType. Both the dense vector type \a T1
 * and the sparse vector type \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.
 */
template< typename T1    // Type of the left-hand side dense vector
        , typename T2 >  // Type of the right-hand side sparse vector
inline const typename MathTrait<typename T1::ResultType,typename T2::ResultType>::MultType
   operator*( const DenseVector<T1,false>& lhs, const SparseVector<T2,true>& rhs )
{
   using boost::remove_reference;

   typedef typename T1::CompositeType            Lhs;            // Composite type of the left-hand side dense vector expression
   typedef typename T2::CompositeType            Rhs;            // Composite type of the right-hand side sparse vector expression
   typedef typename T1::ResultType               R1;             // Result type of the left-hand side dense vector expression
   typedef typename T2::ResultType               R2;             // Result type of the right-hand side sparse vector expression
   typedef typename MathTrait<R1,R2>::MultType   MultType;       // Multiplication result type
   typedef typename remove_reference<Rhs>::type  X2;             // Auxiliary type for the right-hand side composite type
   typedef typename X2::ConstIterator            ConstIterator;  // Iterator type of the right-hand side sparse vector expression

   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE ( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( T2 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( MultType );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T1, false );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T2, true  );

   Lhs left ( ~lhs );
   Rhs right( ~rhs );

   const size_t m( left.size()  );
   const size_t n( right.size() );
   const ConstIterator end( right.end() );

   // Estimation of the number of nonzeros per row
   std::vector<size_t> capacities( m );
   for( size_t i=0; i<m; ++i )
      if( !isDefault( left[i] ) )
         capacities[i] = right.nonZeros();

   // Creating the resulting sparse matrix
   MultType A( m, n, capacities );

   // Performing the vector-vector multiplication
   for( size_t i=0; i<m; ++i ) {
      if( !isDefault( left[i] ) ) {
         for( ConstIterator element=right.begin(); element!=end; ++element ) {
            A.append( i, element->index(), left[i] * element->value() );
         }
      }
   }

   return A;
}
//*************************************************************************************************

} // namespace pe

#endif
