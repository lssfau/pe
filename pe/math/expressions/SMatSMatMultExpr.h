//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatSMatMultExpr.h
 *  \brief Header file for the sparse matrix/sparse matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATSMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATSMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <set>
#include <stdexcept>
#include <vector>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/math/Functions.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/VectorN.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of two sparse matrices (\f$ C=A*B \f$).
 * \ingroup sparse_matrix
 *
 * \param lhs The left-hand side sparse matrix for the multiplication.
 * \param rhs The right-hand side sparse matrix for the multiplication.
 * \return The resulting matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This operator represents the multiplication of two sparse matrices:

   \code
   pe::SMatN A, B, C;
   // ... Resizing and initialization
   C = A * B;
   \endcode

 * The operator returns a sparse matrix of the higher-order element type of the two involved
 * matrix element types \a T1::ElementType and \a T2::ElementType. Both matrix types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 *
 * The algorithm is optimized based on the assumption that the two given sparse matrices are
 * only sparsely filled. At first a conservative estimation of the expected number of nonzeros
 * in the result matrix is performed. The number of nonzeros in row \f$ i \f$ is bounded by
 * \f$ p = \min(p_A \cdot p_B, n) \f$, where \f$ p_{A/B} \f$ is the maximum number of nonzeros
 * per row in the matrix \f$ \mat{A} \f$ respectively \f$ \mat{B}\f$ and \f$ n \f$ is the number
 * of columns of \f$ \mat{B} \f$ and thus the number of columns in the result matrix. Then the
 * necessary data structures are allocated and the result is computed row by row. First the
 * sparsity structure of the current row is determined, which takes \f$ \bigO(p \log p)\f$
 * operations, then for each nonzero entry the scalar product between the corresponding sparse
 * row in \f$ \mat{A} \f$ and the corresponding sparse column in \f$ \mat{B} \f$ is calculated,
 * which takes \f$ \bigO(p_A + q_B) \f$ operations. This process is repeated for each of the
 * \f$ m \f$ rows, which results in the following total complexity:
 *
 *          \f[ \bigO(m p ( \log p + p_A + q_B )) = \bigO(m p \max(p_A, q_B)) \f]
 *
 * For dense matrices this reduces to the usual cubic complexity. But for Jacobians arising
 * from multibody simulations, \f$ p_A \f$ and \f$ q_B \f$ are constant and the complexity
 * reduces to \f$ \bigO(m p_B \log p_B) \f$, which is, assuming the number of constraints per
 * body is bounded, linear in the number of constraints.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side sparse matrix
const typename MathTrait<typename T1::ResultType,typename T2::ResultType>::MultType
   operator*( const SparseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   using boost::remove_reference;

   typedef typename T1::CompositeType            Lhs;               // Composite type of the left-hand side sparse matrix expression
   typedef typename T2::CompositeType            Rhs;               // Composite type of the right-hand side sparse matrix expression
   typedef typename T1::ResultType               R1;                // Result type of the left-hand side sparse matrix expression
   typedef typename T2::ResultType               R2;                // Result type of the left-hand side sparse matrix expression
   typedef typename MathTrait<R1,R2>::MultType   MultType;          // Multiplication result type
   typedef typename MultType::ElementType        ET;                // Result type element type
   typedef typename remove_reference<Lhs>::type  X1;                // Auxiliary type for the left-hand side composite type
   typedef typename remove_reference<Rhs>::type  X2;                // Auxiliary type for the right-hand side composite type
   typedef typename X1::ConstIterator            LhsConstIterator;  // Iterator type of the left-hand side sparse matrix expression
   typedef typename X2::ConstIterator            RhsConstIterator;  // Iterator type of the right-hand side sparse vector expression

   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T2 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( MultType );

   if( (~lhs).columns() != (~rhs).rows() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   Lhs left ( ~lhs );
   Rhs right( ~rhs );

   const size_t m( left.rows() );
   const size_t n( right.columns() );

   // Conservative estimation of the number of nonzeros per row
   std::vector<size_t> capacities( m );
   for( size_t i=0; i<m; ++i ) {
      size_t accu( 0 );
      for( LhsConstIterator j=left.begin(i); j!=left.end(i); ++j )
         accu += right.nonZeros( j->index() );
      capacities[i] = min( accu, n );
   }

   MultType ret( m, n, capacities );
   VectorN<ET> tmp( n );

   for( size_t i=0; i<m; ++i )
   {
      const LhsConstIterator jend( (~lhs).end(i) );

      // Performing the multiplication and determining the sparsity structure of the i-th row
      std::set<size_t> spstruct;
      for( LhsConstIterator j=left.begin(i); j!=jend; ++j )
      {
         const size_t idx( j->index() );
         const RhsConstIterator kend( right.end(idx) );

         for( RhsConstIterator k=right.begin(idx); k!=kend; ++k ) {
            if( spstruct.insert( k->index() ).second ) {
               tmp[ k->index() ] = j->value() * k->value();
            }
            else {
               tmp[ k->index() ] += j->value() * k->value();
            }
         }
      }

      // Merging the data from the temporary vector into the result matrix
      const std::set<size_t>::const_iterator end( spstruct.end() );
      for( std::set<size_t>::const_iterator it=spstruct.begin(); it!=end; ++it ) {
         const size_t j( *it );

         if( !isDefault( tmp[j] ) )
            ret.append( i, j, tmp[j] );
      }
   }

   return ret;
}
//*************************************************************************************************

} // namespace pe

#endif
