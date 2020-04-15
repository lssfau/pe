//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatSMatAddExpr.h
 *  \brief Header file for the sparse matrix/sparse matrix addition expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATSMATADDEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATSMATADDEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <vector>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Addition operator for the addition of two sparse matrices (\f$ A=B+C \f$).
 * \ingroup sparse_matrix
 *
 * \param lhs The left-hand side matrix for the matrix addition.
 * \param rhs The right-hand side matrix to be added to the left-hand side matrix.
 * \return The sum of the two sparse matrices.
 * \exception std::invalid_argument Matrix sizes do not match
 *
 * This operator represents the addition of two sparse matrices:

   \code
   pe::SMatN A, B, C;
   // ... Resizing and initialization
   C = A + B;
   \endcode

 * The operator returns a sparse matrix of the higher-order element type of the two involved
 * vector element types \a T1::ElementType and \a T2::ElementType. Both matrix types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current number of rows and columns of the two given  matrices don't match, a
 * \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side sparse matrix
const typename MathTrait<typename T1::ResultType,typename T2::ResultType>::AddType
   operator+( const SparseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   using boost::remove_reference;

   typedef typename T1::CompositeType            Lhs;               // Composite type of the left-hand side sparse matrix expression
   typedef typename T2::CompositeType            Rhs;               // Composite type of the right-hand side sparse matrix expression
   typedef typename T1::ResultType               R1;                // Result type of the left-hand side sparse matrix expression
   typedef typename T2::ResultType               R2;                // Result type of the left-hand side sparse matrix expression
   typedef typename MathTrait<R1,R2>::AddType    AddType;           // Addition result type
   typedef typename remove_reference<Lhs>::type  X1;                // Auxiliary type for the left-hand side composite type
   typedef typename remove_reference<Rhs>::type  X2;                // Auxiliary type for the right-hand side composite type
   typedef typename X1::ConstIterator            LhsConstIterator;  // Iterator type of the left-hand side sparse matrix expression
   typedef typename X2::ConstIterator            RhsConstIterator;  // Iterator type of the right-hand side sparse matrix expression

   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T2 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( AddType );

   if( (~lhs).rows() != (~rhs).rows() || (~lhs).columns() != (~rhs).columns() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   Lhs left ( ~lhs );
   Rhs right( ~rhs );

   // Analyzing the sparse matrices to predict the required row storage capacities
   std::vector<size_t> capacities( left.rows(), 0 );

   for( size_t i=0; i<left.rows(); ++i )
   {
      const LhsConstIterator lend( left.end(i)  );
      const RhsConstIterator rend( right.end(i) );

      LhsConstIterator l( left.begin(i)  );
      RhsConstIterator r( right.begin(i) );
      size_t accu( left.nonZeros(i) + right.nonZeros(i) );

      for( ; l!=lend && r!=rend; ++l ) {
         while( r->index() < l->index() && ++r != rend ) {}
         if( r!=rend && l->index() == r->index() ) {
            --accu;
            ++r;
         }
      }

      pe_INTERNAL_ASSERT( accu <= left.columns(), "Invalid number of non-zero elements predicted" );

      capacities[i] = accu;
   }

   // Merging the two matrices
   AddType tmp( left.rows(), left.columns(), capacities );

   for( size_t i=0; i<left.rows(); ++i )
   {
      const LhsConstIterator lend( left.end(i)  );
      const RhsConstIterator rend( right.end(i) );

      LhsConstIterator l( left.begin(i) );
      RhsConstIterator r( right.begin(i) );

      while( l != lend && r != rend )
      {
         if( l->index() < r->index() ) {
            tmp.append( i, l->index(), l->value() );
            ++l;
         }
         else if( l->index() > r->index() ) {
            tmp.append( i, r->index(), r->value() );
            ++r;
         }
         else {
            tmp.append( i, l->index(), l->value()+r->value() );
            ++l;
            ++r;
         }
      }

      while( l != lend ) {
         tmp.append( i, l->index(), l->value() );
         ++l;
      }

      while( r != rend ) {
         tmp.append( i, r->index(), r->value() );
         ++r;
      }
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
