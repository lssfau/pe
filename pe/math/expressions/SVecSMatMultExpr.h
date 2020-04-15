//=================================================================================================
/*!
 *  \file pe/math/expressions/SVecSMatMultExpr.h
 *  \brief Header file for the sparse vector/sparse matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SVECSMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SVECSMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/Reset.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a sparse vector and a sparse matrix
 *        (\f$ \vec{a}^T=\vec{b}^T*C \f$).
 * \ingroup sparse_matrix
 *
 * \param vec The left-hand side transpose sparse vector for the multiplication.
 * \param mat The right-hand side sparse matrix for the multiplication.
 * \return The resulting transpose vector.
 * \exception std::invalid_argument Vector and matrix sizes do not match.
 *
 * This operator represents the multiplication between a transpose sparse vector and a sparse
 * matrix:

   \code
   pe::SVecNT x, y;
   pe::SMatN A;
   // ... Resizing and initialization
   y = x * A;
   \endcode

 * The operator returns a transpose sparse vector of the higher-order element type of the two
 * involved element types \a T1::ElementType and \a T2::ElementType. Both the dense matrix type
 * \a T1 and the dense vector type \a T2 as well as the two element types \a T1::ElementType
 * and \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current size of the vector \a vec doesn't match the current number of rows of
 * the matrix \a mat, a \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side sparse vector
        , typename T2 >  // Type of the right-hand side sparse matrix
inline const typename MathTrait<typename T1::ResultType,typename T2::ResultType>::MultType
   operator*( const SparseVector<T1,true>& vec, const SparseMatrix<T2>& mat )
{
   using boost::remove_reference;

   typedef typename T1::CompositeType           VT;              // Composite type of the left-hand side sparse vector expression
   typedef typename T1::ResultType              R1;              // Result type of the left-hand side sparse vector expression
   typedef typename T2::ResultType              R2;              // Result type of the right-hand side sparse matrix expression
   typedef typename MathTrait<R1,R2>::MultType  MultType;        // Multiplication result type
   typedef typename MultType::ElementType       ET;              // Result type element type
   typedef typename remove_reference<VT>::type  X1;              // Auxiliary type for the left-hand side composite type
   typedef typename X1::ConstIterator           VectorIterator;  // Iterator type of the left-hand side sparse vector expression
   typedef typename R2::ConstIterator           MatrixIterator;  // Iterator type of the right-hand side sparse matrix expression

   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T2 );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( MultType );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T1, true );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( MultType, true );

   if( (~vec).size() != (~mat).rows() )
      throw std::invalid_argument( "Vector and matrix sizes do not match" );

   VT vector( ~vec );

   // Transposing the right-hand side sparse matrix
   R2 matT( trans( ~mat ) );
   pe_INTERNAL_ASSERT( (~mat).rows() == matT.columns(), "Invalid matrix transpose" );
   pe_INTERNAL_ASSERT( (~mat).columns() == matT.rows(), "Invalid matrix transpose" );

   // Performing the vector-matrix multiplication
   MultType tmp( (~mat).columns() );
   ET accu;
   const VectorIterator vend( vector.end() );

   if( vector.nonZeros() == size_t(0) ) return tmp;

   for( size_t i=0; i<tmp.size(); ++i )
   {
      const MatrixIterator mend ( matT.end(i)   );
      MatrixIterator       melem( matT.begin(i) );

      if( melem == mend ) continue;

      VectorIterator velem( vector.begin() );

      reset( accu );

      while( true ) {
         if( velem->index() < melem->index() ) {
            ++velem;
            if( velem == vend ) break;
         }
         else if( melem->index() < velem->index() ) {
            ++melem;
            if( melem == mend ) break;
         }
         else {
            accu = velem->value() * melem->value();
            ++velem;
            ++melem;
            break;
         }
      }

      if( velem != vend && melem != mend )
      {
         while( true ) {
            if( velem->index() < melem->index() ) {
               ++velem;
               if( velem == vend ) break;
            }
            else if( melem->index() < velem->index() ) {
               ++melem;
               if( melem == mend ) break;
            }
            else {
               accu += velem->value() * melem->value();
               ++velem;
               if( velem == vend ) break;
               ++melem;
               if( melem == mend ) break;
            }
         }
      }

      if( !isDefault( accu ) )
         tmp.insert( i, accu );
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
