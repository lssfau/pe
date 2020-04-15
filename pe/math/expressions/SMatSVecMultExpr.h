//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatSVecMultExpr.h
 *  \brief Header file for the sparse matrix/sparse vector multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATSVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATSVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/Reset.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a sparse matrix and a sparse vector
 *        (\f$ \vec{a}=B*\vec{c} \f$).
 * \ingroup sparse_matrix
 *
 * \param mat The left-hand side sparse matrix for the multiplication.
 * \param vec The right-hand side sparse vector for the multiplication.
 * \return The resulting vector.
 * \exception std::invalid_argument Matrix and vector sizes do not match.
 *
 * This operator represents the multiplication between a sparse matrix and a sparse vector:

   \code
   pe::SMatN A;
   pe::SVecN x, y;
   // ... Resizing and initialization
   y = A * x;
   \endcode

 * The operator returns a sparse vector of the higher-order element type of the two involved
 * element types \a T1::ElementType and \a T2::ElementType. Both the sparse matrix type \a T1
 * and the sparse vector type \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current size of the vector \a vec doesn't match the current number of columns
 * of the matrix \a mat, a \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side sparse vector
inline const typename MathTrait<typename T1::ResultType,typename T2::ResultType>::MultType
   operator*( const SparseMatrix<T1>& mat, const SparseVector<T2,false>& vec )
{
   using boost::remove_reference;

   typedef typename T1::CompositeType           MT;              // Composite type of the left-hand side sparse matrix expression
   typedef typename T2::CompositeType           VT;              // Composite type of the right-hand side sparse vector expression
   typedef typename T1::ResultType              R1;              // Result type of the left-hand side sparse matrix expression
   typedef typename T2::ResultType              R2;              // Result type of the right-hand side sparse vector expression
   typedef typename MathTrait<R1,R2>::MultType  MultType;        // Multiplication result type
   typedef typename MultType::ElementType       ET;              // Result type element type
   typedef typename remove_reference<MT>::type  X1;              // Auxiliary type for the left-hand side composite type
   typedef typename remove_reference<VT>::type  X2;              // Auxiliary type for the right-hand side composite type
   typedef typename X1::ConstIterator           MatrixIterator;  // Iterator type of the left-hand side sparse matrix expression
   typedef typename X2::ConstIterator           VectorIterator;  // Iterator type of the right-hand side sparse vector expression

   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( T1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( T2 );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( MultType );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T2, false );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( MultType, false );

   if( (~mat).columns() != (~vec).size() )
      throw std::invalid_argument( "Matrix and vector sizes do not match" );

   MT matrix( ~mat );
   VT vector( ~vec );

   MultType tmp( matrix.rows() );
   ET accu;
   const VectorIterator vend( vector.end()  );

   if( vector.nonZeros() == size_t(0) ) return tmp;

   for( size_t i=0; i<tmp.size(); ++i )
   {
      const MatrixIterator mend ( matrix.end(i)   );
      MatrixIterator       melem( matrix.begin(i) );

      if( melem == mend ) continue;

      VectorIterator velem( vector.begin() );

      reset( accu );

      while( true ) {
         if( melem->index() < velem->index() ) {
            ++melem;
            if( melem == mend ) break;
         }
         else if( velem->index() < melem->index() ) {
            ++velem;
            if( velem == vend ) break;
         }
         else {
            accu = melem->value() * velem->value();
            ++melem;
            ++velem;
            break;
         }
      }

      if( melem != mend && velem != vend )
      {
         while( true ) {
            if( melem->index() < velem->index() ) {
               ++melem;
               if( melem == mend ) break;
            }
            else if( velem->index() < melem->index() ) {
               ++velem;
               if( velem == vend ) break;
            }
            else {
               accu += melem->value() * velem->value();
               ++melem;
               if( melem == mend ) break;
               ++velem;
               if( velem == vend ) break;
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
