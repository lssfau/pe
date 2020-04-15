//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecSMatMultExpr.h
 *  \brief Header file for the dense vector/sparse matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECSMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECSMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Reference.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DVECSMATMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense vector-sparse matrix multiplications.
 * \ingroup dense_vector_expression
 *
 * The DVecSMatMultExpr class represents the compile time expression for multiplications
 * between dense vectors and sparse matrices.
 */
template< typename VT    // Type of the left-hand side dense vector
        , typename MT >  // Type of the right-hand side sparse matrix
class DVecSMatMultExpr : public DenseVector< DVecSMatMultExpr<VT,MT>, true >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT::ResultType     VRT;  //!< Result type of the left-hand side dense vector expression.
   typedef typename MT::ResultType     MRT;  //!< Result type of the right-hand side sparse matrix expression.
   typedef typename VT::CompositeType  VCT;  //!< Composite type of the left-hand side dense vector expression.
   typedef typename MT::CompositeType  MCT;  //!< Composite type of the right-hand side sparse matrix expression.
   typedef typename MT::ConstIterator  ConstIterator;  //!< Iterator over the matrix elements.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DVecSMatMultExpr<VT,MT>                This;           //!< Type of this DVecSMatMultExpr instance.
   typedef typename MathTrait<VRT,MRT>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::TransposeType     TransposeType;  //!< Transpose type for expression template evaluations.
   typedef const ResultType                       CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense vector expression.
   typedef typename SelectType<IsExpression<VT>::value,const VRT,VCT>::Type  Lhs;

   //! Composite type of the right-hand side sparse matrix expression.
   typedef const MRT  Rhs;
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DVecSMatMultExpr class.
   */
   explicit inline DVecSMatMultExpr( const VT& vec, const MT& mat )
      : vec_ ( vec )         // Left-hand side dense vector of the multiplication expression
      , matT_( trans(mat) )  // Right-hand side sparse matrix of the multiplication expression
   {
      pe_INTERNAL_ASSERT( vec_.size() == mat.rows(), "Invalid vector and matrix sizes" );
      pe_INTERNAL_ASSERT( mat.rows() == matT_.columns(), "Invalid matrix transpose" );
      pe_INTERNAL_ASSERT( mat.columns() == matT_.rows(), "Invalid matrix transpose" );
   }
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline const ElementType operator[]( size_t index ) const {
      pe_INTERNAL_ASSERT( index < matT_.columns(), "Invalid vector access index" );

      ElementType res;
      ConstIterator element( matT_.begin(index) );
      const ConstIterator end( matT_.end(index) );

      if( element != end ) {
         res = vec_[element->index()] * element->value();
         ++element;
         for( ; element!=end; ++element ) {
            res += vec_[element->index()] * element->value();
         }
      }
      else {
         reset( res );
      }

      return res;
   }
   //**********************************************************************************************

   //**Size function*******************************************************************************
   /*!\brief Returns the current size/dimension of the vector.
   //
   // \return The size of the vector.
   */
   inline size_t size() const {
      return matT_.rows();
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the expression is aliased with the given address \a alias.
   //
   // \param alias The alias to be checked.
   // \return \a true in case the given alias is contained in this expression, \a false if not.
   */
   template< typename T >
   inline bool isAliased( const T* alias ) const {
      return vec_.isAliased( alias );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs vec_;   //!< Left-hand side dense vector of the multiplication expression.
   Rhs matT_;  //!< Right-hand side sparse matrix of the multiplication expression.
   //**********************************************************************************************

   //**Assignment to dense vectors*****************************************************************
   /*!\brief Assignment of a dense vector-sparse matrix multiplication to a dense vector.
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-sparse
   // matrix multiplication expression to a dense vector.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline void assign( DenseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename MRT::ConstIterator  ConstIterator;

      for( size_t i=0; i<(~lhs).size(); ++i ) {
         ConstIterator element( rhs.matT_.begin(i) );
         if( element != rhs.matT_.end(i) ) {
            (~lhs)[i] = rhs.vec_[ element->index() ] * element->value();
            ++element;
            for( ; element!=rhs.matT_.end(i); ++element ) {
               (~lhs)[i] += rhs.vec_[ element->index() ] * element->value();
            }
         }
         else {
            reset( (~lhs)[i] );
         }
      }
   }
   //**********************************************************************************************

   //**Assignment to sparse vectors****************************************************************
   /*!\brief Assignment of a dense vector-sparse matrix multiplication to a sparse vector.
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-sparse
   // matrix multiplication expression to a sparse vector.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline void assign( SparseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      assign( ~lhs, tmp );
   }
   //**********************************************************************************************

   //**Addition assignment to dense vectors********************************************************
   /*!\brief Addition assignment of a dense vector-sparse matrix multiplication to a dense vector.
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // sparse matrix multiplication expression to a dense vector.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline void addAssign( DenseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename MRT::ConstIterator  ConstIterator;

      for( size_t i=0; i<(~lhs).size(); ++i ) {
         const ConstIterator end( rhs.matT_.end(i) );
         for( ConstIterator element=rhs.matT_.begin(i); element!=end; ++element ) {
            (~lhs)[i] += rhs.vec_[ element->index() ] * element->value();
         }
      }
   }
   //**********************************************************************************************

   //**Addition assignment to sparse vectors*******************************************************
   /*!\brief Addition assignment of a dense vector-sparse matrix multiplication to a sparse vector.
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // sparse matrix multiplication expression to a sparse vector.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline void addAssign( SparseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      addAssign( ~lhs, tmp );
   }
   //**********************************************************************************************

   //**Subtraction assignment to dense vectors*****************************************************
   /*!\brief Subtraction assignment of a dense vector-sparse matrix multiplication to a dense vector.
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // sparse matrix multiplication expression to a dense vector.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline void subAssign( DenseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename MRT::ConstIterator  ConstIterator;

      for( size_t i=0; i<(~lhs).size(); ++i ) {
         const ConstIterator end( rhs.matT_.end(i) );
         for( ConstIterator element=rhs.matT_.begin(i); element!=end; ++element ) {
            (~lhs)[i] -= rhs.vec_[ element->index() ] * element->value();
         }
      }
   }
   //**********************************************************************************************

   //**Subtraction assignment to sparse vectors****************************************************
   /*!\brief Subtraction assignment of a dense vector-sparse matrix multiplication to a sparse vector.
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // sparse matrix multiplication expression to a sparse vector.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline void subAssign( SparseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      subAssign( ~lhs, tmp );
   }
   //**********************************************************************************************

   //**Multiplication assignment to dense vectors**************************************************
   /*!\brief Multiplication assignment of a dense vector-sparse matrix multiplication to a dense vector.
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-sparse matrix multiplication expression to a dense vector.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline void multAssign( DenseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      multAssign( ~lhs, tmp );
   }
   //**********************************************************************************************

   //**Multiplication assignment to sparse vectors*************************************************
   /*!\brief Multiplication assignment of a dense vector-sparse matrix multiplication to a sparse vector.
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-sparse matrix multiplication expression to a sparse vector.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline void multAssign( SparseVector<VT2,true>& lhs, const DVecSMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      multAssign( ~lhs, tmp );
   }
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT, true );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( MT );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a dense vector and a sparse matrix
 *        (\f$ \vec{a}^T=\vec{b}^T*C \f$).
 * \ingroup sparse_matrix
 *
 * \param vec The left-hand side transpose dense vector for the multiplication.
 * \param mat The right-hand side sparse matrix for the multiplication.
 * \return The resulting transpose vector.
 * \exception std::invalid_argument Vector and matrix sizes do not match.
 *
 * This operator represents the multiplication between a transpose dense vector and a sparse
 * matrix:

   \code
   pe::VecNT x, y;
   pe::SMatN A;
   // ... Resizing and initialization
   y = x * A;
   \endcode

 * The operator returns an expression representing a transpose dense vector of the higher-order
 * element type of the two involved element types \a T1::ElementType and \a T2::ElementType.
 * Both the dense matrix type \a T1 and the dense vector type \a T2 as well as the two element
 * types \a T1::ElementType and \a T2::ElementType have to be supported by the MathTrait class
 * template.\n
 * In case the current size of the vector \a vec doesn't match the current number of rows of
 * the matrix \a mat, a \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side dense vector
        , typename T2 >  // Type of the right-hand side sparse matrix
inline const DVecSMatMultExpr<T1,T2>
   operator*( const DenseVector<T1,true>& vec, const SparseMatrix<T2>& mat )
{
   if( (~vec).size() != (~mat).rows() )
      throw std::invalid_argument( "Vector and matrix sizes do not match" );

   return DVecSMatMultExpr<T1,T2>( ~vec, ~mat );
}
//*************************************************************************************************

} // namespace pe

#endif
