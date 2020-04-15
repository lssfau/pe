//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecDMatMultExpr.h
 *  \brief Header file for the dense vector/dense matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECDMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECDMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Reference.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DVECDMATMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense vector-dense matrix multiplications.
 * \ingroup dense_vector_expression
 *
 * The DVecDMatMultExpr class represents the compile time expression for multiplications
 * between dense vectors and dense matrices.
 */
template< typename VT    // Type of the left-hand side dense vector
        , typename MT >  // Type of the right-hand side dense matrix
class DVecDMatMultExpr : public DenseVector< DVecDMatMultExpr<VT,MT>, true >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT::ResultType  VRT;  //!< Result type of the left-hand side dense vector expression.
   typedef typename MT::ResultType  MRT;  //!< Result type of the right-hand side dense matrix expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DVecDMatMultExpr<VT,MT>                This;           //!< Type of this DVecDMatMultExpr instance.
   typedef typename MathTrait<VRT,MRT>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::TransposeType     TransposeType;  //!< Transpose type for expression template evaluations.
   typedef const ResultType                       CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense vector expression.
   typedef typename SelectType<IsExpression<VT>::value,const VRT,const VT&>::Type  Lhs;

   //! Composite type of the right-hand side dense matrix expression.
   typedef typename MT::CompositeType  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DVecDMatMultExpr class.
   */
   explicit inline DVecDMatMultExpr( const VT& vec, const MT& mat )
      : vec_( vec )                                  // Left-hand side dense vector of the multiplication expression
      , mat_( mat )                                  // Right-hand side dense matrix of the multiplication expression
      , end_( ( (mat.rows()-1) & size_t(-2) ) + 1 )  // End of the unrolled calculation loop
   {
      pe_INTERNAL_ASSERT( vec_.size() == mat_.rows(), "Invalid vector and matrix sizes" );
   }
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline const ElementType operator[]( size_t index ) const {
      pe_INTERNAL_ASSERT( index < mat_.columns(), "Invalid vector access index" );

      ElementType res;

      if( mat_.rows() != size_t(0) ) {
         res = vec_[0] * mat_(0,index);
         for( size_t j=1; j<end_; j+=2 ) {
            res += vec_[j] * mat_(j,index) + vec_[j+1] * mat_(j+1,index);
         }
         if( end_ < mat_.rows() ) {
            res += vec_[end_] * mat_(end_,index);
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
      return mat_.columns();
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
      return ( vec_.isAliased( alias ) ) ||
             ( IsExpression<MT>::value && mat_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs          vec_;  //!< Left-hand side dense vector of the multiplication expression.
   Rhs          mat_;  //!< Right-hand side dense matrix of the multiplication expression.
   const size_t end_;  //!< End of the unrolled calculation loop.
   //**********************************************************************************************

   //**Assignment to dense vectors*****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-dense matrix multiplication to a transpose dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose dense vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-dense
   // matrix multiplication expression to a dense vector.
   */
   template< typename VT1 >  // Type of the target dense vector
   friend inline void assign( DenseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const size_t m  ( rhs.mat_.rows()    );
      const size_t n  ( rhs.mat_.columns() );
      const size_t end( n & size_t(-2) );

      if( n == 0 ) {
         reset( ~lhs );
         return;
      }

      for( size_t j=0; j<n; ++j ) {
         (~lhs)[j] = rhs.vec_[0] * rhs.mat_(0,j);
      }
      for( size_t i=1; i<m; ++i ) {
         for( size_t j=0; j<end; j+=2 ) {
            (~lhs)[j  ] += rhs.vec_[i] * rhs.mat_(i,j  );
            (~lhs)[j+1] += rhs.vec_[i] * rhs.mat_(i,j+1);
         }
         if( end < n ) {
            (~lhs)[end] += rhs.vec_[i] * rhs.mat_(i,end);
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse vectors****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-dense matrix multiplication to a transpose sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose sparse vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-dense
   // matrix multiplication expression to a sparse vector.
   */
   template< typename VT1 >  // Type of the target sparse vector
   friend inline void assign( SparseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      assign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to dense vectors********************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-dense matrix multiplication to a transpose
   //        dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose dense vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // dense matrix multiplication expression to a dense vector.
   */
   template< typename VT1 >  // Type of the target dense vector
   friend inline void addAssign( DenseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const size_t m  ( rhs.mat_.rows()    );
      const size_t n  ( rhs.mat_.columns() );
      const size_t end( n & size_t(-2) );

      if( n == 0 ) {
         reset( ~lhs );
         return;
      }

      for( size_t i=0; i<m; ++i ) {
         for( size_t j=0; j<end; j+=2 ) {
            (~lhs)[j  ] += rhs.vec_[i] * rhs.mat_(i,j  );
            (~lhs)[j+1] += rhs.vec_[i] * rhs.mat_(i,j+1);
         }
         if( end < n ) {
            (~lhs)[end] += rhs.vec_[i] * rhs.mat_(i,end);
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to sparse vectors*******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-dense matrix multiplication to a transpose
   //        sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose sparse vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // dense matrix multiplication expression to a sparse vector.
   */
   template< typename VT1 >  // Type of the target sparse vector
   friend inline void addAssign( SparseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      addAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to dense vectors*****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense vector-dense matrix multiplication to a transpose
   //        dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose dense vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // dense matrix multiplication expression to a dense vector.
   */
   template< typename VT1 >  // Type of the target dense vector
   friend inline void subAssign( DenseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const size_t m  ( rhs.mat_.rows()    );
      const size_t n  ( rhs.mat_.columns() );
      const size_t end( n & size_t(-2) );

      if( n == 0 ) {
         reset( ~lhs );
         return;
      }

      for( size_t i=0; i<m; ++i ) {
         for( size_t j=0; j<end; j+=2 ) {
            (~lhs)[j  ] -= rhs.vec_[i] * rhs.mat_(i,j  );
            (~lhs)[j+1] -= rhs.vec_[i] * rhs.mat_(i,j+1);
         }
         if( end < n ) {
            (~lhs)[end] -= rhs.vec_[i] * rhs.mat_(i,end);
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to sparse vectors****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense vector-dense matrix multiplication to a transpose
   //        sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose sparse vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // dense matrix multiplication expression to a sparse vector.
   */
   template< typename VT1 >  // Type of the target sparse vector
   friend inline void subAssign( SparseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      subAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to dense vectors**************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a dense vector-dense matrix multiplication to a transpose
   //        dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose dense vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // dense matrix multiplication expression to a dense vector.
   */
   template< typename VT1 >  // Type of the target dense vector
   friend inline void multAssign( DenseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      multAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to sparse vectors*************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a dense vector-dense matrix multiplication to a transpose
   //        sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side transpose sparse vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-dense matrix multiplication expression to a sparse vector.
   */
   template< typename VT1 >  // Type of the target sparse vector
   friend inline void multAssign( SparseVector<VT1,true>& lhs, const DVecDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, true );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      multAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT, true );
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT );
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
/*!\brief Multiplication operator for the multiplication of a dense vector and a dense matrix
 *        (\f$ \vec{a}^T=\vec{b}^T*C \f$).
 * \ingroup dense_matrix
 *
 * \param vec The left-hand side transpose dense vector for the multiplication.
 * \param mat The right-hand side dense matrix for the multiplication.
 * \return The resulting transpose vector.
 * \exception std::invalid_argument Vector and matrix sizes do not match.
 *
 * This operator represents the multiplication between a transpose dense vector and a dense
 * matrix:

   \code
   pe::MatN A;
   pe::VecNT x, y;
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
        , typename T2 >  // Type of the right-hand side dense matrix
inline const DVecDMatMultExpr<T1,T2>
   operator*( const DenseVector<T1,true>& vec, const DenseMatrix<T2>& mat )
{
   if( (~vec).size() != (~mat).rows() )
      throw std::invalid_argument( "Vector and matrix sizes do not match" );

   return DVecDMatMultExpr<T1,T2>( ~vec, ~mat );
}
//*************************************************************************************************

} // namespace pe

#endif
