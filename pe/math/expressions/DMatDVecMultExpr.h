//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatDVecMultExpr.h
 *  \brief Header file for the dense matrix/dense vector multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATDVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATDVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/BLAS.h>
#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsDouble.h>
#include <pe/util/typetraits/IsFloat.h>


namespace pe {

//=================================================================================================
//
//  CLASS DMATDVECMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense matrix-dense vector multiplications.
 * \ingroup dense_vector_expression
 *
 * The DMatDVecMultExpr class represents the compile time expression for multiplications
 * between dense matrices and dense vectors.
 */
template< typename MT    // Type of the left-hand side dense matrix
        , typename VT >  // Type of the right-hand side dense vector
class DMatDVecMultExpr : public DenseVector< DMatDVecMultExpr<MT,VT>, false >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT::ResultType    MRT;  //!< Result type of the left-hand side dense matrix expression.
   typedef typename VT::ResultType    VRT;  //!< Result type of the right-hand side dense vector expression.
   typedef typename MRT::ElementType  MET;  //!< Element type of the left-hand side dense matrix expression.
   typedef typename VRT::ElementType  VET;  //!< Element type of the right-hand side dense vector epxression.
   //**********************************************************************************************

   //**********************************************************************************************
   //! Compilation switch for the composite type of the left-hand side dense matrix expression.
   enum { blas = HAVE_BLAS && IsExpression<MT>::value &&
                 ( IsFloat<MET>::value || IsDouble<MET>::value ) &&
                 ( IsFloat<VET>::value || IsDouble<VET>::value ) };
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DMatDVecMultExpr<MT,VT>                This;           //!< Type of this DMatDVecMultExpr instance.
   typedef typename MathTrait<MRT,VRT>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::TransposeType     TransposeType;  //!< Transpose type for expression template evaluations.
   typedef const DMatDVecMultExpr&                CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense matrix expression.
   typedef typename SelectType<blas,const MRT,typename MT::CompositeType>::Type  Lhs;

   //! Composite type of the right-hand side dense vector expression.
   typedef typename SelectType<IsExpression<VT>::value,const VRT,const VT&>::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatDVecMultExpr class.
   */
   explicit inline DMatDVecMultExpr( const MT& mat, const VT& vec )
      : mat_( mat )                                     // Left-hand side dense matrix of the multiplication expression
      , vec_( vec )                                     // Right-hand side dense vector of the multiplication expression
      , end_( ( (mat.columns()-1) & size_t(-2) ) + 1 )  // End of the unrolled calculation loop
   {
      pe_INTERNAL_ASSERT( mat_.columns() == vec_.size(), "Invalid matrix and vector sizes" );
   }
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline const ElementType operator[]( size_t index ) const {
      pe_INTERNAL_ASSERT( index < mat_.rows(), "Invalid vector access index" );

      ElementType res;

      if( mat_.columns() != size_t(0) ) {
         res = mat_(index,0) * vec_[0];
         for( size_t j=1; j<end_; j+=2 ) {
            res += mat_(index,j) * vec_[j] + mat_(index,j+1) * vec_[j+1];
         }
         if( end_ < mat_.columns() ) {
            res += mat_(index,end_) * vec_[end_];
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
      return mat_.rows();
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
      return ( IsExpression<MT>::value && mat_.isAliased( alias ) ) ||
             ( vec_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs          mat_;  //!< Left-hand side dense matrix of the multiplication expression.
   Rhs          vec_;  //!< Right-hand side dense vector of the multiplication expression.
   const size_t end_;  //!< End of the unrolled calculation loop.
   //**********************************************************************************************

   //**Assignment to dense vectors*****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense matrix-dense vector multiplication to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   */
   template< typename VT1 >  // Type of the target dense vector
   friend inline void assign( DenseVector<VT1,false>& lhs, const DMatDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      if( rhs.mat_.columns() == size_t(0) ) {
         reset( ~lhs );
         return;
      }

      DMatDVecMultExpr::assign( ~lhs, rhs.mat_, rhs.vec_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Default assignment of a dense matrix-dense vector multiplication****************************
   /*! \cond PE_INTERNAL */
   /*!\brief Default assignment of a dense matrix-dense vector multiplication
   //        (\f$ \vec{y}=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix-dense
   // vector multiplication expression to a dense matrix.
   */
   template< typename VT1    // Type of the left-hand side target vector
           , typename MT1    // Type of the left-hand side matrix operand
           , typename VT2 >  // Type of the right-hand side vector operand
   static inline void assign( VT1& y, const MT1& A, const VT2& x )
   {
      pe_INTERNAL_ASSERT( A.columns() > 0, "Invalid number of columns" );

      y.assign( DMatDVecMultExpr<MT1,VT2>( A, x ) );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**BLAS-based assignment of a matrix-vector multiplication for single precision****************
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based assignment of a dense matrix-dense vector multiplication for single
   //        precision operands (\f$ \vec{y}=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function performs the dense matrix-dense vector multiplication for single precision
   // operands based on the BLAS cblas_sgemv() function.
   */
   template< template<typename,bool> class VT1    // Type of the left-hand side target vector
           , template<typename>      class MT1    // Type of the left-hand side matrix operand
           , template<typename,bool> class VT2 >  // Type of the right-hand side vector operand
   static inline void assign( VT1<float,false>& y, const MT1<float>& A, const VT2<float,false>& x )
   {
      const size_t M( A.rows()    );
      const size_t N( A.columns() );

      if( N > size_t(25) )
         cblas_sgemv( CblasRowMajor, CblasNoTrans, M, N, 1.0F,
                      &A(0,0), N, &x[0], 1, 0.0F, &y[0], 1 );
      else y.assign( DMatDVecMultExpr< MT1<float>, VT2<float,false> >( A, x ) );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**BLAS-based assignment of a matrix-vector multiplication for double precision****************
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based assignment of a dense matrix-dense vector multiplication for double
   //        precision operands (\f$ \vec{y}=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function performs the dense matrix-dense vector multiplication for double precision
   // operands based on the BLAS cblas_dgemv() function.
   */
   template< template<typename,bool> class VT1    // Type of the left-hand side target matrix
           , template<typename>      class MT1    // Type of the left-hand side matrix operand
           , template<typename,bool> class VT2 >  // Type of the right-hand side matrix operand
   static inline void assign( VT1<double,false>& y, const MT1<double>& A, const VT2<double,false>& x )
   {
      const size_t M( A.rows()    );
      const size_t N( A.columns() );

      if( N > size_t(25) )
         cblas_dgemv( CblasRowMajor, CblasNoTrans, M, N, 1.0,
                      &A(0,0), N, &x[0], 1, 0.0, &y[0], 1 );
      else y.assign( DMatDVecMultExpr< MT1<double>, VT2<double,false> >( A, x ) );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**Addition assignment to dense vectors********************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense matrix-dense vector multiplication to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   */
   template< typename VT1 >  // Type of the target dense vector
   friend inline void addAssign( DenseVector<VT1,false>& lhs, const DMatDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      if( rhs.mat_.columns() == size_t(0) ) {
         return;
      }

      DMatDVecMultExpr::addAssign( ~lhs, rhs.mat_, rhs.vec_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Default addition assignment of a dense matrix-dense vector multiplication*******************
   /*! \cond PE_INTERNAL */
   /*!\brief Default addition assignment of a dense matrix-dense vector multiplication
   //        (\f$ \vec{y}+=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // dense vector multiplication expression to a dense vector.
   */
   template< typename VT1    // Type of the left-hand side target vector
           , typename MT1    // Type of the left-hand side matrix operand
           , typename VT2 >  // Type of the right-hand side vector operand
   static inline void addAssign( VT1& y, const MT1& A, const VT2& x )
   {
      pe_INTERNAL_ASSERT( A.columns() > 0, "Invalid number of columns" );

      y.addAssign( DMatDVecMultExpr<MT1,VT2>( A, x ) );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**BLAS-based addition assignment of a matrix-vector multiplication for single precision*******
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based addition assignment of a matrix-vector multiplication for single
   //        precision operands (\f$ \vec{y}+=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function performs the dense matrix-dense vector multiplication for single precision
   // operands based on the BLAS cblas_sgemv() function.
   */
   template< template<typename,bool> class VT1    // Type of the left-hand side target vector
           , template<typename>      class MT1    // Type of the left-hand side matrix operand
           , template<typename,bool> class VT2 >  // Type of the right-hand side vector operand
   static inline void addAssign( VT1<float,false>& y, const MT1<float>& A, const VT2<float,false>& x )
   {
      const size_t M( A.rows()    );
      const size_t N( A.columns() );

      if( N > size_t(25) )
         cblas_sgemv( CblasRowMajor, CblasNoTrans, M, N, 1.0F,
                      &A(0,0), N, &x[0], 1, 1.0F, &y[0], 1 );
      else y.addAssign( DMatDVecMultExpr< MT1<float>, VT2<float,false> >( A, x ) );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**BLAS-based addition assignment of a matrix-vector multiplication for double precision*******
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based addition assignment of a matrix-vector multiplication for double
   //        precision operands (\f$ \vec{y}+=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function performs the dense matrix-dense vector multiplication for double precision
   // operands based on the BLAS cblas_dgemv() function.
   */
   template< template<typename,bool> class VT1    // Type of the left-hand side target vector
           , template<typename>      class MT1    // Type of the left-hand side matrix operand
           , template<typename,bool> class VT2 >  // Type of the right-hand side vector operand
   static inline void addAssign( VT1<double,false>& y, const MT1<double>& A, const VT2<double,false>& x )
   {
      const size_t M( A.rows()    );
      const size_t N( A.columns() );

      if( N > size_t(25) )
         cblas_dgemv( CblasRowMajor, CblasNoTrans, M, N, 1.0,
                      &A(0,0), N, &x[0], 1, 1.0, &y[0], 1 );
      else y.addAssign( DMatDVecMultExpr< MT1<double>, VT2<double,false> >( A, x ) );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**Subtraction assignment to dense vectors*****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense matrix-dense vector multiplication to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   */
   template< typename VT1 >  // Type of the target dense vector
   friend inline void subAssign( DenseVector<VT1,false>& lhs, const DMatDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      if( rhs.mat_.columns() == size_t(0) ) {
         return;
      }

      DMatDVecMultExpr::subAssign( ~lhs, rhs.mat_, rhs.vec_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Default subtraction assignment of a dense matrix-dense vector multiplication****************
   /*! \cond PE_INTERNAL */
   /*!\brief Default subtraction assignment of a dense matrix-dense vector multiplication
   //        (\f$ \vec{y}-=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // dense vector multiplication expression to a dense vector.
   */
   template< typename VT1    // Type of the left-hand side target vector
           , typename MT1    // Type of the left-hand side matrix operand
           , typename VT2 >  // Type of the right-hand side vector operand
   static inline void subAssign( VT1& y, const MT1& A, const VT2& x )
   {
      pe_INTERNAL_ASSERT( A.columns() > 0, "Invalid number of columns" );

      y.subAssign( DMatDVecMultExpr<MT1,VT2>( A, x ) );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**BLAS-based subtraction assignment of a matrix-vector multiplication for single precision****
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based subtraction assignment of a matrix-vector multiplication for single
   //        precision operands (\f$ \vec{y}-=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function performs the dense matrix-dense vector multiplication for single precision
   // operands based on the BLAS cblas_sgemv() function.
   */
   template< template<typename,bool> class VT1    // Type of the left-hand side target vector
           , template<typename>      class MT1    // Type of the left-hand side matrix operand
           , template<typename,bool> class VT2 >  // Type of the right-hand side vector operand
   static inline void subAssign( VT1<float,false>& y, const MT1<float>& A, const VT2<float,false>& x )
   {
      const size_t M( A.rows()    );
      const size_t N( A.columns() );

      if( N > size_t(25) )
         cblas_sgemv( CblasRowMajor, CblasNoTrans, M, N, -1.0F,
                      &A(0,0), N, &x[0], 1, 1.0F, &y[0], 1 );
      else y.subAssign( DMatDVecMultExpr< MT1<float>, VT2<float,false> >( A, x ) );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**BLAS-based subtraction assignment of a matrix-vector multiplication for double precision****
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based subtraction assignment of a matrix-vector multiplication for double
   //        precision operands (\f$ \vec{y}-=A*\vec{x} \f$).
   // \ingroup dense_vector
   //
   // \param y The target left-hand side dense vector.
   // \param A The left-hand side dense matrix operand.
   // \param x The right-hand side dense vector operand.
   // \return void
   //
   // This function performs the dense matrix-dense vector multiplication for double precision
   // operands based on the BLAS cblas_dgemv() function.
   */
   template< template<typename,bool> class VT1    // Type of the left-hand side target vector
           , template<typename>      class MT1    // Type of the left-hand side matrix operand
           , template<typename,bool> class VT2 >  // Type of the right-hand side vector operand
   static inline void subAssign( VT1<double,false>& y, const MT1<double>& A, const VT2<double,false>& x )
   {
      const size_t M( A.rows()    );
      const size_t N( A.columns() );

      if( N > size_t(25) )
         cblas_dgemv( CblasRowMajor, CblasNoTrans, M, N, -1.0,
                      &A(0,0), N, &x[0], 1, 1.0, &y[0], 1 );
      else y.subAssign( DMatDVecMultExpr< MT1<double>, VT2<double,false> >( A, x ) );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT );
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT, false );
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
/*!\brief Multiplication operator for the multiplication of a dense matrix and a dense vector
 *        (\f$ \vec{a}=B*\vec{c} \f$).
 * \ingroup dense_matrix
 *
 * \param mat The left-hand side dense matrix for the multiplication.
 * \param vec The right-hand side dense vector for the multiplication.
 * \return The resulting vector.
 * \exception std::invalid_argument Matrix and vector sizes do not match.
 *
 * This operator represents the multiplication between a dense matrix and a dense vector:

   \code
   pe::MatN A;
   pe::VecN x, y;
   // ... Resizing and initialization
   y = A * x;
   \endcode

 * The operator returns an expression representing a dense vector of the higher-order element
 * type of the two involved element types \a T1::ElementType and \a T2::ElementType. Both the
 * dense matrix type \a T1 and the dense vector type \a T2 as well as the two element types
 * \a T1::ElementType and \a T2::ElementType have to be supported by the MathTrait class
 * template.\n
 * In case the current size of the vector \a vec doesn't match the current number of columns
 * of the matrix \a mat, a \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side dense vector
inline const DMatDVecMultExpr<T1,T2>
   operator*( const DenseMatrix<T1>& mat, const DenseVector<T2,false>& vec )
{
   if( (~mat).columns() != (~vec).size() )
      throw std::invalid_argument( "Matrix and vector sizes do not match" );

   return DMatDVecMultExpr<T1,T2>( ~mat, ~vec );
}
//*************************************************************************************************

} // namespace pe

#endif
