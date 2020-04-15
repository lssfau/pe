//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatDMatMultExpr.h
 *  \brief Header file for the dense matrix/dense matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATDMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATDMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/BLAS.h>
#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/expressions/SparseMatrix.h>
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
//  CLASS DMATDMATMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense matrix-dense matrix multiplications.
 * \ingroup dense_matrix_expression
 *
 * The DMatDMatMultExpr class represents the compile time expression for multiplications between
 * dense matrices.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side dense matrix
class DMatDMatMultExpr : public DenseMatrix< DMatDMatMultExpr<MT1,MT2> >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT1::ResultType     RT1;  //!< Result type of the left-hand side dense matrix expression.
   typedef typename MT2::ResultType     RT2;  //!< Result type of the right-hand side dense matrix expression.
   typedef typename MT1::CompositeType  CT1;  //!< Composite type of the left-hand side dense matrix expression.
   typedef typename MT2::CompositeType  CT2;  //!< Composite type of the right-hand side dense matrix expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename MathTrait<RT1,RT2>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef const ResultType                       CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense matrix expression.
   typedef typename SelectType<IsExpression<MT1>::value,const RT1,CT1>::Type  Lhs;

   //! Composite type of the right-hand side dense matrix expression.
   typedef typename SelectType<IsExpression<MT2>::value,const RT2,CT2>::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatDMatMultExpr class.
   */
   explicit inline DMatDMatMultExpr( const MT1& lhs, const MT2& rhs )
      : lhs_( lhs )  // Left-hand side dense matrix of the multiplication expression
      , rhs_( rhs )  // Right-hand side dense matrix of the multiplication expression
   {
      pe_INTERNAL_ASSERT( lhs.columns() == rhs.rows(), "Invalid matrix sizes" );
   }
   //**********************************************************************************************

   //**Access operator*****************************************************************************
   /*!\brief 2D-access to the matrix elements.
   //
   // \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
   // \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
   // \return Reference to the accessed value.
   */
   inline const ElementType operator()( size_t i, size_t j ) const {
      pe_INTERNAL_ASSERT( i < lhs_.rows()   , "Invalid row access index"    );
      pe_INTERNAL_ASSERT( j < rhs_.columns(), "Invalid column access index" );

      ElementType tmp;

      if( lhs_.columns() != 0 ) {
         const size_t end( ( ( lhs_.columns()-1 ) & size_t(-2) ) + 1 );
         tmp = lhs_(i,0) * rhs_(0,j);
         for( size_t k=1; k<end; k+=2 ) {
            tmp += lhs_(i,k  ) * rhs_(k  ,j);
            tmp += lhs_(i,k+1) * rhs_(k+1,j);
         }
         if( end < lhs_.columns() ) {
            tmp += lhs_(i,end) * rhs_(end,j);
         }
      }
      else {
         reset( tmp );
      }

      return tmp;
   }
   //**********************************************************************************************

   //**Rows function*******************************************************************************
   /*!\brief Returns the current number of rows of the matrix.
   //
   // \return The number of rows of the matrix.
   */
   inline size_t rows() const {
      return lhs_.rows();
   }
   //**********************************************************************************************

   //**Columns function****************************************************************************
   /*!\brief Returns the current number of columns of the matrix.
   //
   // \return The number of columns of the matrix.
   */
   inline size_t columns() const {
      return rhs_.columns();
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the expression is aliased with the given address \a alias.
   //
   // \param alias The alias to be checked.
   // \return \a true in case an alias effect is detected, \a false otherwise.
   */
   template< typename T >
   inline bool isAliased( const T* alias ) const {
      return lhs_.isAliased( alias ) || rhs_.isAliased( alias );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs lhs_;  //!< Left-hand side dense matrix of the multiplication expression.
   Rhs rhs_;  //!< Right-hand side dense matrix of the multiplication expression.
   //**********************************************************************************************

   //**Assignment to dense matrices****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense matrix-dense matrix multiplication to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void assign( DenseMatrix<MT>& lhs, const DMatDMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      if( rhs.lhs_.columns() == 0 ) {
         reset( ~lhs );
         return;
      }

      DMatDMatMultExpr::assign( ~lhs, rhs.lhs_, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Default assignment of a dense matrix-dense matrix multiplication****************************
   /*! \cond PE_INTERNAL */
   /*!\brief Default assignment of a dense matrix-dense matrix multiplication (\f$ C=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix-dense
   // matrix multiplication expression to a dense matrix.
   */
   template< typename MT3    // Type of the left-hand side target matrix
           , typename MT4    // Type of the left-hand side matrix operand
           , typename MT5 >  // Type of the right-hand side matrix operand
   static inline void assign( MT3& C, const MT4& A, const MT5& B )
   {
      for( size_t i=0; i<A.rows(); ++i ) {
         for( size_t k=0; k<B.columns(); ++k ) {
            C(i,k) = A(i,0) * B(0,k);
         }
         for( size_t j=1; j<A.columns(); ++j ) {
            for( size_t k=0; k<B.columns(); ++k ) {
               C(i,k) += A(i,j) * B(j,k);
            }
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**BLAS-based assignment of a matrix-matrix multiplication for single precision****************
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based assignment of a dense matrix-dense matrix multiplication for single
   //        precision matrices (\f$ C=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function performs the dense matrix-dense matrix multiplication for single precision
   // matrices based on the BLAS cblas_sgemm() function.
   */
   template< template<typename> class MT3    // Type of the left-hand side target matrix
           , template<typename> class MT4    // Type of the left-hand side matrix operand
           , template<typename> class MT5 >  // Type of the right-hand side matrix operand
   static inline void assign( MT3<float>& C, const MT4<float>& A, const MT5<float>& B )
   {
      const size_t M( A.rows()    );
      const size_t N( B.columns() );
      const size_t K( A.columns() );
      cblas_sgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, 1.0F,
                   &A(0,0), K, &B(0,0), N, 0.0F, &C(0,0), N );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**BLAS-based assignment of a matrix-matrix multiplication for double precision****************
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based assignment of a dense matrix-dense matrix multiplication for double
   //        precision matrices (\f$ C=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function performs the dense matrix-dense matrix multiplication for double precision
   // matrices based on the BLAS cblas_dgemm() function.
   */
   template< template<typename> class MT3    // Type of the left-hand side target matrix
           , template<typename> class MT4    // Type of the left-hand side matrix operand
           , template<typename> class MT5 >  // Type of the right-hand side matrix operand
   static inline void assign( MT3<double>& C, const MT4<double>& A, const MT5<double>& B )
   {
      const size_t M( A.rows()    );
      const size_t N( B.columns() );
      const size_t K( A.columns() );
      cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, 1.0,
                   &A(0,0), K, &B(0,0), N, 0.0, &C(0,0), N );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**Assignment to sparse matrices***************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense matrix-dense matrix multiplication to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix-dense
   // matrix multiplication expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void assign( SparseMatrix<MT>& lhs, const DMatDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      const ResultType tmp( rhs );
      assign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to dense matrices*******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense matrix-dense matrix multiplication to a dense matrix.
   //        (\f$ C+=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // dense matrix multiplication expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void addAssign( DenseMatrix<MT>& lhs, const DMatDMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      DMatDMatMultExpr::addAssign( ~lhs, rhs.lhs_, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Default addition assignment of a dense matrix-dense matrix multiplication*******************
   /*! \cond PE_INTERNAL */
   /*!\brief Default addition assignment of a dense matrix-dense matrix multiplication (\f$ C+=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // dense matrix multiplication expression to a dense matrix.
   */
   template< typename MT3    // Type of the left-hand side target matrix
           , typename MT4    // Type of the left-hand side matrix operand
           , typename MT5 >  // Type of the right-hand side matrix operand
   static inline void addAssign( MT3& C, const MT4& A, const MT5& B )
   {
      const size_t end( B.columns() & size_t(-2) );

      for( size_t i=0; i<A.rows(); ++i ) {
         for( size_t j=0; j<A.columns(); ++j ) {
            for( size_t k=0; k<end; k+=2 ) {
               C(i,k  ) += A(i,j) * B(j,k  );
               C(i,k+1) += A(i,j) * B(j,k+1);
            }
            if( end < B.columns() ) {
               C(i,end) += A(i,j) * B(j,end);
            }
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**BLAS-based addition assignment of a matrix-matrix multiplication for single precision*******
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based addition assignment of a matrix-matrix multiplication for single
   //        precision matrices (\f$ C+=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function performs the dense matrix-dense matrix multiplication for single precision
   // matrices based on the BLAS cblas_sgemm() function.
   */
   template< template<typename> class MT3    // Type of the left-hand side target matrix
           , template<typename> class MT4    // Type of the left-hand side matrix operand
           , template<typename> class MT5 >  // Type of the right-hand side matrix operand
   static inline void addAssign( MT3<float>& C, const MT4<float>& A, const MT5<float>& B )
   {
      const size_t M( A.rows()    );
      const size_t N( B.columns() );
      const size_t K( A.columns() );
      cblas_sgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, 1.0F,
                   &A(0,0), K, &B(0,0), N, 1.0F, &C(0,0), N );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**BLAS-based addition assignment of a matrix-matrix multiplication for double precision*******
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based addition assignment of a matrix-matrix multiplication for double
   //        precision matrices (\f$ C+=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function performs the dense matrix-dense matrix multiplication for double precision
   // matrices based on the BLAS cblas_sgemm() function.
   */
   template< template<typename> class MT3    // Type of the left-hand side target matrix
           , template<typename> class MT4    // Type of the left-hand side matrix operand
           , template<typename> class MT5 >  // Type of the right-hand side matrix operand
   static inline void addAssign( MT3<double>& C, const MT4<double>& A, const MT5<double>& B )
   {
      const size_t M( A.rows()    );
      const size_t N( B.columns() );
      const size_t K( A.columns() );
      cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, 1.0,
                   &A(0,0), K, &B(0,0), N, 1.0, &C(0,0), N );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**Addition assignment to sparse matrices******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense matrix-dense matrix multiplication to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // dense matrix multiplication expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void addAssign( SparseMatrix<MT>& lhs, const DMatDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      const ResultType tmp( rhs );
      addAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to dense matrices****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense matrix-dense matrix multiplication to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // dense matrix multiplication expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void subAssign( DenseMatrix<MT>& lhs, const DMatDMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      DMatDMatMultExpr::subAssign( ~lhs, rhs.lhs_, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Default subtraction assignment of a dense matrix-dense matrix multiplication****************
   /*! \cond PE_INTERNAL */
   /*!\brief Default subtraction assignment of a dense matrix-dense matrix multiplication
   //        (\f$ C-=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // dense matrix multiplication expression to a dense matrix.
   */
   template< typename MT3    // Type of the left-hand side target matrix
           , typename MT4    // Type of the left-hand side matrix operand
           , typename MT5 >  // Type of the right-hand side matrix operand
   static inline void subAssign( MT3& C, const MT4& A, const MT5& B )
   {
      const size_t end( B.columns() & size_t(-2) );

      for( size_t i=0; i<A.rows(); ++i ) {
         for( size_t j=0; j<A.columns(); ++j ) {
            for( size_t k=0; k<end; k+=2 ) {
               C(i,k  ) -= A(i,j) * B(j,k  );
               C(i,k+1) -= A(i,j) * B(j,k+1);
            }
            if( end < B.columns() ) {
               C(i,end) -= A(i,j) * B(j,end);
            }
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**BLAS-based subraction assignment of a matrix-matrix multiplication for single precision*****
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based subraction assignment of a matrix-matrix multiplication for single
   //        precision matrices (\f$ C-=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function performs the dense matrix-dense matrix multiplication for single precision
   // matrices based on the BLAS cblas_sgemm() function.
   */
   template< template<typename> class MT3    // Type of the left-hand side target matrix
           , template<typename> class MT4    // Type of the left-hand side matrix operand
           , template<typename> class MT5 >  // Type of the right-hand side matrix operand
   static inline void subAssign( MT3<float>& C, const MT4<float>& A, const MT5<float>& B )
   {
      const size_t M( A.rows()    );
      const size_t N( B.columns() );
      const size_t K( A.columns() );
      cblas_sgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, -1.0F,
                   &A(0,0), K, &B(0,0), N, 1.0F, &C(0,0), N );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**BLAS-based subraction assignment of a matrix-matrix multiplication for double precision*****
#if HAVE_BLAS
   /*! \cond PE_INTERNAL */
   /*!\brief BLAS-based subraction assignment of a matrix-matrix multiplication for double
   //        precision matrices (\f$ C-=A*B \f$).
   // \ingroup dense_matrix
   //
   // \param C The target left-hand side dense matrix.
   // \param A The left-hand side multiplication operand.
   // \param B The right-hand side multiplication operand.
   // \return void
   //
   // This function performs the dense matrix-dense matrix multiplication for double precision
   // matrices based on the BLAS cblas_sgemm() function.
   */
   template< template<typename> class MT3    // Type of the left-hand side target matrix
           , template<typename> class MT4    // Type of the left-hand side matrix operand
           , template<typename> class MT5 >  // Type of the right-hand side matrix operand
   static inline void subAssign( MT3<double>& C, const MT4<double>& A, const MT5<double>& B )
   {
      const size_t M( A.rows()    );
      const size_t N( B.columns() );
      const size_t K( A.columns() );
      cblas_dgemm( CblasRowMajor, CblasNoTrans, CblasNoTrans, M, N, K, -1.0,
                   &A(0,0), K, &B(0,0), N, 1.0, &C(0,0), N );
   }
   /*! \endcond */
#endif
   //**********************************************************************************************

   //**Subtraction assignment to sparse matrices***************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense matrix-dense matrix multiplication to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // dense matrix multiplication expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void subAssign( SparseMatrix<MT>& lhs, const DMatDMatMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      const ResultType tmp( rhs );
      subAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT1 );
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT2 );
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
/*!\brief Multiplication operator for the multiplication of two matrices (\f$ A=B*C \f$).
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side matrix for the multiplication.
 * \param rhs The right-hand side matrix for the multiplication.
 * \return The resulting matrix.
 *
 * This operator represents the multiplication of two dense matrices:

   \code
   pe::MatN A, B, C;
   // ... Resizing and initialization
   C = A * B;
   \endcode

 * The operator returns an expression representing a dense matrix of the higher-order element
 * type of the two involved matrix element types \a T1::ElementType and \a T2::ElementType.
 * Both matrix types \a T1 and \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current number of columns of \a lhs and the current number of rows of \a rhs
 * don't match, a \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side dense matrix
inline const DMatDMatMultExpr<T1,T2>
   operator*( const DenseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs )
{
   if( (~lhs).columns() != (~rhs).rows() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   return DMatDMatMultExpr<T1,T2>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
