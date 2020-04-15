//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatSMatMultExpr.h
 *  \brief Header file for the dense matrix/sparse matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATSMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATSMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/constraints/SparseMatrix.h>
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
//  CLASS DMATSMATMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense matrix-sparse matrix multiplications.
 * \ingroup dense_matrix_expression
 *
 * The DMatSMatMultExpr class represents the compile time expression for multiplications between
 * a dense matrix and a sparse matrix.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side sparse matrix
class DMatSMatMultExpr : public DenseMatrix< DMatSMatMultExpr<MT1,MT2> >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT1::ResultType     RT1;  //!< Result type of the left-hand side dense matrix expression.
   typedef typename MT2::ResultType     RT2;  //!< Result type of the right-hand side sparse matrix expression.
   typedef typename MT1::CompositeType  CT1;  //!< Composite type of the left-hand side dense matrix expression.
   typedef typename MT2::CompositeType  CT2;  //!< Composite type of the right-hand side sparse matrix expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename MathTrait<RT1,RT2>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef const ResultType                       CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense matrix expression.
   typedef typename SelectType<IsExpression<MT1>::value,const RT1,CT1>::Type  Lhs;

   //! Composite type of the right-hand side sparse matrix expression.
   typedef const RT2  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatSMatMultExpr class.
   */
   explicit inline DMatSMatMultExpr( const MT1& lhs, const MT2& rhs )
      : lhs_ ( lhs )         // Left-hand side dense matrix of the multiplication expression
      , rhsT_( trans(rhs) )  // Right-hand side sparse matrix of the multiplication expression
   {
      pe_INTERNAL_ASSERT( lhs.columns() == rhs.rows(), "Invalid matrix sizes" );
      pe_INTERNAL_ASSERT( rhs.rows() == rhsT_.columns(), "Invalid matrix transpose" );
      pe_INTERNAL_ASSERT( rhs.columns() == rhsT_.rows(), "Invalid matrix transpose" );
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
      pe_INTERNAL_ASSERT( i < lhs_.rows() , "Invalid row access index"    );
      pe_INTERNAL_ASSERT( j < rhsT_.rows(), "Invalid column access index" );

      typedef typename RT2::ConstIterator  ConstIterator;

      ElementType tmp;
      ConstIterator element( rhsT_.begin(j) );
      const ConstIterator end( rhsT_.end(j) );

      if( element != end ) {
         tmp = lhs_(i,element->index()) * element->value();
         ++element;
         for( ; element!=end; ++element ) {
            tmp += lhs_(i,element->index()) * element->value();
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
      return rhsT_.rows();
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
      return lhs_.isAliased( alias );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs lhs_;   //!< Left-hand side dense matrix of the multiplication expression.
   Rhs rhsT_;  //!< Right-hand side sparse matrix of the multiplication expression.
   //**********************************************************************************************

   //**Assignment to dense matrices****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense matrix-sparse matrix multiplication to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix-sparse
   // matrix multiplication expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void assign( DenseMatrix<MT>& lhs, const DMatSMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      typedef typename RT2::ConstIterator  ConstIterator;

      for( size_t i=0; i<(~lhs).rows(); ++i ) {
         for( size_t j=0; j<(~rhs).columns(); ++j ) {
            ConstIterator element( rhs.rhsT_.begin(j) );
            if( element != rhs.rhsT_.end(j) ) {
               (~lhs)(i,j) = rhs.lhs_(i,element->index()) * element->value();
               ++element;
               for( ; element!=rhs.rhsT_.end(j); ++element ) {
                  (~lhs)(i,j) += rhs.lhs_(i,element->index()) * element->value();
               }
            }
            else {
               reset( (~lhs)(i,j) );
            }
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse matrices***************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense matrix-sparse matrix multiplication to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix-sparse
   // matrix multiplication expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void assign( SparseMatrix<MT>& lhs, const DMatSMatMultExpr& rhs )
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
   /*!\brief Addition assignment of a dense matrix-sparse matrix multiplication to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // sparse matrix multiplication expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void addAssign( DenseMatrix<MT>& lhs, const DMatSMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      typedef typename RT2::ConstIterator  ConstIterator;

      for( size_t i=0; i<(~lhs).rows(); ++i ) {
         for( size_t j=0; j<(~rhs).columns(); ++j ) {
            const ConstIterator end( rhs.rhsT_.end(j) );
            for( ConstIterator element=rhs.rhsT_.begin(j); element!=end; ++element ) {
               (~lhs)(i,j) += rhs.lhs_(i,element->index()) * element->value();
            }
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to sparse matrices******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense matrix-sparse matrix multiplication to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // sparse matrix multiplication expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void addAssign( SparseMatrix<MT>& lhs, const DMatSMatMultExpr& rhs )
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
   /*!\brief Subtraction assignment of a dense matrix-sparse matrix multiplication to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // sparse matrix multiplication expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void subAssign( DenseMatrix<MT>& lhs, const DMatSMatMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      typedef typename RT2::ConstIterator  ConstIterator;

      for( size_t i=0; i<(~lhs).rows(); ++i ) {
         for( size_t j=0; j<(~rhs).columns(); ++j ) {
            const ConstIterator end( rhs.rhsT_.end(j) );
            for( ConstIterator element=rhs.rhsT_.begin(j); element!=end; ++element ) {
               (~lhs)(i,j) -= rhs.lhs_(i,element->index()) * element->value();
            }
         }
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to sparse matrices***************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense matrix-sparse matrix multiplication to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // sparse atrix multiplication expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void subAssign( SparseMatrix<MT>& lhs, const DMatSMatMultExpr& rhs )
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
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE ( MT1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( MT2 );
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
/*!\brief Multiplication operator for the multiplication of a dense matrix and a sparse matrix
 *        (\f$ A=B*C \f$).
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side dense matrix for the multiplication.
 * \param rhs The right-hand side sparse matrix for the multiplication.
 * \return The resulting matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This operator represents the multiplication of a dense matrix and a sparse matrix:

   \code
   pe::MatN A, C;
   pe::SMatN B;
   // ... Resizing and initialization
   C = A * B;
   \endcode

 * The operator returns an expression representing a dense matrix of the higher-order element
 * type of the two involved matrix element types \a T1::ElementType and \a T2::ElementType.
 * Both matrix types \a T1 and \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side sparse matrix
inline const DMatSMatMultExpr<T1,T2>
   operator*( const DenseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   if( (~lhs).columns() != (~rhs).rows() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   return DMatSMatMultExpr<T1,T2>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
