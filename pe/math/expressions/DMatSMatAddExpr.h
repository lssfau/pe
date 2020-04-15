//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatSMatAddExpr.h
 *  \brief Header file for the dense matrix/sparse matrix addition expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATSMATADDEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATSMATADDEXPR_H_


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
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Reference.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DMATSMATADDEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense matrix-sparse matrix additions.
 * \ingroup dense_matrix_expression
 *
 * The DMatSMatAddExpr class represents the compile time expression for additions between
 * a dense matrix and a sparse matrix.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side sparse matrix
class DMatSMatAddExpr : public DenseMatrix< DMatSMatAddExpr<MT1,MT2> >
                      , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT1::ResultType  RT1;  //!< Result type of the left-hand side dense matrix expression.
   typedef typename MT2::ResultType  RT2;  //!< Result type of the right-hand side sparse matrix expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename MathTrait<RT1,RT2>::AddType  ResultType;     //!< Result type for expression template evaluations.
   typedef const ResultType                      CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType      ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense matrix expression.
   typedef typename MT1::CompositeType  Lhs;

   //! Composite type of the right-hand side sparse matrix expression.
   typedef typename MT2::CompositeType  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatSMatAddExpr class.
   */
   explicit inline DMatSMatAddExpr( const MT1& lhs, const MT2& rhs )
      : lhs_( lhs )  // Left-hand side dense matrix of the addition expression
      , rhs_( rhs )  // Right-hand side sparse matrix of the addition expression
   {
      pe_INTERNAL_ASSERT( lhs.rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( lhs.columns() == rhs.columns(), "Invalid number of columns" );
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
      pe_INTERNAL_ASSERT( j < lhs_.columns(), "Invalid column access index" );
      return lhs_(i,j) + rhs_(i,j);
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
      return lhs_.columns();
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
      return ( IsExpression<MT1>::value && lhs_.isAliased( alias ) ) ||
             ( rhs_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs lhs_;  //!< Left-hand side dense matrix of the addition expression.
   Rhs rhs_;  //!< Right-hand side sparse matrix of the addition expression.
   //**********************************************************************************************

   //**Assignment to dense matrices****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense matrix-sparse matrix addition to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side addition expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix-sparse
   // matrix addition expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void assign( DenseMatrix<MT>& lhs, const DMatSMatAddExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      assign   ( ~lhs, rhs.lhs_ );
      addAssign( ~lhs, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse matrices***************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense matrix-sparse matrix addition to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side addition expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense matrix-sparse
   // matrix addition expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void assign( SparseMatrix<MT>& lhs, const DMatSMatAddExpr& rhs )
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
   /*!\brief Addition assignment of a dense matrix-sparse matrix addition to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side addition expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // sparse matrix addition expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void addAssign( DenseMatrix<MT>& lhs, const DMatSMatAddExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      addAssign( ~lhs, rhs.lhs_ );
      addAssign( ~lhs, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to sparse matrices******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense matrix-sparse matrix addition to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side addition expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense matrix-
   // sparse matrix addition expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void addAssign( SparseMatrix<MT>& lhs, const DMatSMatAddExpr& rhs )
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
   /*!\brief Subtraction assignment of a dense matrix-sparse matrix addition to a dense matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side dense matrix.
   // \param rhs The right-hand side addition expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // sparse matrix addition expression to a dense matrix.
   */
   template< typename MT >  // Type of the target dense matrix
   friend inline void subAssign( DenseMatrix<MT>& lhs, const DMatSMatAddExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).rows()    == rhs.rows()   , "Invalid number of rows"    );
      pe_INTERNAL_ASSERT( (~lhs).columns() == rhs.columns(), "Invalid number of columns" );

      subAssign( ~lhs, rhs.lhs_ );
      subAssign( ~lhs, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to sparse matrices***************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense matrix-sparse matrix addition to a sparse matrix.
   // \ingroup dense_matrix
   //
   // \param lhs The target left-hand side sparse matrix.
   // \param rhs The right-hand side addition expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense matrix-
   // sparse matrix addition expression to a sparse matrix.
   */
   template< typename MT >  // Type of the target sparse matrix
   friend inline void subAssign( SparseMatrix<MT>& lhs, const DMatSMatAddExpr& rhs )
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
/*!\brief Addition operator for the addition of a dense matrix and a sparse matrix
 *        (\f$ A=B+C \f$).
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side dense matrix for the matrix addition.
 * \param rhs The right-hand side sparse matrix to be added to the left-hand side matrix.
 * \return The sum of the two matrices.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This operator represents the addition of a dense matrix and a sparse matrix:

   \code
   pe::MatN A, C;
   pe::SMatN B;
   // ... Resizing and initialization
   C = A + B;
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
const DMatSMatAddExpr<T1,T2>
   operator+( const DenseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   if( (~lhs).rows() != (~rhs).rows() || (~lhs).columns() != (~rhs).columns() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   return DMatSMatAddExpr<T1,T2>( ~lhs, ~rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition operator for the addition of a sparse matrix and a dense matrix
 *        (\f$ A=B+C \f$).
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side sparse matrix for the matrix addition.
 * \param rhs The right-hand side dense matrix to be added to the left-hand side matrix.
 * \return The sum of the two matrices.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This operator represents the addition of a sparse matrix and a dense matrix:

   \code
   pe::SMatN A;
   pe::MatN B, C;
   // ... Resizing and initialization
   C = A + B;
   \endcode

 * The operator returns an expression representing a dense matrix of the higher-order element
 * type of the two involved matrix element types \a T1::ElementType and \a T2::ElementType.
 * Both matrix types \a T1 and \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side dense matrix
const DMatSMatAddExpr<T2,T1>
   operator+( const SparseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs )
{
   if( (~lhs).rows() != (~rhs).rows() || (~lhs).columns() != (~rhs).columns() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   return DMatSMatAddExpr<T2,T1>( ~rhs, ~lhs );
}
//*************************************************************************************************

} // namespace pe

#endif
