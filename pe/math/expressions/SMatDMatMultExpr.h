//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatDMatMultExpr.h
 *  \brief Header file for the sparse matrix/dense matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATDMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATDMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/constraints/SparseMatrix.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS SMATDMATMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for sparse matrix-dense matrix multiplications.
 * \ingroup dense_matrix_expression
 *
 * The SMatDMatMultExpr class represents the compile time expression for multiplications between
 * a sparse matrix and a dense matrix.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side sparse matrix
class SMatDMatMultExpr : public DenseMatrix< SMatDMatMultExpr<MT1,MT2> >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT1::ResultType     RT1;  //!< Result type of the left-hand side sparse matrix expression.
   typedef typename MT2::ResultType     RT2;  //!< Result type of the right-hand side dense matrix expression.
   typedef typename MT1::CompositeType  CT1;  //!< Composite type of the left-hand side sparse matrix expression.
   typedef typename MT2::CompositeType  CT2;  //!< Composite type of the right-hand side dense matrix expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename MathTrait<RT1,RT2>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef const SMatDMatMultExpr&                CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side sparse matrix expression.
   typedef typename SelectType<IsExpression<MT1>::value,const RT1,CT1>::Type  Lhs;

   //! Composite type of the right-hand side dense matrix expression.
   typedef typename SelectType<IsExpression<MT2>::value,const RT2,CT2>::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the SMatDMatMultExpr class.
   */
   explicit inline SMatDMatMultExpr( const MT1& lhs, const MT2& rhs )
      : lhs_( lhs )  // Left-hand side sparse matrix of the multiplication expression
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

      typedef typename MT1::ConstIterator  ConstIterator;

      ElementType tmp;
      ConstIterator element( lhs_.begin(i) );
      const ConstIterator end( lhs_.end(i) );

      if( element != end ) {
         tmp = element->value() * rhs_(element->index(),j);
         ++element;
         for( ; element!=end; ++element ) {
            tmp += element->value() * rhs_(element->index(),j);
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
   Lhs lhs_;  //!< Left-hand side sparse matrix of the multiplication expression.
   Rhs rhs_;  //!< Right-hand side dense matrix of the multiplication expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SPARSE_MATRIX_TYPE( MT1 );
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE ( MT2 );
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
/*!\brief Multiplication operator for the multiplication of a sparse matrix and a dense matrix
 *        (\f$ A=B*C \f$).
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side sparse matrix for the multiplication.
 * \param rhs The right-hand side dense matrix for the multiplication.
 * \return The resulting matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This operator represents the multiplication of a sparse matrix and a dense matrix:

   \code
   pe::SMatN A;
   pe::MatN B, C;
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
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side dense matrix
inline const SMatDMatMultExpr<T1,T2>
   operator*( const SparseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs )
{
   if( (~lhs).columns() != (~rhs).rows() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   return SMatDMatMultExpr<T1,T2>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
