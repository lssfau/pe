//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatScalarDivExpr.h
 *  \brief Header file for the dense matrix/scalar division expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATSCALARDIVEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATSCALARDIVEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/EnableIf.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsFloatingPoint.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  CLASS DMATSCALARDIVEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for divisions of a dense matrix by a scalar.
 * \ingroup dense_matrix_expression
 *
 * The DMatScalarDivExpr class represents the compile time expression for divisions of dense
 * matrices and by scalar values.
 */
template< typename MT    // Type of the left-hand side dense matrix
        , typename ST >  // Type of the right-hand side scalar value
class DMatScalarDivExpr : public DenseMatrix< DMatScalarDivExpr<MT,ST> >
                        , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT::ResultType  RT;  //!< Result type of the dense matrix expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename MathTrait<RT,ST>::DivType  ResultType;     //!< Result type for expression template evaluations.
   typedef const DMatScalarDivExpr&            CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType    ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense matrix expression.
   typedef typename MT::CompositeType  Lhs;

   //! Composite type of the right-hand side scalar value.
   typedef typename SelectType<IsNumeric<ElementType>::value,ElementType,ST>::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatScalarDivExpr class.
   */
   explicit inline DMatScalarDivExpr( const MT& matrix, ST scalar )
      : matrix_( matrix )
      , scalar_( ( IsNumeric<ElementType>::value && IsFloatingPoint<ElementType>::value ) ?
                 ( Rhs(1)/scalar ) : ( scalar ) )
   {}
   //**********************************************************************************************

   //**Access operator*****************************************************************************
   /*!\brief 2D-access to the matrix elements.
   //
   // \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
   // \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
   // \return Reference to the accessed value.
   */
   inline const ElementType operator()( size_t i, size_t j ) const {
      pe_INTERNAL_ASSERT( i < matrix_.rows()   , "Invalid row access index"    );
      pe_INTERNAL_ASSERT( j < matrix_.columns(), "Invalid column access index" );

      if( IsFloatingPoint<ElementType>::value )
         return matrix_(i,j) * scalar_;
      else
         return matrix_(i,j) / scalar_;
   }
   //**********************************************************************************************

   //**Rows function*******************************************************************************
   /*!\brief Returns the current number of rows of the matrix.
   //
   // \return The number of rows of the matrix.
   */
   inline size_t rows() const {
      return matrix_.rows();
   }
   //**********************************************************************************************

   //**Columns function****************************************************************************
   /*!\brief Returns the current number of columns of the matrix.
   //
   // \return The number of columns of the matrix.
   */
   inline size_t columns() const {
      return matrix_.columns();
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
      return ( IsExpression<MT>::value && matrix_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs matrix_;  //!< Left-hand side dense matrix of the division expression.
   Rhs scalar_;  //!< Right-hand side scalar of the division expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT );
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE     ( ST );
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
/*!\brief Division operator for the division of a dense matrix by a scalar value
 *        (\f$ A=B*s \f$).
 * \ingroup dense_matrix
 *
 * \param mat The left-hand side dense matrix for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result matrix.
 *
 * This operator represents the division of a dense matrix by a scalar value:

   \code
   pe::MatN A, B;
   // ... Resizing and initialization
   B = A / 0.24;
   \endcode

 * The operator returns an expression representing a dense matrix of the higher-order element
 * type of the involved data types \a T1::ElementType and \a T2. Note that this operator only
 * works for scalar values of built-in data type.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side scalar
inline const typename EnableIf< IsNumeric<T2>, DMatScalarDivExpr<T1,T2> >::Type
   operator/( const DenseMatrix<T1>& mat, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   return DMatScalarDivExpr<T1,T2>( ~mat, scalar );
}
//*************************************************************************************************

} // namespace pe

#endif
