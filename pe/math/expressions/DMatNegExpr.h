//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatNegExpr.h
 *  \brief Header file for the dense matrix negation expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATNEGEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATNEGEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DMATNEGEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense matrix negations.
 * \ingroup dense_matrix_expression
 *
 * The DMatNegExpr class represents the compile time expression for negations of dense matrices.
 */
template< typename MT >  // Type of the dense matrix
class DMatNegExpr : public DenseMatrix< DMatNegExpr<MT> >
                  , private Expression
{
public:
   //**Type definitions****************************************************************************
   typedef MT                        ResultType;     //!< Result type for expression template evaluations.
   typedef const DMatNegExpr&        CompositeType;  //!< Data type for composite expression templates.
   typedef typename MT::ElementType  ElementType;    //!< Resulting element type.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatNegExpr class.
   */
   explicit inline DMatNegExpr( const MT& dm )
      : dm_( dm )
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
      pe_INTERNAL_ASSERT( i < dm_.rows()   , "Invalid row access index"    );
      pe_INTERNAL_ASSERT( j < dm_.columns(), "Invalid column access index" );
      return -dm_(i,j);
   }
   //**********************************************************************************************

   //**Rows function*******************************************************************************
   /*!\brief Returns the current number of rows of the matrix.
   //
   // \return The number of rows of the matrix.
   */
   inline size_t rows() const {
      return dm_.rows();
   }
   //**********************************************************************************************

   //**Columns function****************************************************************************
   /*!\brief Returns the current number of columns of the matrix.
   //
   // \return The number of columns of the matrix.
   */
   inline size_t columns() const {
      return dm_.columns();
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
      return ( IsExpression<MT>::value && dm_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   const MT& dm_;  //!< Dense matrix of the negation expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL UNARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Unary minus operator for the negation of a dense matrix (\f$ A = -B \f$).
 * \ingroup dense_matrix
 *
 * \param dm The dense matrix to be negated.
 * \return The negation of the matrix.
 *
 * This operator represents the negation of a dense matrix:

   \code
   pe::MatN A, B;
   // ... Resizing and initialization
   B = -A;
   \endcode

 * The operator returns an expression representing the negation of the given dense matrix.
 */
template< typename MT >  // Type of the dense matrix
inline const DMatNegExpr<MT> operator-( const DenseMatrix<MT>& dm )
{
   return DMatNegExpr<MT>( ~dm );
}
//*************************************************************************************************

} // namespace pe

#endif
