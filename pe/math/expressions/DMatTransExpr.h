//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatTransExpr.h
 *  \brief Header file for the dense matrix transpose expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATTRANSEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATTRANSEXPR_H_


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
//  CLASS DMATTRANSEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense matrix transpositions.
 * \ingroup dense_matrix_expression
 *
 * The DMatTranspExpr class represents the compile time expression for transpositions of
 * dense matrices.
 */
template< typename MT >  // Type of the dense matrix
class DMatTransExpr : public DenseMatrix< DMatTransExpr<MT> >
                    , private Expression
{
public:
   //**Type definitions****************************************************************************
   typedef MT                        ResultType;     //!< Result type for expression template evaluations.
   typedef const DMatTransExpr&      CompositeType;  //!< Data type for composite expression templates.
   typedef typename MT::ElementType  ElementType;    //!< Resulting element type.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatTransExpr class.
   */
   explicit inline DMatTransExpr( const MT& dm )
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
      pe_INTERNAL_ASSERT( i < dm_.columns(), "Invalid row access index"    );
      pe_INTERNAL_ASSERT( j < dm_.rows()   , "Invalid column access index" );
      return dm_(j,i);
   }
   //**********************************************************************************************

   //**Rows function*******************************************************************************
   /*!\brief Returns the current number of rows of the matrix.
   //
   // \return The number of rows of the matrix.
   */
   inline size_t rows() const {
      return dm_.columns();
   }
   //**********************************************************************************************

   //**Columns function****************************************************************************
   /*!\brief Returns the current number of columns of the matrix.
   //
   // \return The number of columns of the matrix.
   */
   inline size_t columns() const {
      return dm_.rows();
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
   const MT& dm_;  //!< Dense matrix of the transposition expression.
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
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the transpose of the given dense matrix.
 * \ingroup dense_matrix
 *
 * \param dm The dense matrix to be transposed.
 * \return The transpose of the matrix.
 *
 * This function returns an expression representing the transpose of the given dense matrix:

   \code
   pe::MatN A, B;
   // ... Resizing and initialization
   B = trans( A );
   \endcode
 */
template< typename MT >  // Type of the dense matrix
inline const DMatTransExpr<MT> trans( const DenseMatrix<MT>& dm )
{
   return DMatTransExpr<MT>( ~dm );
}
//*************************************************************************************************

} // namespace pe

#endif
