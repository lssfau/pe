//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecTDVecMultExpr.h
 *  \brief Header file for the dense vector/dense vector outer product expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECTDVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECTDVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DVECTDVECMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for outer products between two dense vectors.
 * \ingroup dense_vector_expression
 *
 * The DVecTDVecMultExpr class represents the compile time expression for outer products
 * between dense vectors.
 */
template< typename VT1    // Type of the left-hand side dense vector
        , typename VT2 >  // Type of the right-hand side dense vector
class DVecTDVecMultExpr : public DenseMatrix< DVecTDVecMultExpr<VT1,VT2> >
                        , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT1::ResultType  RT1;  //!< Result type of the left-hand side dense vector expression.
   typedef typename VT2::ResultType  RT2;  //!< Result type of the right-hand side dense vector expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename MathTrait<RT1,RT2>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef const DVecTDVecMultExpr&               CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense vector expression.
   typedef typename SelectType<IsExpression<VT1>::value,const RT1,const VT1&>::Type  Lhs;

   //! Composite type of the right-hand side dense vector expression.
   typedef typename SelectType<IsExpression<VT2>::value,const RT2,const VT2&>::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DVecTDVecMultExpr class.
   */
   explicit inline DVecTDVecMultExpr( const VT1& lhs, const VT2& rhs )
      : lhs_( lhs )  // Left-hand side dense vector of the multiplication expression
      , rhs_( rhs )  // Right-hand side dense vector of the multiplication expression
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
      pe_INTERNAL_ASSERT( i < lhs_.size(), "Invalid row access index"    );
      pe_INTERNAL_ASSERT( j < rhs_.size(), "Invalid column access index" );

      return lhs_[i] * rhs_[j];
   }
   //**********************************************************************************************

   //**Rows function*******************************************************************************
   /*!\brief Returns the current number of rows of the matrix.
   //
   // \return The number of rows of the matrix.
   */
   inline size_t rows() const {
      return lhs_.size();
   }
   //**********************************************************************************************

   //**Columns function****************************************************************************
   /*!\brief Returns the current number of columns of the matrix.
   //
   // \return The number of columns of the matrix.
   */
   inline size_t columns() const {
      return rhs_.size();
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
      return ( !IsExpression<VT1>::value && lhs_.isAliased( alias ) ) ||
             ( !IsExpression<VT2>::value && rhs_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs lhs_;  //!< Left-hand side dense vector of the multiplication expression.
   Rhs rhs_;  //!< Right-hand side dense vector of the multiplication expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT1 );
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT2 );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT1, false );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT2, true  );
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
/*!\brief Multiplication operator for the outer product of two dense vectors
 *        (\f$ A=\vec{b}*\vec{c}^T \f$).
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side dense vector for the outer product.
 * \param rhs The right-hand side transpose dense vector for the outer product.
 * \return The resulting matrix.
 *
 * This operator represents the outer product between a dense vector and a transpose dense
 * vector:

   \code
   pe::VecN a, b;
   pe::MatN A;
   // ... Resizing and initialization
   A = a * trans(b);
   \endcode

 * The operator returns an expression representing a dense matrix of the higher-order element
 * type of the two involved element types \a T1::ElementType and \a T2::ElementType. Both
 * dense vector types \a T1 and \a T2 as well as the two element types \a T1::ElementType
 * and \a T2::ElementType have to be supported by the MathTrait class template.
 */
template< typename T1    // Type of the left-hand side dense vector
        , typename T2 >  // Type of the right-hand side dense vector
inline const DVecTDVecMultExpr<T1,T2>
   operator*( const DenseVector<T1,false>& lhs, const DenseVector<T2,true>& rhs )
{
   return DVecTDVecMultExpr<T1,T2>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
