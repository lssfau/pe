//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecDVecDivExpr.h
 *  \brief Header file for the dense vector/dense vector division expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECDVECDIVEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECDVECDIVEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DVECDVECDIVEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense vector-dense vector divisions.
 * \ingroup dense_vector_expression
 *
 * The DVecDVecDivExpr class represents the compile time expression for componentwise
 * divisions between dense vectors.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF >     // Transposition flag
class DVecDVecDivExpr : public DenseVector< DVecDVecDivExpr<VT1,VT2,TF>, TF >
                      , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT1::ResultType     RT1;  //!< Result type of the left-hand side dense vector expression.
   typedef typename VT2::ResultType     RT2;  //!< Result type of the right-hand side dense vector expression.
   typedef typename VT1::CompositeType  CT1;  //!< Composite type of the left-hand side dense vector expression.
   typedef typename VT2::CompositeType  CT2;  //!< Composite type of the right-hand side denese vector expression.
   typedef typename VT1::TransposeType  TT1;  //!< Transpose type of the left-hand side dense vector expression.
   typedef typename VT2::TransposeType  TT2;  //!< Transpose type of the right-hand side dense vector expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DVecDVecDivExpr<VT1,VT2,TF>           This;           //!< Type of this DVecDVecDivExpr instance.
   typedef typename MathTrait<RT1,RT2>::DivType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::ElementType      ElementType;    //!< Resulting element type.
   typedef const DVecDVecDivExpr&                CompositeType;  //!< Data type for composite expression templates.
   typedef DVecDVecDivExpr<TT1,TT2,!TF>          TransposeType;  //!< Transpose type for expression template evaluations.

   //! Composite type of the left-hand side dense vector expression.
   typedef typename VT1::CompositeType  Lhs;

   //! Composite type of the right-hand side dense vector expression.
   typedef typename VT2::CompositeType  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DVecDVecDivExpr class.
   */
   explicit inline DVecDVecDivExpr( const VT1& lhs, const VT2& rhs )
      : lhs_( lhs )  // Left-hand side dense vector of the division expression
      , rhs_( rhs )  // Right-hand side dense vector of the division expression
   {
      pe_INTERNAL_ASSERT( lhs.size() == rhs.size(), "Invalid vector sizes" );
   }
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline const ElementType operator[]( size_t index ) const {
      pe_INTERNAL_ASSERT( index < lhs_.size(), "Invalid vector access index" );
      return lhs_[index] / rhs_[index];
   }
   //**********************************************************************************************

   //**Size function*******************************************************************************
   /*!\brief Returns the current size/dimension of the vector.
   //
   // \return The size of the vector.
   */
   inline size_t size() const {
      return lhs_.size();
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
      return ( IsExpression<VT1>::value && lhs_.isAliased( alias ) ) ||
             ( IsExpression<VT2>::value && rhs_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs lhs_;  //!< Left-hand side dense vector of the division expression.
   Rhs rhs_;  //!< Right-hand side dense vector of the division expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT1 );
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT2 );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT1, TF );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT2, TF );
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
/*!\brief Division operator for the componentwise division of two dense vectors
 *        (\f$ \vec{a}=\vec{b}/\vec{c} \f$).
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side dense vector for the vector division.
 * \param rhs The right-hand side dense vector for the vector division.
 * \return The division of the two vectors.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the componentwise division of two dense vectors:

   \code
   pe::VecN a, b, c;
   // ... Resizing and initialization
   c = a / b;
   \endcode

 * The operator returns an expression representing a dense vector of the higher-order element
 * type of the two involved vector element types \a T1::ElementType and \a T2::ElementType.
 * Both vector types \a T1 and \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown. Please note that no checks for invalid division (such as for instance divisions
 * by zero) are applied!
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline const DVecDVecDivExpr<T1,T2,TF>
   operator/( const DenseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs )
{
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   return DVecDVecDivExpr<T1,T2,TF>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
