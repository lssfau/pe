//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecDVecMultExpr.h
 *  \brief Header file for the dense vector/dense vector multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECDVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECDVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Reference.h>
#include <pe/util/EnableIf.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsReference.h>


namespace pe {

//=================================================================================================
//
//  CLASS DVECDVECMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense vector-dense vector multiplications.
 * \ingroup dense_vector_expression
 *
 * The DVecDVecMultExpr class represents the compile time expression for componentwise
 * multiplications between dense vectors.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF >     // Transposition flag
class DVecDVecMultExpr : public DenseVector< DVecDVecMultExpr<VT1,VT2,TF>, TF >
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

   //**********************************************************************************************
   //! Compilation switch for the evaluation strategy of the multiplication expression.
   /*! The \a useAssign compile time constant expression represents a compilation switch for
       the evaluation strategy of the multiplication expression. In case either of the two
       dense vector operands requires an intermediate evaluation, \a useAssign will be set to
       \a true and the multiplication expression will be evaluated via the \a assign function
       family. Otherwise \a useAssign will be set to \a false and the expression will be
       evaluated via the subscript operator. */
   enum { useAssign = ( !IsReference<CT1>::value || !IsReference<CT2>::value ) };
   //**********************************************************************************************

   //**********************************************************************************************
   //! Helper structure for the explicit application of the SFINAE principal.
   template< typename VT >
   struct UseAssign {
      enum { value = useAssign };
   };
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DVecDVecMultExpr<VT1,VT2,TF>           This;         //!< Type of this DVecDVecMultExpr instance.
   typedef typename MathTrait<RT1,RT2>::MultType  ResultType;   //!< Result type for expression template evaluations.
   typedef typename ResultType::ElementType       ElementType;  //!< Resulting element type.

   //! Data type for composite expression templates.
   typedef typename SelectType< useAssign, const ResultType, const DVecDVecMultExpr& >::Type  CompositeType;

   //! Transpose type for expression template evaluations.
   typedef DVecDVecMultExpr<TT1,TT2,!TF>  TransposeType;

   //! Composite type of the left-hand side dense vector expression.
   typedef typename SelectType< IsReference<CT1>::value, CT1, const VT1& >::Type  Lhs;

   //! Composite type of the right-hand side dense vector expression.
   typedef typename SelectType< IsReference<CT2>::value, CT2, const VT2& >::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DVecDVecMultExpr class.
   */
   explicit inline DVecDVecMultExpr( const VT1& lhs, const VT2& rhs )
      : lhs_( lhs )  // Left-hand side dense vector of the multiplication expression
      , rhs_( rhs )  // Right-hand side dense vector of the multiplication expression
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
      return lhs_[index] * rhs_[index];
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
   Lhs lhs_;  //!< Left-hand side dense vector of the multiplication expression.
   Rhs rhs_;  //!< Right-hand side dense vector of the multiplication expression.
   //**********************************************************************************************

   //**Assignment to dense vectors*****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-dense vector multiplication to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-dense
   // vector multiplication expression to a dense vector. Due to the explicit application of
   // the SFINAE principle, this operator can only be selected by the compiler in case either
   // of the two operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      assign( DenseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      assign    ( ~lhs, rhs.lhs_ );
      multAssign( ~lhs, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse vectors****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-dense vector multiplication to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-dense
   // vector multiplication expression to a sparse vector. Due to the explicit application of
   // the SFINAE principle, this operator can only be selected by the compiler in case either
   // of the two operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      assign( SparseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, TF );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      assign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to dense vectors********************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-dense vector multiplication to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // dense vector multiplication expression to a dense vector. Due to the explicit application
   // of the SFINAE principle, this operator can only be selected by the compiler in case either
   // of the operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      addAssign( DenseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      CT1 tmp1( rhs.lhs_ );
      CT2 tmp2( rhs.rhs_ );
      addAssign( ~lhs, tmp1*tmp2 );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to sparse vectors*******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-dense vector multiplication to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // dense vector multiplication expression to a sparse vector. Due to the explicit application
   // of the SFINAE principle, this operator can only be selected by the compiler in case either
   // of the operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      addAssign( SparseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, TF );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      addAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to dense vectors*****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense vector-dense vector multiplication to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // dense vector multiplication expression to a dense vector. Due to the explicit application of
   // the SFINAE principle, this operator can only be selected by the compiler in case either of
   // the operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      subAssign( DenseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      CT1 tmp1( rhs.lhs_ );
      CT2 tmp2( rhs.rhs_ );
      subAssign( ~lhs, tmp1*tmp2 );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to sparse vectors****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense vector-dense vector multiplication to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // dense vector multiplication expression to a sparse vector. Due to the explicit application of
   // the SFINAE principle, this operator can only be selected by the compiler in case either of
   // the operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      subAssign( SparseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, TF );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      subAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to dense vectors**************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a dense vector-dense vector multiplication to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-dense vector multiplication expression to a dense vector. Due to the explicit
   // application of the SFINAE principle, this operator can only be selected by the compiler
   // in case either of the operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      multAssign( DenseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      multAssign( ~lhs, rhs.lhs_ );
      multAssign( ~lhs, rhs.rhs_ );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to sparse vectors*************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a dense vector-dense vector multiplication to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-dense vector multiplication expression to a sparse vector. Due to the explicit
   // application of the SFINAE principle, this operator can only be selected by the compiler
   // in case either of the operands requires an intermediate evaluation.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT> >::Type
      multAssign( SparseVector<VT,TF>& lhs, const DVecDVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      multAssign( ~lhs, rhs.lhs_ );
      multAssign( ~lhs, rhs.rhs_ );
   }
   /*! \endcond */
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
/*!\brief Multiplication operator for the componentwise product of two dense vectors
 *        (\f$ \vec{a}=\vec{b}*\vec{c} \f$).
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side dense vector for the component product.
 * \param rhs The right-hand side dense vector for the component product.
 * \return The product of the two vectors.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the component product of two dense vectors:

   \code
   pe::VecN a, b, c;
   // ... Resizing and initialization
   c = a * b;
   \endcode

 * The operator returns an expression representing a dense vector of the higher-order element
 * type of the two involved vector element types \a T1::ElementType and \a T2::ElementType.
 * Both vector types \a T1 and \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline const DVecDVecMultExpr<T1,T2,TF>
   operator*( const DenseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs )
{
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   return DVecDVecMultExpr<T1,T2,TF>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
