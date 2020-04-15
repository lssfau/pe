//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecScalarDivExpr.h
 *  \brief Header file for the dense vector/scalar division expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECSCALARDIVEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECSCALARDIVEXPR_H_


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
#include <pe/util/constraints/Numeric.h>
#include <pe/util/constraints/Reference.h>
#include <pe/util/EnableIf.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsFloatingPoint.h>
#include <pe/util/typetraits/IsNumeric.h>
#include <pe/util/typetraits/IsReference.h>


namespace pe {

//=================================================================================================
//
//  CLASS DVECSCALARDIVEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for divisions of a dense vector by a scalar.
 * \ingroup dense_vector_expression
 *
 * The DVecScalarDivExpr class represents the compile time expression for divisions of dense
 * vectors by scalar values.
 */
template< typename VT  // Type of the left-hand side dense vector
        , typename ST  // Type of the right-hand side scalar value
        , bool TF >    // Transposition flag
class DVecScalarDivExpr : public DenseVector< DVecScalarDivExpr<VT,ST,TF>, TF >
                        , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT::ResultType     RT;  //!< Result type of the dense vector expression.
   typedef typename VT::CompositeType  CT;  //!< Composite type of the dense vector expression.
   typedef typename VT::TransposeType  TT;  //!< Transpose type of the dense vector expression.
   //**********************************************************************************************

   //**********************************************************************************************
   //! Compilation switch for the evaluation strategy of the multiplication expression.
   /*! The \a useAssign compile time constant expression represents a compilation switch for
       the evaluation strategy of the multiplication expression. In case either the dense vector
       operand requires an intermediate evaluation, \a useAssign will be set to \a true and the
       multiplication expression will be evaluated via the \a assign function family. Otherwise
       \a useAssign will be set to \a false and the expression will be evaluated via the subscript
       operator. */
   enum { useAssign = !IsReference<CT>::value };
   //**********************************************************************************************

   //**********************************************************************************************
   //! Helper structure for the explicit application of the SFINAE principal.
   template< typename VT2 >
   struct UseAssign {
      enum { value = useAssign };
   };
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DVecScalarDivExpr<VT,ST,TF>         This;         //!< Type of this DVecScalarDivExpr instance.
   typedef typename MathTrait<RT,ST>::DivType  ResultType;   //!< Result type for expression template evaluations.
   typedef typename ResultType::ElementType    ElementType;  //!< Resulting element type.

   //! Data type for composite expression templates.
   typedef typename SelectType< useAssign, const ResultType, const DVecScalarDivExpr& >::Type  CompositeType;

   //! Transpose type for expression template evaluations.
   typedef DVecScalarDivExpr<TT,ST,!TF>  TransposeType;

   //! Composite type of the left-hand side dense vector expression.
   typedef typename SelectType< IsReference<CT>::value, CT, const VT& >::Type  Lhs;

   //! Composite type of the right-hand side scalar value.
   typedef typename SelectType< IsNumeric<ElementType>::value, ElementType, ST >::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DVecScalarDivExpr class.
   */
   explicit inline DVecScalarDivExpr( const VT& vector, ST scalar )
      : vector_( vector )
      , scalar_( ( IsNumeric<ElementType>::value && IsFloatingPoint<ElementType>::value ) ?
                 ( Rhs(1)/scalar ) : ( scalar ) )
   {}
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline const ElementType operator[]( size_t index ) const {
      pe_INTERNAL_ASSERT( index < vector_.size(), "Invalid vector access index" );
      if( IsNumeric<ElementType>::value && IsFloatingPoint<ElementType>::value )
         return vector_[index] * scalar_;
      else
         return vector_[index] / scalar_;
   }
   //**********************************************************************************************

   //**Size function*******************************************************************************
   /*!\brief Returns the current size/dimension of the vector.
   //
   // \return The size of the vector.
   */
   inline size_t size() const {
      return vector_.size();
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
      return ( IsExpression<VT>::value && vector_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs vector_;  //!< Left-hand side dense vector of the division expression.
   Rhs scalar_;  //!< Right-hand side scalar of the division expression.
   //**********************************************************************************************

   //**Assignment to dense vectors*****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-scalar division to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side division expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-scalar
   // division expression to a dense vector. Due to the explicit application of the SFINAE
   // principle, this operator can only be selected by the compiler in case the vector
   // operand requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      assign( DenseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      assign( ~lhs, rhs.vector_ );

      const size_t size( rhs.size() );
      if( IsNumeric<ElementType>::value && IsFloatingPoint<ElementType>::value ) {
         for( size_t i=0; i<size; ++i )
            (~lhs)[i] *= rhs.scalar_;
      }
      else {
         for( size_t i=0; i<size; ++i )
            (~lhs)[i] /= rhs.scalar_;
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse vectors****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-scalar division to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side division expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-scalar
   // division expression to a sparse vector. Due to the explicit application of the SFINAE
   // principle, this operator can only be selected by the compiler in case the vector
   // operand requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      assign( SparseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      assign( ~lhs, rhs.vector_ );

      typename VT2::Iterator begin( (~lhs).begin() );
      const typename VT2::Iterator end( (~lhs).end() );

      if( IsNumeric<ElementType>::value && IsFloatingPoint<ElementType>::value ) {
         for( ; begin!=end; ++begin )
            begin->value() *= rhs.scalar_;
      }
      else {
         for( ; begin!=end; ++begin )
            begin->value() /= rhs.scalar_;
      }
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to dense vectors********************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-scalar division to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side division expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // scalar division expression to a dense vector. Due to the explicit application of the
   // SFINAE principle, this operator can only be selected by the compiler in case the vector
   // operand requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      addAssign( DenseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
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

   //**Addition assignment to sparse vectors*******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-scalar division to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side division expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // scalar division expression to a sparse vector. Due to the explicit application of the
   // SFINAE principle, this operator can only be selected by the compiler in case the vector
   // operand requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      addAssign( SparseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
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
   /*!\brief Subtraction assignment of a dense vector-scalar division to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side division expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // scalar division expression to a dense vector. Due to the explicit application of the SFINAE
   // principle, this operator can only be selected by the compiler in case the vector operand
   // requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      subAssign( DenseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
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

   //**Subtraction assignment to sparse vectors****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense vector-scalar division to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side division expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // scalar division expression to a sparse vector. Due to the explicit application of the SFINAE
   // principle, this operator can only be selected by the compiler in case the vector operand
   // requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      subAssign( SparseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
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
   /*!\brief Multiplication assignment of a dense vector-scalar division to a dense vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side division expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-scalar division expression to a dense vector. Due to the explicit application
   // of the SFINAE principle, this operator can only be selected by the compiler in case the
   // vector operand requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target dense vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      multAssign( DenseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, TF );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      multAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to sparse vectors*************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a dense vector-scalar division to a sparse vector.
   // \ingroup dense_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side division expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-scalar division expression to a sparse vector. Due to the explicit application
   // of the SFINAE principle, this operator can only be selected by the compiler in case the
   // vector operand requires an intermediate evaluation.
   */
   template< typename VT2 >  // Type of the target sparse vector
   friend inline typename EnableIf< UseAssign<VT2> >::Type
      multAssign( SparseVector<VT2,TF>& lhs, const DVecScalarDivExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( ResultType );
      pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( ResultType, TF );
      pe_CONSTRAINT_MUST_BE_REFERENCE_TYPE( typename ResultType::CompositeType );

      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      const ResultType tmp( rhs );
      multAssign( ~lhs, tmp );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE( VT );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT, TF );
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( ST );
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
/*!\brief Division operator for the divison of a dense vector by a scalar value
 *        (\f$ \vec{a}=\vec{b}/s \f$).
 * \ingroup dense_vector
 *
 * \param vec The left-hand side dense vector for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result vector.
 *
 * This operator represents the division of a dense vector by a scalar value:

   \code
   pe::VecN a, b;
   // ... Resizing and initialization
   b = a / 0.24;
   \endcode

 * The operator returns an expression representing a dense vector of the higher-order
 * element type of the involved data types \a T1::ElementType and \a T2. Both data types
 * \a T1::ElementType and \a T2 have to be supported by the MathTrait class template.
 * Note that this operator only works for scalar values of built-in data type.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline const typename EnableIf< IsNumeric<T2>, DVecScalarDivExpr<T1,T2,TF> >::Type
   operator/( const DenseVector<T1,TF>& vec, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   return DVecScalarDivExpr<T1,T2,TF>( ~vec, scalar );
}
//*************************************************************************************************

} // namespace pe

#endif
