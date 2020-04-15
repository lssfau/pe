//=================================================================================================
/*!
 *  \file pe/math/expressions/SVecScalarDivExpr.h
 *  \brief Header file for the sparse vector/scalar division expression
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

#ifndef _PE_MATH_EXPRESSIONS_SVECSCALARDIVEXPR_H_
#define _PE_MATH_EXPRESSIONS_SVECSCALARDIVEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/SparseVector.h>
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
//  CLASS SVECSCALARDIVEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for divisions of a sparse vector by a scalar.
 * \ingroup sparse_vector_expression
 *
 * The SVecScalarDivExpr class represents the compile time expression for divisions of sparse
 * vectors by scalar values.
 */
template< typename VT  // Type of the left-hand side sparse vector
        , typename ST  // Type of the right-hand side scalar value
        , bool TF >    // Transposition flag
class SVecScalarDivExpr : public SparseVector< SVecScalarDivExpr<VT,ST,TF>, TF >
                        , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT::ResultType     RT;  //!< Result type of the sparse vector expression.
   typedef typename VT::TransposeType  TT;  //!< Transpose type of the sparse vector expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef SVecScalarDivExpr<VT,ST,TF>         This;           //!< Type of this SVecScalarDivExpr instance.
   typedef typename MathTrait<RT,ST>::DivType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::ElementType    ElementType;    //!< Resulting element type.
   typedef const SVecScalarDivExpr&            CompositeType;  //!< Data type for composite expression templates.
   typedef SVecScalarDivExpr<TT,ST,!TF>        TransposeType;  //!< Transpose type for expression template evaluations.

   //! Composite type of the left-hand side sparse vector expression.
   typedef typename VT::CompositeType  Lhs;

   //! Composite type of the right-hand side scalar value.
   typedef typename SelectType< IsNumeric<ElementType>::value, ElementType, ST >::Type  Rhs;
   //**********************************************************************************************

   //**ConstIterator class definition**************************************************************
   /*!\brief Iterator over the elements of the sparse vector/scalar multiplication expression.
   */
   class ConstIterator
   {
    public:
      //**Type definitions*************************************************************************
      //! Iterator type of the left-hand side sparse vector expression.
      typedef typename boost::remove_reference<Lhs>::type::ConstIterator  IteratorType;
      //*******************************************************************************************

      //**Constructor******************************************************************************
      /*!\brief Constructor for the ConstIterator class.
      */
      inline ConstIterator( IteratorType vector, Rhs scalar )
         : vector_( vector )  // Iterator over the elements of the left-hand side sparse vector expression
         , scalar_( scalar )  // Right hand side scalar of the multiplication expression
      {}
      //*******************************************************************************************

      //**Prefix increment operator****************************************************************
      /*!\brief Pre-increment operator.
      //
      // \return Reference to the incremented expression iterator.
      */
      inline ConstIterator& operator++() {
         ++vector_;
         return *this;
      }
      //*******************************************************************************************

      //**Element access operator******************************************************************
      /*!\brief Direct access to the sparse vector element at the current iterator position.
      //
      // \return Reference to the sparse vector element at the current iterator position.
      */
      inline const ConstIterator* operator->() const {
         return this;
      }
      //*******************************************************************************************

      //**Value function***************************************************************************
      /*!\brief Access to the current value of the sparse element.
      //
      // \return The current value of the sparse element.
      */
      inline ElementType value() const {
         if( IsNumeric<ElementType>::value && IsFloatingPoint<ElementType>::value )
            return vector_->value() * scalar_;
         else
            return vector_->value() / scalar_;
      }
      //*******************************************************************************************

      //**Index function***************************************************************************
      /*!\brief Access to the current index of the sparse element.
      //
      // \return The current index of the sparse element.
      */
      inline size_t index() const {
         return vector_->index();
      }
      //*******************************************************************************************

      //**Equality operator************************************************************************
      /*!\brief Equality comparison between two ConstIterator objects.
      //
      // \param rhs The right-hand side expression iterator.
      // \return \a true if the iterators refer to the same element, \a false if not.
      */
      inline bool operator==( const ConstIterator& rhs ) const {
         return vector_ == rhs.vector_;
      }
      //*******************************************************************************************

      //**Inequality operator**********************************************************************
      /*!\brief Inequality comparison between two ConstIterator objects.
      //
      // \param rhs The right-hand side expression iterator.
      // \return \a true if the iterators don't refer to the same element, \a false if they do.
      */
      inline bool operator!=( const ConstIterator& rhs ) const {
         return vector_ != rhs.vector_;
      }
      //*******************************************************************************************

    private:
      //**Member variables*************************************************************************
      IteratorType vector_;  //!< Iterator over the elements of the left-hand side sparse vector expression.
      Rhs scalar_;           //!< Right hand side scalar of the multiplication expression.
      //*******************************************************************************************
   };
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the SVecScalarDivExpr class.
   */
   explicit inline SVecScalarDivExpr( const VT& vector, ST scalar )
      : vector_( vector )
      , scalar_( ( IsNumeric<ElementType>::value && IsFloatingPoint<ElementType>::value ) ?
                 ( Rhs(1)/scalar ) : ( scalar ) )
   {}
   //**********************************************************************************************

   //**Begin function******************************************************************************
   /*!\brief Returns an iterator to the first non-zero element of the sparse vector.
   //
   // \return Iterator to the first non-zero element of the sparse vector.
   */
   inline ConstIterator begin() const {
      return ConstIterator( vector_.begin(), scalar_ );
   }
   //**********************************************************************************************

   //**End function********************************************************************************
   /*!\brief Returns an iterator just past the last non-zero element of the sparse vector.
   //
   // \return Iterator just past the last non-zero element of the sparse vector.
   */
   inline ConstIterator end() const {
      return ConstIterator( vector_.end(), scalar_ );
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

   //**NonZeros function***************************************************************************
   /*!\brief Returns the number of non-zero elements in the sparse vector.
   //
   // \return The number of non-zero elements in the sparse vector.
   */
   inline size_t nonZeros() const {
      return vector_.nonZeros();
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
   Lhs vector_;  //!< Left-hand side sparse vector of the division expression.
   Rhs scalar_;  //!< Right-hand side scalar of the division expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( VT );
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
/*!\brief Division operator for the divison of a sparse vector by a scalar value
 *        (\f$ \vec{a}=\vec{b}/s \f$).
 * \ingroup sparse_vector
 *
 * \param vec The left-hand side sparse vector for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result vector.
 *
 * This operator represents the division of a sparse vector by a scalar value:

   \code
   pe::SVecN a, b;
   // ... Resizing and initialization
   b = a / 0.24;
   \endcode

 * The operator returns a sparse vector of the higher-order element type of the involved data
 * types \a T1::ElementType and \a T2. Note that this operator only works for scalar values
 * of built-in data type.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename T1, typename T2, bool TF >
inline const typename EnableIf< IsNumeric<T2>, SVecScalarDivExpr<T1,T2,TF> >::Type
   operator/( const SparseVector<T1,TF>& vec, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   return SVecScalarDivExpr<T1,T2,TF>( ~vec, scalar );
}
//*************************************************************************************************

} // namespace pe

#endif
