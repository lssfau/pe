//=================================================================================================
/*!
 *  \file pe/math/expressions/SVecScalarMultExpr.h
 *  \brief Header file for the sparse vector/scalar multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SVECSCALARMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SVECSCALARMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/EnableIf.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  CLASS SVECSCALARMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for sparse vector-scalar multiplications.
 * \ingroup sparse_vector_expression
 *
 * The SVecScalarMultExpr class represents the compile time expression for multiplications between
 * a sparse vector and a scalar value.
 */
template< typename VT  // Type of the left-hand side sparse vector
        , typename ST  // Type of the right-hand side scalar value
        , bool TF >    // Transposition flag
class SVecScalarMultExpr : public SparseVector< SVecScalarMultExpr<VT,ST,TF>, TF >
                         , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT::ResultType     RT;  //!< Result type of the sparse vector expression.
   typedef typename VT::TransposeType  TT;  //!< Transpose type of the sparse vector expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef SVecScalarMultExpr<VT,ST,TF>         This;           //!< Type of this SVecScalarMultExpr instance.
   typedef typename MathTrait<RT,ST>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::ElementType     ElementType;    //!< Resulting element type.
   typedef const SVecScalarMultExpr&            CompositeType;  //!< Data type for composite expression templates.
   typedef SVecScalarMultExpr<TT,ST,!TF>        TransposeType;  //!< Transpose type for expression template evaluations.

   //! Composite type of the left-hand side sparse vector expression.
   typedef typename VT::CompositeType  Lhs;

   //! Composite type of the right-hand side scalar value.
   typedef typename SelectType<IsNumeric<ElementType>::value,ElementType,ST>::Type  Rhs;
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
         return vector_->value() * scalar_;
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
   /*!\brief Constructor for the SVecScalarMultExpr class.
   */
   explicit inline SVecScalarMultExpr( const VT& vector, ST scalar )
      : vector_( vector )  // Left-hand side sparse vector of the multiplication expression
      , scalar_( scalar )  // Right-hand side scalar of the multiplication expression
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
      return vector_.isAliased( alias );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs vector_;  //!< Left-hand side sparse vector of the multiplication expression.
   Rhs scalar_;  //!< Right-hand side scalar of the multiplication expression.
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
/*!\brief Multiplication operator for the multiplication of a sparse vector and a scalar value
 *        (\f$ \vec{a}=\vec{b}*s \f$).
 * \ingroup sparse_vector
 *
 * \param vec The left-hand side sparse vector for the multiplication.
 * \param scalar The right-hand side scalar value for the multiplication.
 * \return The scaled result vector.
 *
 * This operator represents the multiplication between a sparse vector and a scalar value:

   \code
   pe::SVecN a, b;
   // ... Resizing and initialization
   b = a * 1.25;
   \endcode

 * The operator returns a sparse vector of the higher-order element type of the involved data
 * types \a T1::ElementType and \a T2. Note that this operator only works for scalar values
 * of built-in data type.
 */
template< typename T1  // Type of the left-hand side sparse vector
        , typename T2  // Type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline const typename EnableIf< IsNumeric<T2>, SVecScalarMultExpr<T1,T2,TF> >::Type
   operator*( const SparseVector<T1,TF>& vec, T2 scalar )
{
   return SVecScalarMultExpr<T1,T2,TF>( ~vec, scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a scalar value and a sparse vector
 *        (\f$ \vec{a}=s*\vec{b} \f$).
 * \ingroup sparse_vector
 *
 * \param scalar The left-hand side scalar value for the multiplication.
 * \param vec The right-hand side sparse vector for the multiplication.
 * \return The scaled result vector.
 *
 * This operator represents the multiplication between a a scalar value and sparse vector:

   \code
   pe::VecN a, b;
   // ... Resizing and initialization
   b = 1.25 * a;
   \endcode

 * The operator returns a sparse vector of the higher-order element type of the involved data
 * types \a T1 and \a T2::ElementType. Note that this operator only works for scalar values
 * of built-in data type.
 */
template< typename T1  // Type of the left-hand side scalar
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline const typename EnableIf< IsNumeric<T1>, SVecScalarMultExpr<T2,T1,TF> >::Type
   operator*( T1 scalar, const SparseVector<T2,TF>& vec )
{
   return SVecScalarMultExpr<T2,T1,TF>( ~vec, scalar );
}
//*************************************************************************************************

} // namespace pe

#endif
