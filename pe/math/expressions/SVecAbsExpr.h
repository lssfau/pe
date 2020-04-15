//=================================================================================================
/*!
 *  \file pe/math/expressions/SVecAbsExpr.h
 *  \brief Header file for the sparse vector absolute value expression
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

#ifndef _PE_MATH_EXPRESSIONS_SVECABSEXPR_H_
#define _PE_MATH_EXPRESSIONS_SVECABSEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS SVECABSEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for the sparse vector abs() function.
 * \ingroup sparse_vector_expression
 *
 * The SVecAbsExpr class represents the compile time expression for the calculation of the
 * absolute value of each element of a sparse vector via the abs() function.
 */
template< typename VT  // Type of the sparse vector
        , bool TF >    // Transposition flag
class SVecAbsExpr : public SparseVector< SVecAbsExpr<VT,TF>, TF >
                  , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT::TransposeType  TT;  //!< Transpose type of the left-hand side dense vector expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef SVecAbsExpr<VT,TF>        This;           //!< Type of this SVecAbsExpr instance.
   typedef typename VT::ResultType   ResultType;     //!< Result type for expression template evaluations.
   typedef typename VT::ElementType  ElementType;    //!< Resulting element type.
   typedef const SVecAbsExpr&        CompositeType;  //!< Data type for composite expression templates.
   typedef SVecAbsExpr<TT,!TF>       TransposeType;  //!< Transpose type for expression template evaluations.

   //! Composite type of the sparse vector expression.
   typedef typename VT::CompositeType  CT;
   //**********************************************************************************************

   //**ConstIterator class definition**************************************************************
   /*!\brief Iterator over the elements of the sparse vector absolute value expression.
   */
   class ConstIterator
   {
    public:
      //**Type definitions*************************************************************************
      //! Iterator type of the sparse vector expression.
      typedef typename boost::remove_reference<CT>::type::ConstIterator  IteratorType;
      //*******************************************************************************************

      //**Constructor******************************************************************************
      /*!\brief Constructor for the ConstIterator class.
      */
      inline ConstIterator( IteratorType it )
         : it_( it )  // Iterator over the elements of the sparse vector expression
      {}
      //*******************************************************************************************

      //**Prefix increment operator****************************************************************
      /*!\brief Pre-increment operator.
      //
      // \return Reference to the incremented expression iterator.
      */
      inline ConstIterator& operator++() {
         ++it_;
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
         return std::abs( it_->value() );
      }
      //*******************************************************************************************

      //**Index function***************************************************************************
      /*!\brief Access to the current index of the sparse element.
      //
      // \return The current index of the sparse element.
      */
      inline size_t index() const {
         return it_->index();
      }
      //*******************************************************************************************

      //**Equality operator************************************************************************
      /*!\brief Equality comparison between two ConstIterator objects.
      //
      // \param rhs The right-hand side expression iterator.
      // \return \a true if the iterators refer to the same element, \a false if not.
      */
      inline bool operator==( const ConstIterator& rhs ) const {
         return it_ == rhs.it_;
      }
      //*******************************************************************************************

      //**Inequality operator**********************************************************************
      /*!\brief Inequality comparison between two ConstIterator objects.
      //
      // \param rhs The right-hand side expression iterator.
      // \return \a true if the iterators don't refer to the same element, \a false if they do.
      */
      inline bool operator!=( const ConstIterator& rhs ) const {
         return it_ != rhs.it_;
      }
      //*******************************************************************************************

    private:
      //**Member variables*************************************************************************
      IteratorType it_;  //!< Iterator over the elements of the sparse vector expression.
      //*******************************************************************************************
   };
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the SVecAbsExpr class.
   */
   explicit inline SVecAbsExpr( const VT& sv )
      : sv_( sv )
   {}
   //**********************************************************************************************

   //**Begin function******************************************************************************
   /*!\brief Returns an iterator to the first non-zero element of the sparse vector.
   //
   // \return Iterator to the first non-zero element of the sparse vector.
   */
   inline ConstIterator begin() const {
      return ConstIterator( sv_.begin() );
   }
   //**********************************************************************************************

   //**End function********************************************************************************
   /*!\brief Returns an iterator just past the last non-zero element of the sparse vector.
   //
   // \return Iterator just past the last non-zero element of the sparse vector.
   */
   inline ConstIterator end() const {
      return ConstIterator( sv_.end() );
   }
   //**********************************************************************************************

   //**Size function*******************************************************************************
   /*!\brief Returns the current size/dimension of the vector.
   //
   // \return The size of the vector.
   */
   inline size_t size() const {
      return sv_.size();
   }
   //**********************************************************************************************

   //**NonZeros function***************************************************************************
   /*!\brief Returns the number of non-zero elements in the sparse vector.
   //
   // \return The number of non-zero elements in the sparse vector.
   */
   inline size_t nonZeros() const {
      return sv_.nonZeros();
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
      return sv_.isAliased( alias );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   CT sv_;  //!< Sparse vector of the absolute value expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( VT );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT, TF );
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
/*!\brief Returns a vector containing the absolute values of each single element of \a sv.
 * \ingroup sparse_vector
 *
 * \param sv The integral sparse input vector.
 * \return The absolute value of each single element of \a sv.
 *
 * The \a abs function calculates the absolute value of each element of the sparse input vector
 * \a sv. The operator returns an expression representing this operation.\n
 * The following example demonstrates the use of the \a abs function:

   \code
   pe::SVecN a, b;
   // ... Resizing and initialization
   b = abs( a );
   \endcode
 */
template< typename VT  // Type of the sparse vector
        , bool TF >    // Transposition flag
inline const SVecAbsExpr<VT,TF> abs( const SparseVector<VT,TF>& sv )
{
   return SVecAbsExpr<VT,TF>( ~sv );
}
//*************************************************************************************************

} // namespace pe

#endif
