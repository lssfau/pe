//=================================================================================================
/*!
 *  \file pe/math/expressions/DVecSVecMultExpr.h
 *  \brief Header file for the dense vector/sparse vector multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DVECSVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DVECSVECMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Reference.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DVECSVECMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense vector-sparse vector multiplications.
 * \ingroup sparse_vector_expression
 *
 * The DVecSVecMultExpr class represents the compile time expression for componentwise
 * multiplications between a dense vector and a sparse vector.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF >     // Transposition flag
class DVecSVecMultExpr : public SparseVector< DVecSVecMultExpr<VT1,VT2,TF>, TF >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT1::ResultType     RT1;  //!< Result type of the left-hand side dense vector expression.
   typedef typename VT2::ResultType     RT2;  //!< Result type of the right-hand side sparse vector expression.
   typedef typename VT1::TransposeType  TT1;  //!< Transpose type of the left-hand side sparse vector expression.
   typedef typename VT2::TransposeType  TT2;  //!< Transpose type of the right-hand side dense vector expression.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DVecSVecMultExpr<VT1,VT2,TF>           This;           //!< Type of this DVecSVecMultExpr instance.
   typedef typename MathTrait<RT1,RT2>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.
   typedef const ResultType                       CompositeType;  //!< Data type for composite expression templates.
   typedef DVecSVecMultExpr<TT1,TT2,!TF>          TransposeType;  //!< Transpose type for expression template evaluations.

   //! Composite type of the left-hand side dense vector expression.
   typedef typename VT1::CompositeType  Lhs;

   //! Composite type of the right-hand side sparse vector expression.
   typedef typename VT2::CompositeType  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DVecSVecMultExpr class.
   */
   explicit inline DVecSVecMultExpr( const VT1& lhs, const VT2& rhs )
      : lhs_( lhs )  // Left-hand side dense vector of the multiplication expression
      , rhs_( rhs )  // Right-hand side sparse vector of the multiplication expression
   {
      pe_INTERNAL_ASSERT( lhs.size() == rhs.size(), "Invalid vector sizes" );
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

   //**NonZeros function***************************************************************************
   /*!\brief Returns the number of non-zero elements in the sparse vector.
   //
   // \return The number of non-zero elements in the sparse vector.
   */
   inline size_t nonZeros() const {
      return rhs_.nonZeros();
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
             ( rhs_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs lhs_;  //!< Left-hand side dense vector of the multiplication expression.
   Rhs rhs_;  //!< Right-hand side sparse vector of the multiplication expression.
   //**********************************************************************************************

   //**Assignment to dense vectors*****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-sparse
   // vector multiplication expression to a dense vector.
   */
   template< typename VT  // Type of the target dense vector
           , bool TF2 >   // Transposition flag of the target dense vector
   friend inline void assign( DenseVector<VT,TF2>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  ConstIterator;

      for( ConstIterator element=rhs.rhs_.begin(); element!=rhs.rhs_.end(); ++element )
         (~lhs)[element->index()] = rhs.lhs_[element->index()] * element->value();
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Assignment to sparse vectors****************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Assignment of a dense vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be assigned.
   // \return void
   //
   // This function implements the performance optimized assignment of a dense vector-sparse
   // vector multiplication expression to a sparse vector.
   */
   template< typename VT  // Type of the target sparse vector
           , bool TF2 >   // Transposition flag of the target sparse vector
   friend inline void assign( SparseVector<VT,TF2>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  ConstIterator;

      for( ConstIterator element=rhs.rhs_.begin(); element!=rhs.rhs_.end(); ++element )
         (~lhs).append( element->index(), rhs.lhs_[element->index()] * element->value() );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to dense vectors********************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // sparse vector multiplication expression to a dense vector.
   */
   template< typename VT  // Type of the target dense vector
           , bool TF2 >   // Transposition flag of the target dense vector
   friend inline void addAssign( DenseVector<VT,TF2>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  ConstIterator;

      for( ConstIterator element=rhs.rhs_.begin(); element!=rhs.rhs_.end(); ++element )
         (~lhs)[element->index()] += rhs.lhs_[element->index()] * element->value();
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Addition assignment to sparse vectors*******************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Addition assignment of a dense vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be added.
   // \return void
   //
   // This function implements the performance optimized addition assignment of a dense vector-
   // sparse vector multiplication expression to a sparse vector.
   */
   template< typename VT  // Type of the target sparse vector
           , bool TF2 >   // Transposition flag of the target sparse vector
   friend inline void addAssign( SparseVector<VT,TF2>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( ResultType );
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
   /*!\brief Subtraction assignment of a dense vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // sparse vector multiplication expression to a dense vector.
   */
   template< typename VT  // Type of the target dense vector
           , bool TF2 >   // Transposition flag of the target dense vector
   friend inline void subAssign( DenseVector<VT,TF2>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  ConstIterator;

      for( ConstIterator element=rhs.rhs_.begin(); element!=rhs.rhs_.end(); ++element )
         (~lhs)[element->index()] -= rhs.lhs_[element->index()] * element->value();
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Subtraction assignment to sparse vectors****************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Subtraction assignment of a dense vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be subtracted.
   // \return void
   //
   // This function implements the performance optimized subtraction assignment of a dense vector-
   // sparse vector multiplication expression to a sparse vector.
   */
   template< typename VT  // Type of the target sparse vector
           , bool TF2 >   // Transposition flag of the target sparse vector
   friend inline void subAssign( SparseVector<VT,TF2>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( ResultType );
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
   /*!\brief Multiplication assignment of a dense vector-sparse vector multiplication to a dense vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side dense vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-sparse vector multiplication expression to a dense vector.
   */
   template< typename VT  // Type of the target dense vector
           , bool TF2 >   // Transposition flag of the target dense vector
   friend inline void multAssign( DenseVector<VT,TF2>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_INTERNAL_ASSERT( (~lhs).size() == rhs.size(), "Invalid vector sizes" );

      typedef typename boost::remove_reference<Rhs>::type::ConstIterator  ConstIterator;

      const ConstIterator end( rhs.rhs_.end() );
      ConstIterator begin( rhs.rhs_.begin() );
      size_t i( 0 );

      for( ; begin!=end; ++begin ) {
         const size_t index( begin->index() );
         for( ; i<index; ++i )
            reset( (~lhs)[i] );
         (~lhs)[index] *= rhs.lhs_[index] * begin->value();
         ++i;
      }

      for( ; i<rhs.size(); ++i )
         reset( (~lhs)[i] );
   }
   /*! \endcond */
   //**********************************************************************************************

   //**Multiplication assignment to sparse vectors*************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Multiplication assignment of a dense vector-sparse vector multiplication to a sparse vector.
   // \ingroup sparse_vector
   //
   // \param lhs The target left-hand side sparse vector.
   // \param rhs The right-hand side multiplication expression to be multiplied.
   // \return void
   //
   // This function implements the performance optimized multiplication assignment of a dense
   // vector-sparse vector multiplication expression to a sparse vector.
   */
   template< typename VT >  // Type of the target sparse vector
   friend inline void multAssign( SparseVector<VT,TF>& lhs, const DVecSVecMultExpr& rhs )
   {
      pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( ResultType );
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
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE ( VT1 );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( VT2 );
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
/*!\brief Multiplication operator for the componentwise product of a dense vector and a sparse
 *        vector (\f$ \vec{a}=\vec{b}*\vec{c} \f$).
 * \ingroup sparse_vector
 *
 * \param lhs The left-hand side dense vector for the component product.
 * \param rhs The right-hand side sparse vector for the component product.
 * \return The product of the two vectors.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the componentwise multiplication of a dense vector and a sparse
 * vector:

   \code
   pe::VecN a;
   pe::SVecN b, c;
   // ... Resizing and initialization
   c = a * b;
   \endcode

 * The operator returns a sparse vector of the higher-order element type of the two involved
 * vector element types \a T1::ElementType and \a T2::ElementType. Both vector types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline const DVecSVecMultExpr<T1,T2,TF>
   operator*( const DenseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs )
{
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   return DVecSVecMultExpr<T1,T2,TF>( ~lhs, ~rhs );
}
//*************************************************************************************************

} // namespace pe

#endif
