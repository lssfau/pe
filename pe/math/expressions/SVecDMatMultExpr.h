//=================================================================================================
/*!
 *  \file pe/math/expressions/SVecDMatMultExpr.h
 *  \brief Header file for the sparse vector/dense matrix multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_SVECDMATMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_SVECDMATMULTEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/math/constraints/DenseMatrix.h>
#include <pe/math/constraints/SparseVector.h>
#include <pe/math/constraints/TransposeVector.h>
#include <pe/math/Expression.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/SelectType.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS SVECDMATMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for sparse vector-dense matrix multiplications.
 * \ingroup dense_vector_expression
 *
 * The SVecDMatMultExpr class represents the compile time expression for multiplications
 * between sparse vectors and dense matrices.
 */
template< typename VT    // Type of the left-hand side sparse vector
        , typename MT >  // Type of the right-hand side dense matrix
class SVecDMatMultExpr : public DenseVector< SVecDMatMultExpr<VT,MT>, true >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename VT::ResultType     VRT;            //!< Result type of the left-hand side sparse vector expression.
   typedef typename MT::ResultType     MRT;            //!< Result type of the right-hand side dense matrix expression.
   typedef typename VT::ConstIterator  ConstIterator;  //!< Iterator over the sparse vector elements.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef SVecDMatMultExpr<VT,MT>                This;           //!< Type of this SVecDMatMultExpr instance.
   typedef typename MathTrait<VRT,MRT>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::TransposeType     TransposeType;  //!< Transpose type for expression template evaluations.
   typedef const SVecDMatMultExpr&                CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side sparse vector expression.
   typedef typename SelectType<IsExpression<VT>::value,const VRT,const VT&>::Type  Lhs;

   //! Composite type of the right-hand side dense matrix expression.
   typedef typename MT::CompositeType  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the SVecDMatMultExpr class.
   */
   explicit inline SVecDMatMultExpr( const VT& vec, const MT& mat )
      : vec_( vec )  // Left-hand side sparse vector of the multiplication expression
      , mat_( mat )  // Right-hand side dense matrix of the multiplication expression
   {
      pe_INTERNAL_ASSERT( vec_.size() == mat_.rows(), "Invalid vector and matrix sizes" );
   }
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline const ElementType operator[]( size_t index ) const {
      pe_INTERNAL_ASSERT( index < mat_.columns(), "Invalid vector access index" );

      ElementType res;
      ConstIterator element( vec_.begin() );

      if( element != vec_.end() ) {
         res = element->value() * mat_( element->index(), index );
         ++element;
         for( ; element!=vec_.end(); ++element )
            res += element->value() * mat_( element->index(), index );
      }
      else {
         reset( res );
      }

      return res;
   }
   //**********************************************************************************************

   //**Size function*******************************************************************************
   /*!\brief Returns the current size/dimension of the vector.
   //
   // \return The size of the vector.
   */
   inline size_t size() const {
      return mat_.columns();
   }
   //**********************************************************************************************

   //**********************************************************************************************
   /*!\brief Returns whether the expression is aliased with the given address \a alias.
   //
   // \param alias The alias to be checked.
   // \return \a true in case the given alias is contained in this expression, \a false if not.
   */
   template< typename T >
   inline bool isAliased( const T* alias ) const {
      return ( vec_.isAliased( alias ) ) ||
             ( IsExpression<MT>::value && mat_.isAliased( alias ) );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs vec_;  //!< Left-hand side sparse vector of the multiplication expression.
   Rhs mat_;  //!< Right-hand side dense matrix of the multiplication expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( VT );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT, true );
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE( MT );
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
/*!\brief Multiplication operator for the multiplication of a sparse vector and a dense matrix
 *        (\f$ \vec{a}^T=\vec{b}^T*C \f$).
 * \ingroup dense_matrix
 *
 * \param vec The left-hand side transpose sparse vector for the multiplication.
 * \param mat The right-hand side dense matrix for the multiplication.
 * \return The resulting transpose vector.
 * \exception std::invalid_argument Vector and matrix sizes do not match.
 *
 * This operator represents the multiplication between a transpose sparse vector and a dense
 * matrix:

   \code
   pe::MatN A;
   pe::SVecNT x, y;
   // ... Resizing and initialization
   y = x * A;
   \endcode

 * The operator returns an expression representing a transpose sparse vector of the higher-order
 * element type of the two involved element types \a T1::ElementType and \a T2::ElementType.
 * Both the dense matrix type \a T1 and the dense vector type \a T2 as well as the two element
 * types \a T1::ElementType and \a T2::ElementType have to be supported by the MathTrait class
 * template.\n
 * In case the current size of the vector \a vec doesn't match the current number of rows of
 * the matrix \a mat, a \a std::invalid_argument is thrown.
 */
template< typename T1, typename T2 >
inline const SVecDMatMultExpr<T1,T2>
   operator*( const SparseVector<T1,true>& vec, const DenseMatrix<T2>& mat )
{
   if( (~vec).size() != (~mat).rows() )
      throw std::invalid_argument( "Vector and matrix sizes do not match" );

   return SVecDMatMultExpr<T1,T2>( ~vec, ~mat );
}
//*************************************************************************************************

} // namespace pe

#endif
