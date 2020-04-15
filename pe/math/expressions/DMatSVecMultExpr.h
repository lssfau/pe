//=================================================================================================
/*!
 *  \file pe/math/expressions/DMatSVecMultExpr.h
 *  \brief Header file for the dense matrix/sparse vector multiplication expression
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

#ifndef _PE_MATH_EXPRESSIONS_DMATSVECMULTEXPR_H_
#define _PE_MATH_EXPRESSIONS_DMATSVECMULTEXPR_H_


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
//  CLASS DMATSVECMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for dense matrix-sparse vector multiplications.
 * \ingroup dense_vector_expression
 *
 * The DMatSVecMultExpr class represents the compile time expression for multiplications
 * between dense matrices and sparse vectors.
 */
template< typename MT    // Type of the left-hand side dense matrix
        , typename VT >  // Type of the right-hand side sparse vector
class DMatSVecMultExpr : public DenseVector< DMatSVecMultExpr<MT,VT>, false >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT::ResultType     MRT;            //!< Result type of the left-hand side dense matrix expression.
   typedef typename VT::ResultType     VRT;            //!< Result type of the right-hand side sparse vector expression.
   typedef typename VT::ConstIterator  ConstIterator;  //!< Iterator over the sparse vector elements.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef DMatSVecMultExpr<MT,VT>                This;           //!< Type of this DMatSVecMultExpr instance.
   typedef typename MathTrait<MRT,VRT>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef typename ResultType::TransposeType     TransposeType;  //!< Transpose type for expression template evaluations.
   typedef const DMatSVecMultExpr&                CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Composite type of the left-hand side dense matrix expression.
   typedef typename MT::CompositeType  Lhs;

   //! Composite type of the right-hand side sparse vector expression.
   typedef typename SelectType<IsExpression<VT>::value,const VRT,const VT&>::Type  Rhs;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DMatSVecMultExpr class.
   */
   explicit inline DMatSVecMultExpr( const MT& mat, const VT& vec )
      : mat_( mat )  // Left-hand side dense matrix of the multiplication expression
      , vec_( vec )  // Right-hand side sparse vector of the multiplication expression
   {
      pe_INTERNAL_ASSERT( mat_.columns() == vec_.size(), "Invalid matrix and vector sizes" );
   }
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline const ElementType operator[]( size_t index ) const {
      pe_INTERNAL_ASSERT( index < mat_.rows(), "Invalid vector access index" );

      ElementType res;
      ConstIterator element( vec_.begin() );

      if( element != vec_.end() ) {
         res = mat_( index, element->index() ) * element->value();
         ++element;
         for( ; element!=vec_.end(); ++element )
            res += mat_( index, element->index() ) * element->value();
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
      return mat_.rows();
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
      return vec_.isAliased( alias );
   }
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   Lhs mat_;  //!< Left-hand side dense matrix of the multiplication expression.
   Rhs vec_;  //!< Right-hand side sparse vector of the multiplication expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DENSE_MATRIX_TYPE ( MT );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( VT );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( VT, false );
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
/*!\brief Multiplication operator for the multiplication of a dense matrix and a sparse vector
 *        (\f$ \vec{a}=B*\vec{c} \f$).
 * \ingroup dense_matrix
 *
 * \param mat The left-hand side dense matrix for the multiplication.
 * \param vec The right-hand side sparse vector for the multiplication.
 * \return The resulting vector.
 * \exception std::invalid_argument Matrix and vector sizes do not match.
 *
 * This operator represents the multiplication between a dense matrix and a sparse vector:

   \code
   pe::MatN A;
   pe::SVecN x;
   pe::VecN y;
   // ... Resizing and initialization
   y = A * x;
   \endcode

 * The operator returns an expression representing a dense vector of the higher-order element
 * type of the two involved element types \a T1::ElementType and \a T2::ElementType. Both the
 * dense matrix type \a T1 and the sparse vector type \a T2 as well as the two element types
 * \a T1::ElementType and \a T2::ElementType have to be supported by the MathTrait class
 * template.\n
 * In case the current size of the vector \a vec doesn't match the current number of columns
 * of the matrix \a mat, a \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side sparse vector
inline const DMatSVecMultExpr<T1,T2>
   operator*( const DenseMatrix<T1>& mat, const SparseVector<T2,false>& vec )
{
   if( (~mat).columns() != (~vec).size() )
      throw std::invalid_argument( "Matrix and vector sizes do not match" );

   return DMatSVecMultExpr<T1,T2>( ~mat, ~vec );
}
//*************************************************************************************************

} // namespace pe

#endif
