//=================================================================================================
/*!
 *  \file pe/math/DiagonalMatrix.h
 *  \brief Header file for the DiagonalMatrix CRTP base class
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

#ifndef _PE_MATH_DIAGONALMATRIX_H_
#define _PE_MATH_DIAGONALMATRIX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <iomanip>
#include <ostream>
#include <stdexcept>
#include <vector>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/constraints/DiagonalMatrix.h>
#include <pe/math/DenseVector.h>
#include <pe/math/Expression.h>
#include <pe/math/Functions.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/typetraits/IsExpression.h>
#include <pe/util/Assert.h>
#include <pe/util/SelectType.h>
#include <pe/util/typetraits/IsFloatingPoint.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  CLASS DIAGONALMATRIX
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup diagonal_matrix Diagonal Matrices
 * \ingroup sparse_matrix
 */
/*!\defgroup diagonal_matrix_expression Expressions
 * \ingroup diagonal_matrix
 */
/*!\brief Base class for diagonal matrices.
 * \ingroup diagonal_matrix
 *
 * The DiagonalMatrix class is a base class for all diagonal matrix classes. It provides an
 * abstraction from the actual type of the diagonal matrix, but enables a conversion back
 * to this type via the 'Curiously Recurring Template Pattern' (CRTP).
 */
template< typename MT >  // Type of the diagonal matrix
struct DiagonalMatrix
{
   //**Type definitions****************************************************************************
   typedef MT  MatrixType;  //!< Type of the sparse matrix.
   //**********************************************************************************************

   //**Non-const conversion operator***************************************************************
   /*!\brief Conversion operator for non-constant diagonal matrices.
   //
   // \return Reference of the actual type of the sparse matrix.
   */
   inline MatrixType& operator~() {
      return *static_cast<MatrixType*>( this );
   }
   //**********************************************************************************************

   //**Const conversion operators******************************************************************
   /*!\brief Conversion operator for constant diagonal matrices.
   */
   inline const MatrixType& operator~() const {
      return *static_cast<const MatrixType*>( this );
   }
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CLASS DIAGDVECMULTEXPR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Expression object for diagonal matrix-dense vector multiplications.
 * \ingroup dense_vector_expression
 *
 * The DiagDVecMultExpr class represents the compile time expression for multiplications
 * between diagonal matrices and dense vectors.
 */
template< typename MT    // Type of the left-hand side sparse matrix
        , typename VT >  // Type of the right-hand side dense vector
class DiagDVecMultExpr : public DenseVector< DiagDVecMultExpr<MT,VT> >
                       , private Expression
{
private:
   //**Type definitions****************************************************************************
   typedef typename MT::ResultType     MRT;  //!< Result type of the left-hand side dense matrix expression.
   typedef typename VT::ResultType     VRT;  //!< Result type of the right-hand side dense vector expression.
   typedef typename MT::ConstIterator  ConstIterator;  //!< Iterator over the matrix elements.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename MathTrait<MRT,VRT>::MultType  ResultType;     //!< Result type for expression template evaluations.
   typedef const DiagDVecMultExpr&                CompositeType;  //!< Data type for composite expression templates.
   typedef typename ResultType::ElementType       ElementType;    //!< Resulting element type.

   //! Member data type of the left-hand side diagonal matrix expression.
   typedef typename MT::CompositeType  Lhs;

   //! Member data type of the right-hand side dense vector expression.
   typedef typename SelectType<IsExpression<VT>::value,const VRT,const VT&>::Type  Rhs;
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\brief Constructor for the DiagDVecMultExpr class.
   */
   explicit inline DiagDVecMultExpr( const MT& mat, const VT& vec )
      : mat_( mat )  // Left-hand side diagonal matrix of the multiplication expression
      , vec_( vec )  // Right-hand side dense vector of the multiplication expression
   {}
   //**********************************************************************************************

   //**Subscript operator**************************************************************************
   /*!\brief Subscript operator for the direct access to the vector elements.
   //
   // \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
   // \return The accessed value.
   */
   inline ElementType operator[]( size_t index ) const {
      return mat_[index] * vec_[index];
   }
   //**********************************************************************************************

   //**Size function*******************************************************************************
   /*!\brief Returns the current size/dimension of the vector.
   //
   // \return The size of the vector.
   */
   inline size_t size() const {
      return vec_.size();
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
   Lhs mat_;  //!< Left-hand side diagonal matrix of the multiplication expression.
   Rhs vec_;  //!< Right-hand side dense vector of the multiplication expression.
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_DIAGONAL_MATRIX_TYPE( MT );
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE ( VT );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the assignment of a diagonal matrix to another diagonal matrix.
 * \ingroup diagonal_matrix
 *
 * \param lhs The target left-hand side diagonal matrix.
 * \param rhs The right-hand side diagonal matrix to be assigned.
 * \return void
 *
 * This function implements the default assignment of a diagonal matrix to another diagonal matrix.
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void assignExpression( DiagonalMatrix<MT1>& lhs, const DiagonalMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid sizes" );

   (~lhs).assign( rhs );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name DiagonalMatrixN operators */
//@{
template< typename T1, typename T2 >
inline bool operator==( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline bool operator!=( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs );

template< typename MT >
std::ostream& operator<<( std::ostream& os, const DiagonalMatrix<MT>& sm );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two diagonal matrices.
 * \ingroup diagonal_matrix
 *
 * \param lhs The left-hand side diagonal matrix for the comparison.
 * \param rhs The right-hand side diagonal matrix for the comparison.
 * \return \a true if the two diagonal matrices are equal, \a false if not.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side diagonal matrix
inline bool operator==( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs )
{
   // Early exit in case the matrix sizes don't match
   if( (~lhs).size() != (~rhs).size() )
      return false;

   for( size_t i = 0; i<(~lhs).size(); ++i ) {
      if( !equal( (~lhs)[i], (~rhs)[i] ) ) {
         return false;
      }
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two diagonal matrices.
 * \ingroup diagonal_matrix
 *
 * \param lhs The left-hand side diagonal matrix for the comparison.
 * \param rhs The right-hand side diagonal matrix for the comparison.
 * \return \a true if the two diagonal matrices are not equal, \a false if they are equal.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side diagonal matrix
inline bool operator!=( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for diagonal matrices.
 * \ingroup diagonal_matrix
 *
 * \param os Reference to the output stream.
 * \param dm Reference to a constant diagonal matrix object.
 * \return Reference to the output stream.
 */
template< typename MT >  // Type of the diagonal matrix
std::ostream& operator<<( std::ostream& os, const DiagonalMatrix<MT>& dm )
{
   for( size_t i=0; i<(~dm).size(); ++i ) {
      os << std::setw(14) << (~dm)[i];
   }
   os << "\n";

   return os;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name DiagonalMatrix arithmetic operators */
//@{
template< typename T1, typename T2 >
const typename MathTrait<T1,T2>::AddType
   operator+( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs );

template< typename T1, typename T2 >
const typename MathTrait<T1,T2>::SubType
   operator-( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs );

template< typename T1, typename T2 >
const typename MathTrait<T1,T2>::MultType
   operator*( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline const DiagDVecMultExpr<T1,T2>
   operator*( const DiagonalMatrix<T1>& mat, const DenseVector<T2>& vec );

template< typename T1, typename T2 >
inline const typename MathTrait<T1,T2>::MultType
   operator*( const DiagonalMatrix<T1>& mat, const SparseVector<T2>& vec );

template< typename T1, typename T2 >
inline const typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType
   operator*( const DiagonalMatrix<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline const typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType
   operator*( T1 scalar, const DiagonalMatrix<T2>& mat );

template< typename T1, typename T2 >
inline const typename MathTrait<T1,T2,IsNumeric<T2>::value>::DivType
   operator/( const DiagonalMatrix<T1>& vec, T2 scalar );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition operator for the addition of two diagonal matrices (\f$ A=B+C \f$).
 * \ingroup diagonal_matrix
 *
 * \param lhs The left-hand side matrix for the matrix addition.
 * \param rhs The right-hand side matrix to be added to the left-hand side matrix.
 * \return The sum of the two diagonal matrices.
 * \exception std::invalid_argument Matrix sizes do not match
 *
 * This operator represents the addition of two diagonal matrices:

   \code
   pe::DiagN A, B, C;
   // ... Resizing and initialization
   C = A + B;
   \endcode

 * The operator returns a diagonal matrix of the higher-order element type of the two involved
 * vector element types \a T1::ELementType and \a T2::ElementType. Both matrix types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current number of rows and columns of the two given matrices don't match, a
 * \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side diagonal matrix
const typename MathTrait<T1,T2>::AddType
   operator+( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs )
{
   // Checking the sizes of the matrices
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   typedef typename MathTrait<T1,T2>::AddType  AddType;

   AddType tmp( (~lhs).size() );

   for( size_t i=0; i<(~lhs).size(); ++i ) {
      tmp[i] = (~lhs)[i] + (~rhs)[i];
   }
   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction operator for the subtraction of two diagonal matrices (\f$ A=B+C \f$).
 * \ingroup diagonal_matrix
 *
 * \param lhs The left-hand side matrix for the matrix subtraction.
 * \param rhs The right-hand side matrix to be subtracted from the left-hand side matrix.
 * \return The sum of the two diagonal matrices.
 * \exception std::invalid_argument Matrix sizes do not match
 *
 * This operator represents the subtraction of two diagonal matrices:

   \code
   pe::MatN A, B, C;
   // ... Resizing and initialization
   C = A - B;
   \endcode

 * The operator returns a diagonal matrix of the higher-order element type of the two involved
 * vector element types \a T1::ELementType and \a T2::ElementType. Both matrix types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current number of rows and columns of the two given  matrices don't match, a
 * \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side diagonal matrix
const typename MathTrait<T1,T2>::SubType
   operator-( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs )
{
   // Checking the sizes of the matrices
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   typedef typename MathTrait<T1,T2>::AddType  AddType;

   AddType tmp( (~lhs).size() );

   for( size_t i=0; i<(~lhs).size(); ++i ) {
      tmp[i] = (~lhs)[i] - (~rhs)[i];
   }
   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of two diagonal matrices (\f$ A=B*C \f$).
 * \ingroup diagonal_matrix
 *
 * \param lhs The left-hand side diagonal matrix for the multiplication.
 * \param rhs The right-hand side diagonal matrix for the multiplication.
 * \return The resulting matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This operator represents the multiplication of two diagonal matrices:

   \code
   pe::DiagN A, B, C;
   // ... Resizing and initialization
   C = A * B;
   \endcode

 * The operator returns a diagonal matrix of the higher-order element type of the two involved
 * matrix element types \a T1::ELementType and \a T2::ElementType. Both matrix types \a T1
 * and \a T2 as well as the two element types \a T1::ElementType and \a T2::ElementType have
 * to be supported by the MathTrait class template.\n
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side diagonal matrix
const typename MathTrait<T1,T2>::MultType
   operator*( const DiagonalMatrix<T1>& lhs, const DiagonalMatrix<T2>& rhs )
{
   // Checking the sizes of the matrices
   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Matrix sizes do not match" );

   typedef typename MathTrait<T1,T2>::MultType  MultType;

   MultType tmp( (~lhs).size() );

   for( size_t i=0; i<(~lhs).size(); ++i ) {
      tmp[i] = (~lhs)[i] * (~rhs)[i];
   }
   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a diagonal matrix and a dense vector
 *        (\f$ \vec{a}=B*\vec{c} \f$).
 * \ingroup diagonal_matrix
 *
 * \param mat The left-hand side diagonal matrix for the multiplication.
 * \param vec The right-hand side dense vector for the multiplication.
 * \return The resulting vector.
 * \exception std::invalid_argument Matrix and vector sizes do not match.
 *
 * This operator represents the multiplication between a diagonal matrix and a dense vector:

   \code
   pe::DiagN A;
   pe::VecN x, y;
   // ... Resizing and initialization
   y = A * x;
   \endcode

 * The operator returns a dense vector of the higher-order element type of the two involved
 * element types \a T1::ELementType and \a T2::ElementType. Both the diagonal matrix type \a T1
 * and the dense vector type \a T2 as well as the two element types \a T1::ElementType and
 * \a T2::ElementType have to be supported by the MathTrait class template.\n
 * In case the current size of the vector \a vec doesn't match the current number of columns
 * of the matrix \a mat, a \a std::invalid_argument is thrown.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side dense vector
inline const DiagDVecMultExpr<T1,T2>
   operator*( const DiagonalMatrix<T1>& mat, const DenseVector<T2>& vec )
{
   if( (~mat).size() != (~vec).size() )
      throw std::invalid_argument( "Matrix and vector sizes do not match" );

   return DiagDVecMultExpr<T1,T2>( ~mat, ~vec );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a diagonal matrix and a scalar value
 *        (\f$ A=B*s \f$).
 * \ingroup diagonal_matrix
 *
 * \param mat The left-hand side diagonal matrix for the multiplication.
 * \param scalar The right-hand side scalar value for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a diagonal matrix and a scalar value:

   \code
   pe::DiagN A, B;
   // ... Resizing and initialization
   B = A * 1.25;
   \endcode

 * The operator returns a diagonal matrix of the higher-order element type of the involved data
 * types \a T1::ElementType and \a T2. Note that this operator only works for scalar values
 * of built-in data type.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side scalar
inline const typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType
   operator*( const DiagonalMatrix<T1>& mat, T2 scalar )
{
   typedef typename MathTrait<T1,T2>::MultType  MultType;

   MultType tmp( mat );
   for( size_t i=0; i<tmp.size(); ++i ) {
      tmp[i] *= scalar;
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a scalar value and a diagonal matrix
 *        (\f$ A=s*B \f$).
 * \ingroup diagonal_matrix
 *
 * \param scalar The left-hand side scalar value for the multiplication.
 * \param mat The right-hand side diagonal matrix for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a scalar value and a diagonal matrix:

   \code
   pe::DiagN A, B;
   // ... Resizing and initialization
   B = 1.25 * A;
   \endcode

 * The operator returns a diagonal matrix of the higher-order element type of the involved data
 * types \a T1 and \a T2::ElementType. Note that this operator only works for scalar values
 * of built-in data type.
 */
template< typename T1    // Type of the left-hand side scalar
        , typename T2 >  // Type of the right-hand side diagonal matrix
inline const typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType
   operator*( T1 scalar, const DiagonalMatrix<T2>& mat )
{
   typedef typename MathTrait<T1,T2>::MultType  MultType;

   MultType tmp( mat );
   for( size_t i=0; i<tmp.size(); ++i ) {
      tmp[i] *= scalar;
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division operator for the division of a diagonal matrix by a scalar value (\f$ A=B/s \f$).
 * \ingroup diagonal_matrix
 *
 * \param mat The left-hand side diagonal matrix for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result matrix.
 *
 * This operator represents the division of a diagonal matrix by a scalar value:

   \code
   pe::DiagN A, B;
   // ... Resizing and initialization
   B = A / 0.24;
   \endcode

 * The operator returns a diagonal matrix of the higher-order element type of the involved data
 * types \a T1::ElementType and \a T2. Note that this operator only works for scalar values
 * of built-in data type.
 *
 * \b Note: A division by zero is only checked by a user assert.
 */
template< typename T1    // Type of the left-hand side diagonal matrix
        , typename T2 >  // Type of the right-hand side scalar
inline const typename MathTrait<T1,T2,IsNumeric<T2>::value>::DivType
   operator/( const DiagonalMatrix<T1>& mat, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   typedef typename MathTrait<T1,T2>::DivType  DivType;
   typedef typename DivType::ElementType       ET;
   typedef typename DivType::Iterator          Iterator;

   DivType tmp( mat );

   // Depending on the two involved element data types, an integer division is applied
   // or a floating point division is selected.
   if( IsNumeric<ET>::value && IsFloatingPoint<ET>::value ) {
      const ET idiv( ET(1)/static_cast<ET>( scalar ) );
      for( size_t i=0; i<tmp.size(); ++i )
         tmp[i] *= idiv;
   }
   else {
      for( size_t i=0; i<tmp.size(); ++i )
         tmp[i] /= scalar;
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
