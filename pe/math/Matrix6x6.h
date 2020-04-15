//=================================================================================================
/*!
 *  \file pe/math/Matrix6x6.h
 *  \brief Implementation of a 6x6 matrix
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

#ifndef _PE_MATH_MATRIX6X6_H_
#define _PE_MATH_MATRIX6X6_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <ostream>
#include <pe/math/DenseMatrix.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/Vector6.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Const.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/constraints/Volatile.h>
#include <pe/util/EnableIf.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsBuiltin.h>
#include <pe/util/typetraits/IsFloatingPoint.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename Type >       struct SparseMatrix;
template< typename Type, bool > class  SparseVectorN;
template< typename Type, bool > class  Vector6;
template< typename Type, bool > class  VectorN;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup dense_matrix_6x6 Matrix6x6
 * \ingroup dense_matrix
 */
/*!\brief Efficient, generic implementation of a 6x6 matrix.
 * \ingroup dense_matrix_6x6
 *
 * The Matrix6x6 class is the representation of a 6x6 matrix with a total of 36 statically
 * allocated elements of arbitrary type. The naming convention of the elements is as following:

                          \f[\left(\begin{array}{*{6}{c}}
                          aa & ab & ac & ad & ae & af \\
                          ba & bb & bc & bd & be & bf \\
                          ca & cb & cc & cd & ce & cf \\
                          da & db & dc & dd & de & df \\
                          ea & eb & ec & ed & ee & ef \\
                          fa & fb & fc & fd & fe & ff \\
                          \end{array}\right)\f]\n

 * These elements can be accessed directly with the 1D subscript operator or with the 2D function
 * operator. The numbering of the matrix elements is

                          \f[\left(\begin{array}{*{6}{c}}
                          0  & 1  & 2  & 3  & 4  & 5  \\
                          6  & 7  & 8  & 9  & 10 & 11 \\
                          12 & 13 & 14 & 15 & 16 & 17 \\
                          18 & 19 & 20 & 21 & 22 & 23 \\
                          24 & 25 & 26 & 27 & 28 & 29 \\
                          30 & 31 & 32 & 33 & 34 & 35 \\
                          \end{array}\right)\f]

 * Matrix6x6 can be used with any non-cv-qualified element type. The arithmetic operators for
 * matrix/matrix, matrix/vector and matrix/element operations with the same element type work
 * for any element type as long as the element type supports the arithmetic operation. Arithmetic
 * operations between matrices, vectors and elements of different element types are only supported
 * for all data types supported by the MathTrait class template (for details see the MathTrait
 * class description).

   \code
   Matrix6x6< double > a, b, c;
   Matrix6x6< float  > d;
   Matrix6x6< std::complex<double> > e, f, g;
   Matrix6x6< std::complex<float>  > h;

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type >  // Data type of the matrix
class Matrix6x6 : public DenseMatrix< Matrix6x6<Type> >
{
public:
   //**Type definitions****************************************************************************
   typedef Matrix6x6<Type>   This;           //!< Type of this Matrix6x6 instance.
   typedef This              ResultType;     //!< Result type for expression template evaluations.
   typedef Type              ElementType;    //!< Type of the matrix elements.
   typedef const Matrix6x6&  CompositeType;  //!< Data type for composite expression templates.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                              explicit inline Matrix6x6();
                              explicit inline Matrix6x6( Type init );
                                       inline Matrix6x6( const Matrix6x6& m );
   template< typename Other >          inline Matrix6x6( const Matrix6x6<Other>& m  );
   template< typename MT >             inline Matrix6x6( const DenseMatrix<MT>&  dm );
   template< typename MT >             inline Matrix6x6( const SparseMatrix<MT>& sm );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
                              inline Matrix6x6&  operator= ( Type rhs );
                              inline Matrix6x6&  operator= ( const Matrix6x6& rhs );
   template< typename Other > inline Matrix6x6&  operator= ( const Matrix6x6<Other>& rhs );
   template< typename MT >    inline Matrix6x6&  operator= ( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix6x6&  operator= ( const SparseMatrix<MT>& rhs );
                              inline Type&       operator[]( size_t index );
                              inline const Type& operator[]( size_t index )       const;
                              inline Type&       operator()( size_t i, size_t j );
                              inline const Type& operator()( size_t i, size_t j ) const;
   template< typename MT >    inline Matrix6x6&  operator+=( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix6x6&  operator+=( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline Matrix6x6&  operator-=( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix6x6&  operator-=( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline Matrix6x6&  operator*=( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline Matrix6x6&  operator*=( const SparseMatrix<MT>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, Matrix6x6 >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, Matrix6x6 >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t          rows()         const;
                              inline size_t          columns()      const;
                              inline void            reset();
                              inline Matrix6x6&      transpose();
                              inline Matrix6x6&      invert();
                              inline const Matrix6x6 getInverse()   const;
   template< typename Other > inline Matrix6x6&      scale( Other scalar );
                              inline bool            isDiagonal()   const;
                              inline bool            isSymmetric()  const;
                              inline void            swap( Matrix6x6& m ) /* throw() */;
   //@}
   //**********************************************************************************************

   //**Expression template evaluation functions****************************************************
   /*!\name Expression template evaluation functions */
   //@{
   template< typename Other > inline bool isAliased( const Other* alias ) const;
   template< typename MT >    inline void assign   ( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void assign   ( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline void addAssign( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void addAssign( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline void subAssign( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void subAssign( const SparseMatrix<MT>& rhs );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Type v_[36];  //!< The nine statically allocated matrix elements.
                 /*!< Access to the matrix elements is gained via the subscript or function call
                      operator. The order of the elements is
                      \f[\left(\begin{array}{*{6}{c}}
                      0  & 1  & 2  & 3  & 4  & 5  \\
                      6  & 7  & 8  & 9  & 10 & 11 \\
                      12 & 13 & 14 & 15 & 16 & 17 \\
                      18 & 19 & 20 & 21 & 22 & 23 \\
                      24 & 25 & 26 & 27 & 28 & 29 \\
                      30 & 31 & 32 & 33 & 34 & 35 \\
                      \end{array}\right)\f] */
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_NOT_BE_CONST   ( Type );
   pe_CONSTRAINT_MUST_NOT_BE_VOLATILE( Type );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for Matrix6x6.
 *
 * All matrix elements are initialized to the default value (i.e. 0 for integral data types).
 */
template< typename Type >  // Data type of the matrix
inline Matrix6x6<Type>::Matrix6x6()
{
   for( size_t i=0; i<36; ++i )
      v_[i] = Type();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a homogenous initialization of all elements.
 *
 * \param init Initial value for all matrix elements.
 */
template< typename Type >  // Data type of the matrix
inline Matrix6x6<Type>::Matrix6x6( Type init )
{
   for( size_t i=0; i<36; ++i )
      v_[i] = init;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for Matrix6x6.
 *
 * \param m Matrix to be copied.
 *
 * The copy constructor is explicitly defined in order to enable/facilitate NRV optimization.
 */
template< typename Type >  // Data type of the matrix
inline Matrix6x6<Type>::Matrix6x6( const Matrix6x6& m )
{
   for( size_t i=0; i<36; ++i )
      v_[i] = m.v_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different Matrix6x6 instances.
 *
 * \param m Matrix to be copied.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the foreign matrix
inline Matrix6x6<Type>::Matrix6x6( const Matrix6x6<Other>& m )
{
   for( size_t i=0; i<36; ++i )
      v_[i] = m[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different dense matrices.
 *
 * \param dm Dense matrix to be copied.
 * \exception std::invalid_argument Invalid setup of 6x6 matrix.
 *
 * This constructor initializes the 6x6 matrix from the given dense matrix. In case the size
 * of the given matrix does not match the size of the 6x6 matrix (i.e. the number of rows or
 * columns is not 6), a \a std::invalid_argument exception is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the foreign dense matrix
inline Matrix6x6<Type>::Matrix6x6( const DenseMatrix<MT>& dm )
{
   using pe::assign;

   if( (~dm).rows() != size_t(6) || (~dm).columns() != size_t(6) )
      throw std::invalid_argument( "Invalid setup of 6x6 matrix" );

   assign( *this, ~dm );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from sparse matrices.
 *
 * \param sm Sparse matrix to be copied.
 * \exception std::invalid_argument Invalid setup of 6x6 matrix.
 *
 * This constructor initializes the 6x6 matrix from the given sparse matrix. In case the size
 * of the given matrix does not match the size of the 6x6 matrix (i.e. the number of rows or
 * columns is not 6), a \a std::invalid_argument exception is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the foreign sparse matrix
inline Matrix6x6<Type>::Matrix6x6( const SparseMatrix<MT>& sm )
{
   using pe::assign;

   if( (~sm).rows() != size_t(6) || (~sm).columns() != size_t(6) )
      throw std::invalid_argument( "Invalid setup of 6x6 matrix" );

   if( IsBuiltin<Type>::value )
      reset();

   assign( *this, ~sm );
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Homogenous assignment to all matrix elements.
 *
 * \param rhs Scalar value to be assigned to all matrix elements.
 * \return Reference to the assigned matrix.
 */
template< typename Type >  // Data type of the matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator=( Type rhs )
{
   for( size_t i=0; i<36; ++i )
      v_[i] = rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for Matrix6x6.
 *
 * \param rhs Matrix to be copied.
 * \return Reference to the assigned matrix.
 *
 * Explicit definition of a copy assignment operator for performance reasons.
 */
template< typename Type >  // Data type of the matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator=( const Matrix6x6& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   for( size_t i=0; i<36; ++i )
      v_[i] = rhs.v_[i];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different Matrix6x6 instances.
 *
 * \param rhs Matrix to be copied.
 * \return Reference to the assigned matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the foreign matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator=( const Matrix6x6<Other>& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   for( size_t i=0; i<36; ++i )
      v_[i] = rhs[i];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different dense matrices.
 *
 * \param rhs Dense matrix to be copied.
 * \return Reference to the assigned matrix.
 * \exception std::invalid_argument Invalid assignment to 6x6 matrix.
 *
 * This constructor initializes the matrix as a copy of the given dense matrix. In case the
 * number of rows or columns of the given matrix is not 6, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator=( const DenseMatrix<MT>& rhs )
{
   using pe::assign;

   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Invalid assignment to 6x6 matrix" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      Matrix6x6 tmp( rhs );
      swap( tmp );
   }
   else {
      assign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for sparse matrices.
 *
 * \param rhs Sparse matrix to be copied.
 * \return Reference to the assigned matrix.
 * \exception std::invalid_argument Invalid assignment to 6x6 matrix.
 *
 * This constructor initializes the matrix as a copy of the given sparse matrix. In case the
 * number of rows or columns of the given matrix is not 6, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator=( const SparseMatrix<MT>& rhs )
{
   using pe::assign;

   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Invalid assignment to 6x6 matrix" );

   reset();
   assign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 1D-access to the matrix elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..35]\f$.
 * \return Reference to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
template< typename Type >  // Data type of the matrix
inline Type& Matrix6x6<Type>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < 36, "Invalid matrix access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 1D-access to the matrix elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..35]\f$.
 * \return Reference-to-const to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
template< typename Type >  // Data type of the matrix
inline const Type& Matrix6x6<Type>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < 36, "Invalid matrix access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access to the matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range [0..5].
 * \param j Access index for the column. The index has to be in the range [0..5].
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline Type& Matrix6x6<Type>::operator()( size_t i, size_t j )
{
   pe_USER_ASSERT( i<6 && j<6, "Invalid matrix access index" );
   return v_[i*6+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access to the matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range [0..5].
 * \param j Access index for the column. The index has to be in the range [0..5].
 * \return Reference-to-const to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline const Type& Matrix6x6<Type>::operator()( size_t i, size_t j ) const
{
   pe_USER_ASSERT( i<6 && j<6, "Invalid matrix access index" );
   return v_[i*6+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a dense matrix (\f$ A+=B \f$).
 *
 * \param rhs The right-hand side dense matrix to be added to the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator+=( const DenseMatrix<MT>& rhs )
{
   using pe::addAssign;

   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      Matrix6x6 tmp( rhs );
      for( size_t i=0; i<36; ++i )
         v_[i] += tmp.v_[i];
   }
   else {
      addAssign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a sparse matrix (\f$ A+=B \f$).
 *
 * \param rhs The right-hand side sparse matrix to be added to the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator+=( const SparseMatrix<MT>& rhs )
{
   using pe::addAssign;

   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   addAssign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a dense matrix (\f$ A-=B \f$).
 *
 * \param rhs The right-hand side dense matrix to be subtracted from the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator-=( const DenseMatrix<MT>& rhs )
{
   using pe::subAssign;

   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      Matrix6x6 tmp( rhs );
      for( size_t i=0; i<36; ++i )
         v_[i] -= tmp.v_[i];
   }
   else {
      subAssign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a sparse matrix (\f$ A-=B \f$).
 *
 * \param rhs The right-hand side sparse matrix to be subtracted from the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator-=( const SparseMatrix<MT>& rhs )
{
   using pe::subAssign;

   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   subAssign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a dense matrix
 *        (\f$ A*=B \f$).
 *
 * \param rhs The right-hand side dense matrix for the multiplication.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator*=( const DenseMatrix<MT>& rhs )
{
   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   // Creating a temporary due to data dependencies
   Matrix6x6 tmp;

   for( size_t i=0; i<6; ++i ) {
      for( size_t j=0; j<6; ++j ) {
         tmp.v_[i*6  ] += v_[i*6+j] * (~rhs)(j,0);
         tmp.v_[i*6+1] += v_[i*6+j] * (~rhs)(j,1);
         tmp.v_[i*6+2] += v_[i*6+j] * (~rhs)(j,2);
         tmp.v_[i*6+3] += v_[i*6+j] * (~rhs)(j,3);
         tmp.v_[i*6+4] += v_[i*6+j] * (~rhs)(j,4);
         tmp.v_[i*6+5] += v_[i*6+j] * (~rhs)(j,5);
      }
   }

   return this->operator=( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a sparse matrix
 *        (\f$ A*=B \f$).
 *
 * \param rhs The right-hand side sparse matrix for the multiplication.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::operator*=( const SparseMatrix<MT>& rhs )
{
   if( (~rhs).rows() != size_t(6) || (~rhs).columns() != size_t(6) )
      throw std::invalid_argument( "Matrix sizes do not match" );

   typedef typename MT::ConstIterator  ConstIterator;

   const MT rhsT( trans( ~rhs ) );
   Matrix6x6 tmp;

   for( size_t i=0; i<6; ++i ) {
      for( size_t j=0; j<6; ++j ) {
         for( ConstIterator element=rhsT.begin(j); element!=rhsT.end(j); ++element ) {
            tmp[i*6+j] += v_[i*6+element->index()] * element->value();
         }
      }
   }

   return this->operator=( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication between a matrix and
 *        a scalar value (\f$ A*=s \f$).
 *
 * \param rhs The right-hand side scalar value for the multiplication.
 * \return Reference to the matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, Matrix6x6<Type> >::Type&
   Matrix6x6<Type>::operator*=( Other rhs )
{
   for( size_t i=0; i<36; ++i )
      v_[i] *= rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division assignment operator for the division of a matrix by a scalar value
 *        (\f$ A/=s \f$).
 *
 * \param rhs The right-hand side scalar value for the division.
 * \return Reference to the matrix.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename Type >   // Data type of the vector
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, Matrix6x6<Type> >::Type&
   Matrix6x6<Type>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      for( size_t i=0; i<36; ++i )
         v_[i] = static_cast<Type>( static_cast<DT>( v_[i] ) * tmp );
      return *this;
   }
   else {
      for( size_t i=0; i<36; ++i )
         v_[i] /= rhs;
      return *this;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current number of rows of the matrix.
 *
 * \return The number of rows of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t Matrix6x6<Type>::rows() const
{
   return size_t(6);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current number of columns of the matrix.
 *
 * \return The number of columns of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t Matrix6x6<Type>::columns() const
{
   return size_t(6);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
template< typename Type >  // Data type of the matrix
inline void Matrix6x6<Type>::reset()
{
   using pe::reset;
   for( size_t i=0; i<36; ++i )
      reset( v_[i] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transposing the matrix.
 *
 * \return Reference to the transposed matrix.
 */
template< typename Type >  // Data type of the matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::transpose()
{
   std::swap( v_[ 1], v_[ 6] );
   std::swap( v_[ 2], v_[12] );
   std::swap( v_[ 3], v_[18] );
   std::swap( v_[ 4], v_[24] );
   std::swap( v_[ 5], v_[30] );
   std::swap( v_[ 8], v_[13] );
   std::swap( v_[ 9], v_[19] );
   std::swap( v_[10], v_[25] );
   std::swap( v_[11], v_[31] );
   std::swap( v_[15], v_[20] );
   std::swap( v_[16], v_[26] );
   std::swap( v_[17], v_[32] );
   std::swap( v_[22], v_[27] );
   std::swap( v_[23], v_[33] );
   std::swap( v_[29], v_[34] );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the matrix.
 *
 * \return Reference to the inverted matrix.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data types will result in a compile time error.
 */
template< typename Type >  // Data type of the matrix
inline Matrix6x6<Type>& Matrix6x6<Type>::invert()
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the inverse of the matrix.
 *
 * \return The inverse of the matrix.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data types will result in a compile time error.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix6x6<Type> Matrix6x6<Type>::getInverse() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   return Matrix6x6();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the matrix by the scalar value \a scalar (\f$ A*=s \f$).
 *
 * \param scalar The scalar value for the matrix scaling.
 * \return Reference to the matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the scalar value
inline Matrix6x6<Type>& Matrix6x6<Type>::scale( Other scalar )
{
   for( size_t i=0; i<36; ++i )
      v_[i] *= scalar;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if the matrix is diagonal.
 *
 * \return \a true if the matrix is diagonal, \a false if not.
 *
 * This function tests whether the matrix is diagonal, i.e. if the non-diagonal elements are
 * default elements. In case of integral or floating point data types, a diagonal matrix has
 * the form

                          \f[\left(\begin{array}{*{6}{c}}
                          aa & 0  & 0  & 0  & 0  & 0  \\
                          0  & bb & 0  & 0  & 0  & 0  \\
                          0  & 0  & cc & 0  & 0  & 0  \\
                          0  & 0  & 0  & dd & 0  & 0  \\
                          0  & 0  & 0  & 0  & ee & 0  \\
                          0  & 0  & 0  & 0  & 0  & ff \\
                          \end{array}\right)\f]
 */
template< typename Type >  // Data type of the matrix
inline bool Matrix6x6<Type>::isDiagonal() const
{
   for( size_t i=0; i<5; ++i ) {
      for( size_t j=i+1; j<6; ++j ) {
         if( !isDefault( v_[i*6+j] ) || !isDefault( v_[j*6+i] ) )
            return false;
      }
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if the matrix is symmetric.
 *
 * \return \a true if the matrix is symmetric, \a false if not.
 */
template< typename Type >  // Data type of the matrix
inline bool Matrix6x6<Type>::isSymmetric() const
{
   for( size_t i=0; i<5; ++i ) {
      for( size_t j=i+1; j<6; ++j ) {
         if( !equal( v_[i*6+j], v_[j*6+i] ) )
            return false;
      }
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two 6x6 matrices.
 *
 * \param m The matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the matrix
inline void Matrix6x6<Type>::swap( Matrix6x6& m ) /* throw() */
{
   for( size_t i=0; i<36; ++i )
      std::swap( v_[i], m.v_[i] );
}
//*************************************************************************************************




//=================================================================================================
//
//  EXPRESSION TEMPLATE EVALUATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the matrix is aliased with the given address \a alias.
 *
 * \param alias The alias to be checked.
 * \return \a true in case the alias corresponds to this matrix, \a false if not.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the foreign expression
inline bool Matrix6x6<Type>::isAliased( const Other* alias ) const
{
   return static_cast<const void*>( this ) == static_cast<const void*>( alias );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be assigned.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void Matrix6x6<Type>::assign( const DenseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 6 && (~rhs).columns() == 6, "Invalid matrix size" );

   for( size_t i=0; i<6; ++i )
      for( size_t j=0; j<6; ++j )
         v_[i*6+j] = (~rhs)(i,j);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be assigned.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void Matrix6x6<Type>::assign( const SparseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 6 && (~rhs).columns() == 6, "Invalid matrix size" );

   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<6; ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*6+element->index()] = element->value();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be added.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void Matrix6x6<Type>::addAssign( const DenseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 6 && (~rhs).columns() == 6, "Invalid matrix size" );

   for( size_t i=0; i<6; ++i )
      for( size_t j=0; j<6; ++j )
         v_[i*6+j] += (~rhs)(i,j);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be added.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void Matrix6x6<Type>::addAssign( const SparseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 6 && (~rhs).columns() == 6, "Invalid matrix size" );

   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<6; ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*6+element->index()] += element->value();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be subtracted.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void Matrix6x6<Type>::subAssign( const DenseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 6 && (~rhs).columns() == 6, "Invalid matrix size" );

   for( size_t i=0; i<6; ++i )
      for( size_t j=0; j<6; ++j )
         v_[i*6+j] -= (~rhs)(i,j);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be subtracted.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void Matrix6x6<Type>::subAssign( const SparseMatrix<MT>& rhs )
{
   pe_INTERNAL_ASSERT( (~rhs).rows() == 6 && (~rhs).columns() == 6, "Invalid matrix size" );

   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<6; ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*6+element->index()] -= element->value();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Matrix6x6 operators */
//@{
template< typename T1, typename T2 >
inline bool operator==( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const Matrix6x6<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const Matrix6x6<T2>& mat );

template< typename T1, typename T2 >
inline bool operator!=( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const Matrix6x6<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const Matrix6x6<T2>& mat );

template< typename Type >
std::ostream& operator<<( std::ostream& os, const Matrix6x6<Type>& m );

template< typename Type >
inline bool isnan( const Matrix6x6<Type>& m );

template< typename Type >
inline const Matrix6x6<Type> abs( const Matrix6x6<Type>& m );

template< typename Type >
inline const Matrix6x6<Type> fabs( const Matrix6x6<Type>& m );

template< typename Type >
inline void reset( Matrix6x6<Type>& m );

template< typename Type >
inline void clear( Matrix6x6<Type>& m );

template< typename Type >
inline bool isDefault( const Matrix6x6<Type>& m );

template< typename Type >
inline const Matrix6x6<Type> trans( const Matrix6x6<Type>& m );

template< typename Type >
inline const Matrix6x6<Type> inv( const Matrix6x6<Type>& m );

template< typename Type >
inline const Matrix6x6<typename MathTrait<Type,Type>::MultType> sq( const Matrix6x6<Type>& m );

template< typename Type >
inline void swap( Matrix6x6<Type>& a, Matrix6x6<Type>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two matrices.
 * \ingroup dense_matrix_6x6
 *
 * \param lhs The left-hand side matrix for the comparison.
 * \param rhs The right-hand side matrix for the comparison.
 * \return \a true if the two matrices are equal, \a false if not.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline bool operator==( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs )
{
   // In order to compare the two matrices, the data values of the lower-order data
   // type are converted to the higher-order data type within the equal function.
   for( size_t i=0; i<36; ++i )
      if( !equal( lhs[i], rhs[i] ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a matrix and a scalar value.
 * \ingroup dense_matrix_6x6
 *
 * \param mat The left-hand side matrix for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if all elements of the matrix are equal to the scalar, \a false if not.
 *
 * If all values of the matrix are equal to the scalar value, the equality test returns true,
 * otherwise false.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const Matrix6x6<T1>& mat, T2 scalar )
{
   // In order to compare the matrix and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type.
   for( size_t i=0; i<36; ++i )
      if( !equal( mat[i], scalar ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a scalar value and a matrix.
 * \ingroup dense_matrix_6x6
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param mat The right-hand side matrix for the comparison.
 * \return \a true if all elements of the matrix are equal to the scalar, \a false if not.
 *
 * If all values of the matrix are equal to the scalar value, the equality test returns true,
 * otherwise false.
 */
template< typename T1    // Data type of the left-hand side scalar
        , typename T2 >  // Data type of the right-hand side matrix
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const Matrix6x6<T2>& mat )
{
   return ( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two matrices.
 * \ingroup dense_matrix_6x6
 *
 * \param lhs The left-hand side matrix for the comparison.
 * \param rhs The right-hand side matrix for the comparison.
 * \return \a true if the two matrices are not equal, \a false if they are equal.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline bool operator!=( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a matrix and a scalar value.
 * \ingroup dense_matrix_6x6
 *
 * \param mat The left-hand side matrix for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if at least one element of the matrix is different from the scalar, \a false if not.
 *
 * If one value of the matrix is inequal to the scalar value, the inequality test returns true,
 * otherwise false.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const Matrix6x6<T1>& mat, T2 scalar )
{
   return !( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a scalar value and a matrix.
 * \ingroup dense_matrix_6x6
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param mat The right-hand side matrix for the comparison.
 * \return \a true if at least one element of the matrix is different from the scalar, \a false if not.
 *
 * If one value of the matrix is inequal to the scalar value, the inequality test returns true,
 * otherwise false.
 */
template< typename T1    // Data type of the left-hand side scalar
        , typename T2 >  // Data type of the right-hand side matrix
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const Matrix6x6<T2>& mat )
{
   return !( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for 6x6 matrices.
 * \ingroup dense_matrix_6x6
 *
 * \param os Reference to the output stream.
 * \param m Reference to a constant matrix object.
 * \return Reference to the output stream.
 */
template< typename Type >  // Data type of the matrix
std::ostream& operator<<( std::ostream& os, const Matrix6x6<Type>& m )
{
   return os << " ( " << m[ 0] << " , " << m[ 1] << " , " << m[ 2] << " , " << m[ 3] << " , " << m[ 4] << " , " << m[ 5] << " )\n"
             << " ( " << m[ 6] << " , " << m[ 7] << " , " << m[ 8] << " , " << m[ 9] << " , " << m[10] << " , " << m[11] << " )\n"
             << " ( " << m[12] << " , " << m[13] << " , " << m[14] << " , " << m[15] << " , " << m[16] << " , " << m[17] << " )\n"
             << " ( " << m[18] << " , " << m[19] << " , " << m[20] << " , " << m[21] << " , " << m[22] << " , " << m[23] << " )\n"
             << " ( " << m[24] << " , " << m[25] << " , " << m[26] << " , " << m[27] << " , " << m[28] << " , " << m[29] << " )\n"
             << " ( " << m[30] << " , " << m[31] << " , " << m[32] << " , " << m[33] << " , " << m[34] << " , " << m[35] << " )\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks the given matrix for not-a-number elements.
 * \ingroup dense_matrix_6x6
 *
 * \param m The matrix to be checked for not-a-number elements.
 * \return \a true if at least one element of the matrix is not-a-number, \a false otherwise.
 */
template< typename Type >  // Data type of the matrix
inline bool isnan( const Matrix6x6<Type>& m )
{
   for( size_t i=0; i<36; ++i )
      if( isnan( m[i] ) ) return true;
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a matrix containing the absolute values of each single element of \a m.
 * \ingroup dense_matrix_6x6
 *
 * \param m The integral input matrix.
 * \return The absolute value of each single element of \a m.
 *
 * The \a abs function calculates the absolute value of each element of the input matrix \a m.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix6x6<Type> abs( const Matrix6x6<Type>& m )
{
   using std::abs;

   Matrix6x6<Type> tmp( m.size() );

   for( size_t i=0; i<36; ++i )
      tmp[i] = abs( m[i] );

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a matrix containing the absolute values of each single element of \a m.
 * \ingroup dense_matrix_6x6
 *
 * \param m The floating point input matrix.
 * \return The absolute value of each single element of \a m.
 *
 * The \a fabs function calculates the absolute value of each element of the input matrix \a m.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix6x6<Type> fabs( const Matrix6x6<Type>& m )
{
   using std::fabs;

   Matrix6x6<Type> tmp( m.size() );

   for( size_t i=0; i<36; ++i )
      tmp[i] = fabs( m[i] );

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given 6x6 matrix.
 * \ingroup dense_matrix_6x6
 *
 * \param m The matrix to be resetted.
 * \return void
 */
template< typename Type >  // Data type of the matrix
inline void reset( Matrix6x6<Type>& m )
{
   m.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given 6x6 matrix.
 * \ingroup dense_matrix_6x6
 *
 * \param m The matrix to be cleared.
 * \return void
 *
 * Clearing a 6x6 matrix is equivalent to resetting it via the reset() function.
 */
template< typename Type >  // Data type of the matrix
inline void clear( Matrix6x6<Type>& m )
{
   m.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given 6x6 matrix is in default state.
 * \ingroup dense_matrix_6x6
 *
 * \param m The matrix to be tested for its default state.
 * \return \a true in case the given matrix is component-wise zero, \a false otherwise.
 */
template< typename Type >  // Data type of the matrix
inline bool isDefault( const Matrix6x6<Type>& m )
{
   return isDefault( m[ 0] ) && isDefault( m[ 1] ) && isDefault( m[ 2] ) && isDefault( m[ 3] ) &&
          isDefault( m[ 4] ) && isDefault( m[ 5] ) && isDefault( m[ 6] ) && isDefault( m[ 7] ) &&
          isDefault( m[ 8] ) && isDefault( m[ 9] ) && isDefault( m[10] ) && isDefault( m[11] ) &&
          isDefault( m[12] ) && isDefault( m[13] ) && isDefault( m[14] ) && isDefault( m[15] ) &&
          isDefault( m[16] ) && isDefault( m[17] ) && isDefault( m[18] ) && isDefault( m[19] ) &&
          isDefault( m[20] ) && isDefault( m[21] ) && isDefault( m[22] ) && isDefault( m[23] ) &&
          isDefault( m[24] ) && isDefault( m[25] ) && isDefault( m[26] ) && isDefault( m[27] ) &&
          isDefault( m[28] ) && isDefault( m[29] ) && isDefault( m[30] ) && isDefault( m[31] ) &&
          isDefault( m[32] ) && isDefault( m[33] ) && isDefault( m[34] ) && isDefault( m[35] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the transpose of the matrix.
 * \ingroup dense_matrix_6x6
 *
 * \param m The matrix to be transposed.
 * \return The transpose of the matrix.
 *
 * This function returns the transpose of the given 6x6 matrix:

   \code
   pe::Mat6 A, B;
   // ... Initialization
   B = trans( A );
   \endcode
 */
template< typename Type >  // Data type of the matrix
inline const Matrix6x6<Type> trans( const Matrix6x6<Type>& m )
{
   Matrix6x6<Type> tmp;

   for( size_t i=0; i<6; ++i ) {
      tmp[i*6  ] = m[   i];
      tmp[i*6+1] = m[ 6+i];
      tmp[i*6+2] = m[12+i];
      tmp[i*6+3] = m[18+i];
      tmp[i*6+4] = m[24+i];
      tmp[i*6+5] = m[30+i];
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the given 6x6 matrix.
 * \ingroup dense_matrix_6x6
 *
 * \param m The matrix to be inverted.
 * \return The inverse of the matrix.
 *
 * This function returns the inverse of the given 6x6 matrix. It has the same effect as calling
 * the getInverse() member function of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline const Matrix6x6<Type> inv( const Matrix6x6<Type>& m )
{
   return m.getInverse();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Squaring the given 6x6 matrix.
 * \ingroup dense_matrix_6x6
 *
 * \param m The 6x6 matrix to be squared.
 * \return The result of the square operation.
 *
 * This function squares the given 6x6 matrix \a m. This function has the same effect as
 * multiplying the matrix with itself (\f$ m * m \f$).
 */
template< typename Type >  // Data type of the matrix
inline const Matrix6x6<typename MathTrait<Type,Type>::MultType> sq( const Matrix6x6<Type>& m )
{
   return m * m;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two 6x6 matrices.
 * \ingroup dense_matrix_6x6
 *
 * \param a The first matrix to be swapped.
 * \param b The second matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the matrix
inline void swap( Matrix6x6<Type>& a, Matrix6x6<Type>& b ) /* throw() */
{
   a.swap( b );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL UNARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Matrix6x6 unary arithmetic operators */
//@{
template< typename Type >
inline const Matrix6x6<Type> operator-( const Matrix6x6<Type>& m );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unary minus operator for the negation of a 6x6 matrix (\f$ A = -B \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param m The 6x6 matrix to be negated.
 * \return The negation of the matrix.
 *
 * This operator represents the negation of a 6x6 matrix:

   \code
   pe::Mat6 A, B;
   // ... Initialization
   B = -A;
   \endcode
 */
template< typename Type >  // Data type of the matrix
inline const Matrix6x6<Type> operator-( const Matrix6x6<Type>& m )
{
   Matrix6x6<Type> tmp;
   for( size_t i=0; i<36; ++i )
      tmp[i] = -m[i];
   return tmp;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Matrix6x6 binary arithmetic operators */
//@{
template< typename T1, typename T2 >
inline const Matrix6x6< typename MathTrait<T1,T2>::AddType >
   operator+( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix6x6< typename MathTrait<T1,T2>::SubType >
   operator-( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix6x6< typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType >
   operator*( const Matrix6x6<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline const Matrix6x6< typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType >
   operator*( T1 scalar, const Matrix6x6<T2>& mat );

template< typename T1, typename T2 >
inline const Vector6< typename MathTrait<T1,T2>::MultType, false >
   operator*( const Matrix6x6<T1>& lhs, const Vector6<T2,false>& rhs );

template< typename T1, typename T2 >
inline const Vector6< typename MathTrait<T1,T2>::MultType, true >
   operator*( const Vector6<T1,true>& lhs, const Matrix6x6<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix6x6< typename MathTrait<T1,T2>::MultType >
   operator*( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs );

template< typename T1, typename T2 >
inline const Matrix6x6< typename MathTrait< T1, T2, IsNumeric<T1>::value && IsNumeric<T2>::value >::DivType >
   operator/( const Matrix6x6<T1>& mat, T2 scalar );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition operator for the addition of two matrices (\f$ A=B+C \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param lhs The left-hand side matrix for the matrix addition.
 * \param rhs The right-hand side matrix to be added to the left-hand side matrix.
 * \return The sum of the two matrices.
 *
 * This operator represents the addition of two 6x6 matrices:

   \code
   pe::Mat6 A, B, C;
   // ... Initialization
   C = A + B;
   \endcode

 * The operator returns a 6x6 matrix of the higher-order element type of the two involved matrix
 * element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be supported by the
 * MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline const Matrix6x6< typename MathTrait<T1,T2>::AddType >
   operator+( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::AddType  AT;

   Matrix6x6<AT> tmp;
   for( size_t i=0; i<36; ++i )
      tmp[i] = lhs[i] + rhs[i];

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction operator for the subtraction of two matrices (\f$ A=B-C \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param lhs The left-hand side matrix for the matrix subtraction.
 * \param rhs The right-hand side matrix to be subtracted from the matrix.
 * \return The difference of the two matrices.
 *
 * This operator represents the subtraction of two 6x6 matrices:

   \code
   pe::Mat6 A, B, C;
   // ... Initialization
   C = A - B;
   \endcode

 * The operator returns a 6x6 matrix of the higher-order element type of the two involved matrix
 * element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be supported by the
 * MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline const Matrix6x6< typename MathTrait<T1,T2>::SubType >
   operator-( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::SubType  ST;

   Matrix6x6<ST> tmp;
   for( size_t i=0; i<36; ++i )
      tmp[i] = lhs[i] - rhs[i];

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a matrix and a scalar value
 *        (\f$ A=B*s \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param mat The left-hand side matrix for the multiplication.
 * \param scalar The right-hand side scalar value for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a 6x6 matrix and a scalar value:

   \code
   pe::Mat6 A, B;
   // ... Initialization
   B = A * 1.25;
   \endcode

 * The operator returns a 6x6 matrix of the higher-order element type of the involved data
 * types \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported by the MathTrait
 * class template. Note that this operator only works for scalar values of built-in data type.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline const Matrix6x6< typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType >
   operator*( const Matrix6x6<T1>& mat, T2 scalar )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;

   Matrix6x6<MT> tmp;

   for( size_t i=0; i<36; ++i )
      tmp[i] = scalar * mat[i];

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a scalar value and a matrix
 *        (\f$ A=s*B \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param scalar The left-hand side scalar value for the multiplication.
 * \param mat The right-hand side matrix for the multiplication.
 * \return The scaled result matrix.
 *
 * This operator represents the multiplication between a scalar value and a 6x6 matrix:

   \code
   pe::Mat6 A, B;
   // ... Initialization
   B = 1.25 * A;
   \endcode

 * The operator returns a 6x6 matrix of the higher-order element type of the involved data
 * types \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported by the MathTrait
 * class template. Note that this operator only works for scalar values of built-in data type.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline const Matrix6x6< typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType >
   operator*( T1 scalar, const Matrix6x6<T2>& mat )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;

   Matrix6x6<MT> tmp;

   for( size_t i=0; i<36; ++i )
      tmp[i] = scalar * mat[i];

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a matrix and a vector
 *        (\f$ \vec{a}=B*\vec{c} \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param lhs The left-hand side matrix for the multiplication.
 * \param rhs The right-hand side vector for the multiplication.
 * \return The resulting vector.
 *
 * This operator represents the multiplication between a 6x6 matrix and a 6D vector:

   \code
   pe::Mat6 A;
   pe::Vec6 x, y;
   // ... Initialization
   y = A * x;
   \endcode

 * The operator returns a 6D vector of the higher order element type of the two involved
 * element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be supported
 * by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side vector
inline const Vector6< typename MathTrait<T1,T2>::MultType, false >
   operator*( const Matrix6x6<T1>& lhs, const Vector6<T2,false>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;

   Vector6<MT,false> tmp;

   for( size_t i=0; i<6; ++i ) {
      tmp[0] += lhs[   i] * rhs[i];
      tmp[1] += lhs[ 6+i] * rhs[i];
      tmp[2] += lhs[12+i] * rhs[i];
      tmp[3] += lhs[18+i] * rhs[i];
      tmp[4] += lhs[24+i] * rhs[i];
      tmp[5] += lhs[30+i] * rhs[i];
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a vector and a matrix
 *        (\f$ \vec{a}^T=\vec{b}^T*C \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param lhs The left-hand side transpose vector for the multiplication.
 * \param rhs The right-hand side matrix for the multiplication.
 * \return The resulting transpose vector.
 *
 * This operator represents the multiplication between a transpose 6D vector and a 6x6
 * matrix:

   \code
   pe::Mat6 A;
   pe::Vec6T x, y;
   // ... Initialization
   y = x * A;
   \endcode

 * The operator returns a transpose 6D vector of the higher-order element type of the two
 * involved element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be
 * supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side vector
        , typename T2 >  // Data type of the right-hand side matrix
inline const Vector6< typename MathTrait<T1,T2>::MultType, true >
   operator*( const Vector6<T1,true>& lhs, const Matrix6x6<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;

   Vector6<MT,true> tmp;

   for( size_t i=0; i<6; ++i ) {
      tmp[0] += lhs[i] * rhs[i*6  ];
      tmp[1] += lhs[i] * rhs[i*6+1];
      tmp[2] += lhs[i] * rhs[i*6+2];
      tmp[3] += lhs[i] * rhs[i*6+3];
      tmp[4] += lhs[i] * rhs[i*6+4];
      tmp[5] += lhs[i] * rhs[i*6+5];
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of two matrices (\f$ A=B*C \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param lhs The left-hand side matirx for the multiplication.
 * \param rhs The right-hand side matrix for the multiplication.
 * \return The resulting matrix.
 *
 * This operator represents the multiplication of two 6x6 matrices:

   \code
   pe::Mat6 A, B, C;
   // ... Initialization
   C = A * B;
   \endcode

 * The operator returns a 6x6 matrix of the higher-order element type of the two involved
 * matrix element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be
 * supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side matrix
inline const Matrix6x6< typename MathTrait<T1,T2>::MultType >
   operator*( const Matrix6x6<T1>& lhs, const Matrix6x6<T2>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;

   Matrix6x6<MT> tmp;

   for( size_t i=0; i<6; ++i ) {
      for( size_t j=0; j<6; ++j ) {
         tmp[i*6  ] += lhs[i*6+j] * rhs[j*6  ];
         tmp[i*6+1] += lhs[i*6+j] * rhs[j*6+1];
         tmp[i*6+2] += lhs[i*6+j] * rhs[j*6+2];
         tmp[i*6+3] += lhs[i*6+j] * rhs[j*6+3];
         tmp[i*6+4] += lhs[i*6+j] * rhs[j*6+4];
         tmp[i*6+5] += lhs[i*6+j] * rhs[j*6+5];
      }
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division operator for the divison of a matrix by a scalar value
 *        (\f$ \vec{a}=\vec{b}/s \f$).
 * \ingroup dense_matrix_6x6
 *
 * \param mat The left-hand side matrix for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result matrix.
 *
 * This operator represents the division of a 6x6 matrix by a scalar value:

   \code
   pe::Mat6 A, B;
   // ... Initialization
   B = A / 0.24;
   \endcode

 * The operator returns a 6x6 matrix of the higher-order element type of the involved data types
 * \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported by the MathTrait class
 * template. Note that this operator is only selected in case a 6x6 matrix with either integral
 * or floating point data elements is divided by a scalar value of built-in data type.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename T1    // Data type of the left-hand side matrix
        , typename T2 >  // Data type of the right-hand side scalar
inline const Matrix6x6< typename MathTrait< T1, T2, IsNumeric<T1>::value && IsNumeric<T2>::value >::DivType >
   operator/( const Matrix6x6<T1>& mat, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   typedef typename MathTrait<T1,T2>::DivType  DT;

   Matrix6x6<DT> tmp;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT idiv( DT(1)/static_cast<DT>( scalar ) );
      for( size_t i=0; i<36; ++i )
         tmp[i] = mat[i] * idiv;
   }
   else {
      for( size_t i=0; i<36; ++i )
         tmp[i] = mat[i] / scalar;
   }

   return tmp;
}
//*************************************************************************************************




//=================================================================================================
//
//  MATHTRAIT SPECIALIZATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
template< typename T1, typename T2 >
struct MathTrait< Matrix6x6<T1>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::DivType  >  DivType;
};

template< typename T1, typename T2 >
struct MathTrait< T1, Matrix6x6<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix6x6<T1>, Vector6<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector6<T1,true>, Matrix6x6<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix6x6<T1>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, Matrix6x6<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix6x6<T1>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, Matrix6x6<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix6x6<T1>, Matrix6x6<T2> >
{
   typedef Matrix6x6< typename MathTrait<T1,T2>::HighType >  HighType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::LowType  >  LowType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief 6x6 real matrix.
 * \ingroup dense_matrix_6x6
 */
typedef Matrix6x6<real>  Mat6;
//*************************************************************************************************

} // namespace pe

#endif
