//=================================================================================================
/*!
 *  \file pe/math/DiagonalMatrixN.h
 *  \brief Implementation of a diagonal matrix
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

#ifndef _PE_MATH_DIAGONALMATRIXN_H_
#define _PE_MATH_DIAGONALMATRIXN_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <vector>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/DiagonalMatrix.h>
#include <pe/system/Precision.h>
#include <pe/system/Restrict.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Const.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/constraints/SameSize.h>
#include <pe/util/constraints/Volatile.h>
#include <pe/util/EnableIf.h>
#include <pe/util/Null.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsFloatingPoint.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename > class VectorN;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup diagonal_matrix_N DiagonalMatrixN
 * \ingroup sparse_matrix
 */
/*!\brief Implementation of a square diagonal matrix of size \f$ N \f$.
 * \ingroup diagonal_matrix_N
 *
 * The DiagonalMatrixN class is the representation of a \f$ N \times N \f$ matrix with nonzero
 * elements located only on the diagonal.

 * Inserting/accessing elements in a diagonal matrix can be done by the subscript operator.

 * DiagonalMatrixN can be used with any non-cv-qualified element type. The arithmetic operators
 * for matrix/matrix, matrix/vector and matrix/element operations with the same element type work
 * for any element type as long as the element type supports the arithmetic operation. Arithmetic
 * operations between matrices, vectors and elements of different element types are only supported
 * for all data types supported by the MathTrait class template (for details see the MathTrait
 * class description).

   \code
   DiagonalMatrixN< double > a, b, c;
   DiagonalMatrixN< float  > d;
   DiagonalMatrixN< std::complex<double> > e, f, g;
   DiagonalMatrixN< std::complex<float>  > h;

   ...         // Appropriate resizing

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type >  // Data type of the sparse matrix
class DiagonalMatrixN : public DiagonalMatrix< DiagonalMatrixN<Type> >
{
public:
   //**Type definitions****************************************************************************
   typedef DiagonalMatrixN         ResultType;     //!< Result type for expression template evaluations.
   typedef const DiagonalMatrixN&  CompositeType;  //!< Data type for composite expression templates.
   typedef Type                    ElementType;    //!< Type of the diagonal matrix elements.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                           explicit inline DiagonalMatrixN();
                           explicit inline DiagonalMatrixN( size_t n );
                           explicit inline DiagonalMatrixN( size_t n, Type init );
                                    inline DiagonalMatrixN( const DiagonalMatrixN& dm );
   template< typename VT > explicit inline DiagonalMatrixN( const VectorN<VT>& dm );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~DiagonalMatrixN();
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
                           inline DiagonalMatrixN&      operator= ( const DiagonalMatrixN&    rhs );
   template< typename MT > inline DiagonalMatrixN&      operator= ( const DiagonalMatrix<MT>& rhs );
                           inline Type&                 operator[]( size_t i );
                           inline const Type&           operator[]( size_t i ) const;
                           inline const DiagonalMatrixN operator- () const;
   template< typename MT > inline DiagonalMatrixN&      operator+=( const DiagonalMatrix<MT>& rhs );
   template< typename MT > inline DiagonalMatrixN&      operator-=( const DiagonalMatrix<MT>& rhs );
   template< typename MT > inline DiagonalMatrixN&      operator*=( const DiagonalMatrix<MT>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, DiagonalMatrixN >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, DiagonalMatrixN >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t           rows()               const;
                              inline size_t           columns()            const;
                              inline size_t           size()               const;
                              inline size_t           capacity()           const;
                              inline size_t           nonZeros()           const;
                              inline void             reset();
                              inline void             clear();
                                     void             resize ( size_t n, bool preserve=true );
                              inline void             reserve( size_t n );
                              inline bool             isDiagonal() const;
                              inline bool             isSymmetric() const;
   template< typename Other > inline DiagonalMatrixN& scale( Other scalar );
   template< typename Other > inline DiagonalMatrixN& scaleDiagonal( Other scalar );
                              inline void             swap( DiagonalMatrixN& sm ) /* throw() */;
   //@}
   //**********************************************************************************************

   //**Expression template evaluation functions****************************************************
   /*!\name Expression template evaluation functions */
   //@{
   template< typename Other > inline bool isAliased( const Other* alias ) const;
   template< typename MT >    inline void assign   ( const DiagonalMatrix<MT>& rhs );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t size_;           //!< The current size/dimension of the diagonal matrix.
   size_t capacity_;       //!< The maximum capacity of the diagonal matrix.
   Type* pe_RESTRICT v_;   //!< The dynamically allocated diagonal matrix elements.
                           /*!< Access to the diagonal elements is gained via the subscript operator.
                                The order of the elements is
                                \f[\left(\begin{array}{*{5}{c}}
                                0 & 1 & 2 & \cdots & N-1 \\
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
/*!\brief The default constructor for DiagonalMatrixN.
 */
template< typename Type >  // Data type of the diagonal matrix
inline DiagonalMatrixN<Type>::DiagonalMatrixN()
   : size_    ( 0 )  // The current size of the diagonal matrix
   , capacity_( 0 )  // The current capacity of the pointer array
   , v_( NULL )      // The matrix elements
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a diagonal matrix of size \f$ N \times N \f$.
 *
 * \param n The number of rows and columns of the matrix.
 *
 * \b Note: This constructor is only responsible to allocate the required dynamic memory. No
 *          element initialization is performed!
 */
template< typename Type >  // Data type of the diagonal matrix
inline DiagonalMatrixN<Type>::DiagonalMatrixN( size_t n )
   : size_    ( n )     // The current size of the diagonal matrix
   , capacity_( n )     // The current capacity of the pointer array
   , v_( new Type[n] )  // The matrix elements
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a diagonal matrix of size \f$ N \times N \f$ with homogeneous
 *        initialization.
 *
 * \param n The number of rows and columns of the matrix.
 * \param init The initial value of the diagonal elements.
 *
 * All diagonal elements are initialized with the specified value.
 */
template< typename Type >  // Data type of the diagonal matrix
inline DiagonalMatrixN<Type>::DiagonalMatrixN( size_t n, Type init )
   : size_    ( n )     // The current size of the diagonal matrix
   , capacity_( n )     // The current capacity of the pointer array
   , v_( new Type[n] )  // The matrix elements
{
   for( size_t i=0; i<size_; ++i )
      v_[i] = init;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for DiagonalMatrixN.
 *
 * \param dm Diagonal matrix to be copied.
 */
template< typename Type >  // Data type of the diagonal matrix
inline DiagonalMatrixN<Type>::DiagonalMatrixN( const DiagonalMatrixN& dm )
   : size_    ( dm.size_ )      // The current size of the diagonal matrix
   , capacity_( dm.capacity_ )  // The current capacity of the pointer array
   , v_( new Type[size_] )      // The matrix elements
{
   for( size_t i=0; i<size_; ++i )
      v_[i] = dm.v_[i];
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for DiagonalMatrixN.
 */
template< typename Type >  // Data type of the diagonal matrix
inline DiagonalMatrixN<Type>::~DiagonalMatrixN()
{
   delete [] v_;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for DiagonalMatrixN.
 *
 * \param rhs Diagonal matrix to be copied.
 * \return Reference to the assigned diagonal matrix.
 *
 * The diagonal matrix is resized according to the given diagonal matrix and initialized as a
 * copy of this matrix.
 */
template< typename Type >  // Data type of the diagonal matrix
inline DiagonalMatrixN<Type>& DiagonalMatrixN<Type>::operator=( const DiagonalMatrixN& rhs )
{
   if( &rhs == this ) return *this;

   resize( rhs.size_, false );

   for( size_t i=0; i<size_; ++i )
      v_[i] = rhs.v_[i];

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the diagonal elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the vector
inline Type& DiagonalMatrixN<Type>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < size_, "Invalid vector access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the diagonal elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the vector
inline const Type& DiagonalMatrixN<Type>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < size_, "Invalid vector access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unary minus operator for the negation of a diagonal matrix (\f$ A = -B \f$).
 *
 * \return The negated diagonal matrix.
 */
template< typename Type >  // Data type of the diagonal matrix
inline const DiagonalMatrixN<Type> DiagonalMatrixN<Type>::operator-() const
{
   DiagonalMatrixN tmp( *this );

   for( size_t i=0; i<tmp.size_; ++i ) {
      v_[i] = -v_[i];
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a diagonal matrix (\f$ A+=B \f$).
 *
 * \param rhs The right-hand side diagonal matrix to be added to the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side diagonal matrix
inline DiagonalMatrixN<Type>& DiagonalMatrixN<Type>::operator+=( const DiagonalMatrix<MT>& rhs )
{
   DiagonalMatrixN tmp( *this + rhs );
   swap( tmp );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a diagonal matrix (\f$ A-=B \f$).
 *
 * \param rhs The right-hand side diagonal matrix to be subtracted from the matrix.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two matrices don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side diagonal matrix
inline DiagonalMatrixN<Type>& DiagonalMatrixN<Type>::operator-=( const DiagonalMatrix<MT>& rhs )
{
   DiagonalMatrixN tmp( *this - rhs );
   swap( tmp );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a diagonal matrix
 * \brief (\f$ A*=B \f$).
 *
 * \param rhs The right-hand side diagonal matrix for the multiplication.
 * \return Reference to the matrix.
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * In case the current sizes of the two given matrices don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side diagonal matrix
inline DiagonalMatrixN<Type>& DiagonalMatrixN<Type>::operator*=( const DiagonalMatrix<MT>& rhs )
{
   DiagonalMatrixN tmp( *this * rhs );
   swap( tmp );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication between a diagonal matrix and
 * \brief a scalar value (\f$ A*=s \f$).
 *
 * \param rhs The right-hand side scalar value for the multiplication.
 * \return Reference to the diagonal matrix.
 */
template< typename Type >   // Data type of the diagonal matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, DiagonalMatrixN<Type> >::Type&
   DiagonalMatrixN<Type>::operator*=( Other rhs )
{
   for( size_t i=0; i<size_; ++i ) {
      v_[i] *= rhs;
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division assignment operator for the division of a diagonal matrix by a scalar value
 * \brief (\f$ A/=s \f$).
 *
 * \param rhs The right-hand side scalar value for the division.
 * \return Reference to the matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, DiagonalMatrixN<Type> >::Type&
   DiagonalMatrixN<Type>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      for( size_t i=0; i<size_; ++i ) {
         v_[i] = static_cast<Type>( static_cast<DT>( v_[i] ) * tmp );
      }
   }
   else {
      for( size_t i=0; i<size_; ++i ) {
         v_[i] /= rhs;
      }
   }

   return *this;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the current number of rows of the diagonal matrix.
 *
 * \return The number of rows of the diagonal matrix.
 *
 * Since the diagonal matrix is square, the number of rows and columns is always identical.
 */
template< typename Type >  // Data type of the diagonal matrix
inline size_t DiagonalMatrixN<Type>::rows() const
{
   return size_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current number of columns of the diagonal matrix.
 *
 * \return The number of columns of the diagonal matrix.
 *
 * Since the diagonal matrix is square, the number of rows and columns is always identical.
 */
template< typename Type >  // Data type of the diagonal matrix
inline size_t DiagonalMatrixN<Type>::columns() const
{
   return size_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current size of the diagonal matrix.
 *
 * \return The number of rows and columns respectively of the diagonal matrix.
 */
template< typename Type >  // Data type of the diagonal matrix
inline size_t DiagonalMatrixN<Type>::size() const
{
   return size_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum capacity of the diagonal matrix.
 *
 * \return The capacity of the diagonal matrix.
 */
template< typename Type >  // Data type of the diagonal matrix
inline size_t DiagonalMatrixN<Type>::capacity() const
{
   return capacity_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of non-zero elements in the diagonal matrix
 *
 * \return The number of non-zero elements in the diagonal matrix.
 */
template< typename Type >  // Data type of the diagonal matrix
inline size_t DiagonalMatrixN<Type>::nonZeros() const
{
   size_t nonzeros( 0 );

   for( size_t i=0; i<size_; ++i ) {
      if( v_[i] != Type() )
         ++nonzeros;
   }

   return nonzeros;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
template< typename Type >  // Data type of the diagonal matrix
inline void DiagonalMatrixN<Type>::reset()
{
   using pe::reset;
   for( size_t i=0; i<size_; ++i )
      reset( v_[i] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the diagonal matrix.
 *
 * \return void
 *
 * After the clear() function, the size of the diagonal matrix is 0.
 */
template< typename Type >  // Data type of the diagonal matrix
inline void DiagonalMatrixN<Type>::clear()
{
   size_ = 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the size of the diagonal matrix.
 *
 * \param n The new size of the diagonal matrix.
 * \param preserve \a true if the old values of the vector should be preserved, \a false if not.
 * \return void
 *
 * This function resizes the diagonal matrix using the given size to \a n. During this operation,
 * new dynamic memory may be allocated in case the capacity of the matrix is too small. Therefore
 * this function potentially changes all diagonal elements. In order to preserve the old diagonal
 * values, the \a preserve flag can be set to \a true. However, new diagonal elements are not
 * initialized!\n
 */
template< typename Type >  // Data type of the vector
inline void DiagonalMatrixN<Type>::resize( size_t n, bool preserve )
{
   if( n == size_ ) return;

   if( preserve )
   {
      Type* pe_RESTRICT v = new Type[n];
      const size_t minsize( pe::min( n, size_ ) );

      for( size_t i=0; i<minsize; ++i )
         v[i] = v_[i];

      std::swap( v_, v );
      delete [] v;
      capacity_ = n;
   }
   else if( n > capacity_ ) {
      Type* pe_RESTRICT v = new Type[n];
      std::swap( v_, v );
      delete [] v;
      capacity_ = n;
   }

   size_ = n;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of the diagonal matrix.
 *
 * \param n The new minimum capacity of the diagonal matrix.
 * \return void
 *
 * This function increases the capacity of the diagonal matrix to at least \a n elements.
 * The current values of the diagonal elements are preserved.
 */
template< typename Type >  // Data type of the diagonal matrix
void DiagonalMatrixN<Type>::reserve( size_t n )
{
   if( n > capacity_ )
   {
      // Allocating a new array
      Type* pe_RESTRICT tmp = new Type[n];

      // Replacing the old array
      std::copy( v_, v_+size_, tmp );
      std::swap( tmp, v_ );
      capacity_ = n;
      delete [] tmp;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if the matrix is diagonal.
 *
 * \return \a true if the matrix is diagonal, \a false if not.
 *
 * This function unconditionally returns \a true for diagonal matrices.
 */
template< typename Type >  // Data type of the diagonal matrix
inline bool DiagonalMatrixN<Type>::isDiagonal() const
{
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if the matrix is symmetric.
 *
 * \return \a true if the matrix is symmetric, \a false if not.
 *
 * This function unconditionally returns \a true for diagonal matrices.
 */
template< typename Type >  // Data type of the matrix
inline bool DiagonalMatrixN<Type>::isSymmetric() const
{
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the diagonal matrix by the scalar value \a scalar (\f$ A=B*s \f$).
 *
 * \param scalar The scalar value for the matrix scaling.
 * \return Reference to the diagonal matrix.
 */
template< typename Type >   // Data type of the diagonal matrix
template< typename Other >  // Data type of the scalar value
inline DiagonalMatrixN<Type>& DiagonalMatrixN<Type>::scale( Other scalar )
{
   return *this * scalar;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling the diagonal matrix by the scalar value \a scalar.
 *
 * \param scalar The scalar value for the matrix scaling.
 * \return Reference to the diagonal matrix.
 */
template< typename Type >   // Data type of the diagonal matrix
template< typename Other >  // Data type of the scalar value
inline DiagonalMatrixN<Type>& DiagonalMatrixN<Type>::scaleDiagonal( Other scalar )
{
   return *this * scalar;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two sparse matrices.
 *
 * \param dm The diagonal matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the diagonal matrix
inline void DiagonalMatrixN<Type>::swap( DiagonalMatrixN& dm ) /* throw() */
{
   std::swap( size_, dm.size_ );
   std::swap( capacity_, dm.capacity_ );
   std::swap( v_, dm.v_ );
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
template< typename Type >   // Data type of the diagonal matrix
template< typename Other >  // Data type of the foreign expression
inline bool DiagonalMatrixN<Type>::isAliased( const Other* alias ) const
{
   return static_cast<const void*>( this ) == static_cast<const void*>( alias );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a diagonal matrix.
 *
 * \param rhs The right-hand side diagonal matrix to be assigned.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the diagonal matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void DiagonalMatrixN<Type>::assign( const DiagonalMatrix<MT>& rhs )
{
   for( size_t i=0; i<(~rhs).size(); ++i ) {
      v_[i] = (~rhs)[i];
   }
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
template< typename Type >
inline bool isnan( const DiagonalMatrixN<Type>& m );

template< typename Type >
inline void reset( DiagonalMatrixN<Type>& m );

template< typename Type >
inline bool isDefault( const DiagonalMatrixN<Type>& m );

template< typename Type >
inline void swap( DiagonalMatrixN<Type>& a, DiagonalMatrixN<Type>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks the given matrix for not-a-number elements.
 * \ingroup dense_matrix_MxN
 *
 * \param m The matrix to be checked for not-a-number elements.
 * \return \a true if at least one element of the matrix is not-a-number, \a false otherwise.
 */
template< typename Type >  // Data type of the matrix
inline bool isnan( const DiagonalMatrix<Type>& m )
{
   for( size_t i=0; i<m.rows(); ++i ) {
      if( isnan( m[i] ) ) return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given diagonal matrix.
 * \ingroup diagonal_matrix_N
 *
 * \param m The diagonal matrix to be resetted.
 * \return void
 */
template< typename Type >  // Data type of the diagonal matrix
inline void reset( DiagonalMatrixN<Type>& m )
{
   m.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given diagonal matrix is in default state.
 * \ingroup diagonal_matrix_N
 *
 * \param m The diagonal matrix to be tested for its default state.
 * \return \a true in case the given matrix is component-wise zero, \a false otherwise.
 */
template< typename Type >  // Data type of the diagonal matrix
inline bool isDefault( const DiagonalMatrixN<Type>& m )
{
   for( size_t i=0; i<m.size(); ++i )
      if( !isDefault( m[i] ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two diagonal matrices.
 * \ingroup diagonal_matrix_N
 *
 * \param a The first diagonal matrix to be swapped.
 * \param b The second diagonal matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the sparse matrices
inline void swap( DiagonalMatrixN<Type>& a, DiagonalMatrixN<Type>& b ) /* throw() */
{
   a.swap( b );
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
struct MathTrait< DiagonalMatrixN<T1>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::DivType >   DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T2 );
};

template< typename T1, typename T2 >
struct MathTrait< T1, DiagonalMatrixN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T1 );
};

template< typename T1, typename T2 >
struct MathTrait< DiagonalMatrixN<T1>, VectorN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType >          MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2 >
struct MathTrait< DiagonalMatrixN<T1>, DiagonalMatrixN<T2> >
{
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::HighType >  HighType;
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::LowType  >  LowType;
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef DiagonalMatrixN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Diagonal NxN real matrix.
 * \ingroup diagonal_matrix_N
 */
typedef DiagonalMatrixN<real>  DiagN;
//*************************************************************************************************

} // namespace pe

#endif
