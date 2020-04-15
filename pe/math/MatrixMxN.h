//=================================================================================================
/*!
 *  \file pe/math/MatrixMxN.h
 *  \brief Implementation of a dynamic MxN matrix
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

#ifndef _PE_MATH_MATRIXMXN_H_
#define _PE_MATH_MATRIXMXN_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <stdexcept>
#include <pe/math/DenseMatrix.h>
#include <pe/math/Functions.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/VectorN.h>
#include <pe/system/Precision.h>
#include <pe/system/Restrict.h>
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

template< typename >       class  Matrix2x2;
template< typename >       class  Matrix3x3;
template< typename >       class  Matrix6x6;
template< typename >       struct SparseMatrix;
template< typename, bool > class  SparseVectorN;
template< typename, bool > class  Vector2;
template< typename, bool > class  Vector3;
template< typename, bool > class  Vector6;
template< typename, bool > class  VectorN;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup dense_matrix_MxN MatrixMxN
 * \ingroup dense_matrix
 */
/*!\brief Efficient implementation of a \f$ M \times N \f$ matrix.
 * \ingroup dense_matrix_MxN
 *
 * The MatrixMxN class is the representation of a dynamic \f$ M \times N \f$ matrix with a total
 * of \f$ M \cdot N \f$ dynamically allocated elements. These elements can be directly accessed
 * with the 1D subscript operator or with the 2D function operator. The matrix is stored in a
 * row-wise fashion:

               \f[\left(\begin{array}{*{5}{c}}
               0           & 1             & 2             & \cdots & N-1         \\
               N           & N+1           & N+2           & \cdots & 2 \cdot N-1 \\
               \vdots      & \vdots        & \vdots        & \ddots & \vdots      \\
               M \cdot N-N & M \cdot N-N+1 & M \cdot N-N+2 & \cdots & M \cdot N-1 \\
               \end{array}\right)\f]

 * MatrixMxN can be used with any non-cv-qualified element type. The arithmetic operators for
 * matrix/matrix, matrix/vector and matrix/element operations with the same element type work
 * for any element type as long as the element type supports the arithmetic operation. Arithmetic
 * operations between matrices, vectors and elements of different element types are only supported
 * for all data types supported by the MathTrait class template (for details see the MathTrait
 * class description).

   \code
   MatrixMxN< double > a, b, c;
   MatrixMxN< float  > d;
   MatrixMxN< std::complex<double> > e, f, g;
   MatrixMxN< std::complex<float>  > h;

   ...         // Appropriate resizing

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type >  // Data type of the matrix
class MatrixMxN : public DenseMatrix< MatrixMxN<Type> >
{
public:
   //**Type definitions****************************************************************************
   typedef MatrixMxN<Type>   This;           //!< Type of this MatrixMxN instance.
   typedef This              ResultType;     //!< Result type for expression template evaluations.
   typedef Type              ElementType;    //!< Type of the matrix elements.
   typedef const MatrixMxN&  CompositeType;  //!< Data type for composite expression templates.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                           explicit inline MatrixMxN();
                           explicit inline MatrixMxN( size_t m, size_t n );
                           explicit inline MatrixMxN( size_t m, size_t n, Type init );
                                    inline MatrixMxN( const MatrixMxN& m );
   template< typename MT >          inline MatrixMxN( const DenseMatrix<MT>&  dm );
   template< typename MT >          inline MatrixMxN( const SparseMatrix<MT>& sm );

   template< typename Other, size_t M, size_t N >
   inline MatrixMxN( const Other (&rhs)[M][N] );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~MatrixMxN();
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   template< typename Other, size_t M, size_t N >
   inline MatrixMxN& operator=( const Other (&rhs)[M][N] );

                           inline MatrixMxN&  operator= ( Type set );
                           inline MatrixMxN&  operator= ( const MatrixMxN& set );
   template< typename MT > inline MatrixMxN&  operator= ( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline MatrixMxN&  operator= ( const SparseMatrix<MT>& rhs );
                           inline Type&       operator[]( size_t index );
                           inline const Type& operator[]( size_t index )       const;
                           inline Type&       operator()( size_t i, size_t j );
                           inline const Type& operator()( size_t i, size_t j ) const;
   template< typename MT > inline MatrixMxN&  operator+=( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline MatrixMxN&  operator+=( const SparseMatrix<MT>& rhs );
   template< typename MT > inline MatrixMxN&  operator-=( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline MatrixMxN&  operator-=( const SparseMatrix<MT>& rhs );
   template< typename MT > inline MatrixMxN&  operator*=( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline MatrixMxN&  operator*=( const SparseMatrix<MT>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, MatrixMxN >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, MatrixMxN >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t          rows()               const;
                              inline size_t          columns()            const;
                              inline size_t          capacity()           const;
                              inline size_t          nonZeros()           const;
                              inline size_t          nonZeros( size_t i ) const;
                              inline void            reset();
                              inline void            clear();
                                     void            resize ( size_t m, size_t n, bool preserve=true );
                              inline void            extend ( size_t m, size_t n, bool preserve=true );
                              inline void            reserve( size_t elements );
                              inline MatrixMxN&      transpose();
                              inline MatrixMxN&      invert();
                              inline const MatrixMxN getInverse()         const;
                              inline bool            isDiagonal()         const;
                              inline bool            isSymmetric()        const;
   template< typename Other > inline MatrixMxN&      scale( Other scalar );
                              inline void            swap( MatrixMxN& m ) /* throw() */;
   //@}
   //**********************************************************************************************

   //**Expression template evaluation functions****************************************************
   /*!\name Expression template evaluation functions */
   //@{
   template< typename Other > inline bool isAliased ( const Other* alias ) const;
   template< typename MT >    inline void assign    ( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void assign    ( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline void addAssign ( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void addAssign ( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline void subAssign ( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void subAssign ( const SparseMatrix<MT>& rhs );
   template< typename MT >    inline void multAssign( const DenseMatrix<MT>&  rhs );
   template< typename MT >    inline void multAssign( const SparseMatrix<MT>& rhs );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t m_;             //!< The current number of rows of the matrix.
   size_t n_;             //!< The current number of columns of the matrix.
   size_t capacity_;      //!< The maximum capacity of the matrix.
   Type* pe_RESTRICT v_;  //!< The dynamically allocated matrix elements.
                          /*!< Access to the matrix elements is gained via the subscript or
                               function call operator. The order of the elements is
                               \f[\left(\begin{array}{*{5}{c}}
                               0            & 1             & 2             & \cdots & N-1         \\
                               N            & N+1           & N+2           & \cdots & 2 \cdot N-1 \\
                               \vdots       & \vdots        & \vdots        & \ddots & \vdots      \\
                               M \cdot N-N  & M \cdot N-N+1 & M \cdot N-N+2 & \cdots & M \cdot N-1 \\
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
/*!\brief The default constructor for MatrixMxN.
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>::MatrixMxN()
   : m_       ( 0 )  // The current number of rows of the matrix
   , n_       ( 0 )  // The current number of columns of the matrix
   , capacity_( 0 )  // The maximum capacity of the matrix
   , v_       ( 0 )  // The matrix elements
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a matrix of size \f$ m \times n \f$. No element initialization is performed!
 *
 * \param m The number of rows of the matrix.
 * \param n The number of columns of the matrix.
 *
 * \b Note: This constructor is only responsible to allocate the required dynamic memory. No
 *          element initialization is performed!
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>::MatrixMxN( size_t m, size_t n )
   : m_       ( m )                    // The current number of rows of the matrix
   , n_       ( n )                    // The current number of columns of the matrix
   , capacity_( m*n )                  // The maximum capacity of the matrix
   , v_       ( new Type[capacity_] )  // The matrix elements
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a homogenous initialization of all \f$ m \times n \f$ matrix elements.
 *
 * \param m The number of rows of the matrix.
 * \param n The number of columns of the matrix.
 * \param init The initial value of the matrix elements.
 *
 * All matrix elements are initialized with the specified value.
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>::MatrixMxN( size_t m, size_t n, Type init )
   : m_       ( m )                    // The current number of rows of the matrix
   , n_       ( n )                    // The current number of columns of the matrix
   , capacity_( m*n )                  // The maximum capacity of the matrix
   , v_       ( new Type[capacity_] )  // The matrix elements
{
   for( size_t i=0; i<capacity_; ++i )
      v_[i] = init;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for MatrixMxN.
 *
 * \param m Matrix to be copied.
 *
 * The copy constructor is explicitly defined due to the required dynamic memory management
 * and in order to enable/facilitate NRV optimization.
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>::MatrixMxN( const MatrixMxN& m )
   : m_       ( m.m_  )                // The current number of rows of the matrix
   , n_       ( m.n_  )                // The current number of columns of the matrix
   , capacity_( m_*n_ )                // The maximum capacity of the matrix
   , v_       ( new Type[capacity_] )  // The matrix elements
{
   for( size_t i=0; i<capacity_; ++i )
      v_[i] = m.v_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different dense matrices.
 *
 * \param dm Dense matrix to be copied.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the foreign dense matrix
inline MatrixMxN<Type>::MatrixMxN( const DenseMatrix<MT>& dm )
   : m_       ( (~dm).rows() )         // The current number of rows of the matrix
   , n_       ( (~dm).columns() )      // The current number of columns of the matrix
   , capacity_( m_*n_ )                // The maximum capacity of the matrix
   , v_       ( new Type[capacity_] )  // The matrix elements
{
   using pe::assign;
   assign( *this, ~dm );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from sparse matrices.
 *
 * \param sm Sparse matrix to be copied.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the foreign sparse matrix
inline MatrixMxN<Type>::MatrixMxN( const SparseMatrix<MT>& sm )
   : m_       ( (~sm).rows() )         // The current number of rows of the matrix
   , n_       ( (~sm).columns() )      // The current number of columns of the matrix
   , capacity_( m_*n_ )                // The maximum capacity of the matrix
   , v_       ( new Type[capacity_] )  // The matrix elements
{
   using pe::assign;

   if( IsBuiltin<Type>::value )
      reset();

   assign( *this, ~sm );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Array initialization of all matrix elements.
 *
 * \param rhs \f$ M \times N \f$ dimensional array for the initialization.
 * \return Reference to the assigned matrix.
 *
 * This constructor offers the option to directly initialize the elements of the matrix:

   \code
   const real init[3][3] = { { 1, 2, 3 },
                             { 4, 5 },
                             { 7, 8, 9 } };
   MatrixMxN<real> A = init;
   \endcode

 * The matrix is sized accoring to the size of the array and initialized with the given values.
 * Missing values are initialized with zero (as e.g. the value 6 in the example).
 */
template< typename Type >  // Data type of the matrix
template< typename Other   // Data type of the initialization array
        , size_t M         // Number of rows of the initialization array
        , size_t N >       // Number of columns of the initialization array
inline MatrixMxN<Type>::MatrixMxN( const Other (&rhs)[M][N] )
   : m_       ( M )              // The current number of rows of the matrix
   , n_       ( N )              // The current number of columns of the matrix
   , capacity_( M*N )            // The maximum capacity of the matrix
   , v_       ( new Type[M*N] )  // The matrix elements
{
   for( size_t i=0; i<M; ++i )
      for( size_t j=0; j<N; ++j )
         v_[i*N+j] = rhs[i][j];
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for MatrixMxN.
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>::~MatrixMxN()
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
/*!\brief Array assignment to all matrix elements.
 *
 * \param rhs \f$ M \times N \f$ dimensional array for the assignment.
 * \return Reference to the assigned matrix.
 *
 * This assignment operator offers the option to directly set all elements of the matrix:

   \code
   const real init[3][3] = { { 1, 2, 3 },
                             { 4, 5 },
                             { 7, 8, 9 } };
   MatrixMxN<real> A;
   A = init;
   \endcode

 * The matrix is resized accoring to the size of the array and initialized with the given values.
 * Missing values are initialized with zero (as e.g. the value 6 in the example).
 */
template< typename Type >  // Data type of the matrix
template< typename Other   // Data type of the initialization array
        , size_t M         // Number of rows of the initialization array
        , size_t N >       // Number of columns of the initialization array
inline MatrixMxN<Type>& MatrixMxN<Type>::operator=( const Other (&rhs)[M][N] )
{
   resize( M, N, false );

   for( size_t i=0; i<M; ++i )
      for( size_t j=0; j<N; ++j )
         v_[i*N+j] = rhs[i][j];

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Homogenous assignment to all matrix elements.
 *
 * \param rhs Scalar value to be assigned to all matrix elements.
 * \return Reference to the assigned matrix.
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>& MatrixMxN<Type>::operator=( Type rhs )
{
   const size_t sqrsize( m_*n_ );
   for( size_t i=0; i<sqrsize; ++i )
      v_[i] = rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for MatrixMxN.
 *
 * \param rhs Matrix to be copied.
 * \return Reference to the assigned matrix.
 *
 * The matrix is resized according to the given \f$ M \times N \f$ matrix and initialized as a
 * copy of this matrix.
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>& MatrixMxN<Type>::operator=( const MatrixMxN& rhs )
{
   if( &rhs == this ) return *this;

   resize( rhs.m_, rhs.n_, false );

   const size_t sqrsize( m_*n_ );
   for( size_t i=0; i<sqrsize; ++i )
      v_[i] = rhs.v_[i];

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different dense matrices.
 *
 * \param rhs Dense matrix to be copied.
 * \return Reference to the assigned matrix.
 *
 * The matrix is resized according to the given \f$ M \times N \f$ matrix and initialized as a
 * copy of this matrix.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline MatrixMxN<Type>& MatrixMxN<Type>::operator=( const DenseMatrix<MT>& rhs )
{
   using pe::assign;

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      MatrixMxN tmp( rhs );
      swap( tmp );
   }
   else {
      resize( (~rhs).rows(), (~rhs).columns(), false );
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
 *
 * The matrix is resized according to the given \f$ M \times N \f$ matrix and initialized as a
 * copy of this matrix.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline MatrixMxN<Type>& MatrixMxN<Type>::operator=( const SparseMatrix<MT>& rhs )
{
   using pe::assign;

   resize( (~rhs).rows(), (~rhs).columns(), false );
   reset();
   assign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 1D-access to the matrix elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..M \cdot N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline Type& MatrixMxN<Type>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < m_*n_, "Invalid matrix access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 1D-access to the matrix elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..M \cdot N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline const Type& MatrixMxN<Type>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < m_*n_, "Invalid matrix access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access to the matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
 * \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline Type& MatrixMxN<Type>::operator()( size_t i, size_t j )
{
   pe_USER_ASSERT( i<m_ && j<n_, "Invalid matrix access index" );
   return v_[i*n_+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access to the matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
 * \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the matrix
inline const Type& MatrixMxN<Type>::operator()( size_t i, size_t j ) const
{
   pe_USER_ASSERT( i<m_ && j<n_, "Invalid matrix access index" );
   return v_[i*n_+j];
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
inline MatrixMxN<Type>& MatrixMxN<Type>::operator+=( const DenseMatrix<MT>& rhs )
{
   using pe::addAssign;

   if( (~rhs).rows() != m_ || (~rhs).columns() != n_ )
      throw std::invalid_argument( "Matrix sizes do not match" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      const MatrixMxN tmp( rhs );
      const size_t sqrsize( m_ * n_ );
      for( size_t i=0; i<sqrsize; ++i )
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
inline MatrixMxN<Type>& MatrixMxN<Type>::operator+=( const SparseMatrix<MT>& rhs )
{
   using pe::addAssign;

   if( (~rhs).rows() != m_ || (~rhs).columns() != n_ )
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
inline MatrixMxN<Type>& MatrixMxN<Type>::operator-=( const DenseMatrix<MT>& rhs )
{
   using pe::subAssign;

   if( (~rhs).rows() != m_ || (~rhs).columns() != n_ )
      throw std::invalid_argument( "Matrix sizes do not match" );

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      const MatrixMxN tmp( rhs );
      const size_t sqrsize( m_ * n_ );
      for( size_t i=0; i<sqrsize; ++i )
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
inline MatrixMxN<Type>& MatrixMxN<Type>::operator-=( const SparseMatrix<MT>& rhs )
{
   using pe::subAssign;

   if( (~rhs).rows() != m_ || (~rhs).columns() != n_ )
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
inline MatrixMxN<Type>& MatrixMxN<Type>::operator*=( const DenseMatrix<MT>& rhs )
{
   using pe::multAssign;

   if( (~rhs).rows() != n_ )
      throw std::invalid_argument( "Matrix sizes do not match" );

   multAssign( *this, ~rhs );
   return *this;
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
inline MatrixMxN<Type>& MatrixMxN<Type>::operator*=( const SparseMatrix<MT>& rhs )
{
   using pe::multAssign;

   if( (~rhs).rows() != n_ )
      throw std::invalid_argument( "Matrix sizes do not match" );

   multAssign( *this, ~rhs );
   return *this;
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
inline typename EnableIf< IsNumeric<Other>, MatrixMxN<Type> >::Type&
   MatrixMxN<Type>::operator*=( Other rhs )
{
   const size_t sqrsize( m_*n_ );
   for( size_t i=0; i<sqrsize; ++i )
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
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, MatrixMxN<Type> >::Type&
   MatrixMxN<Type>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   const size_t sqrsize( m_*n_ );

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      for( size_t i=0; i<sqrsize; ++i )
         v_[i] = static_cast<Type>( static_cast<DT>( v_[i] ) * tmp );
   }
   else {
      for( size_t i=0; i<sqrsize; ++i )
         v_[i] /= rhs;
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
/*!\brief Returns the current number of rows of the matrix.
 *
 * \return The number of rows of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t MatrixMxN<Type>::rows() const
{
   return m_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current number of columns of the matrix.
 *
 * \return The number of columns of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t MatrixMxN<Type>::columns() const
{
   return n_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum capacity of the matrix.
 *
 * \return The capacity of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t MatrixMxN<Type>::capacity() const
{
   return capacity_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the total number of non-zero elements in the matrix
 *
 * \return The number of non-zero elements in the sparse matrix.
 */
template< typename Type >  // Data type of the matrix
inline size_t MatrixMxN<Type>::nonZeros() const
{
   size_t nonzeros( 0 );

   const size_t sqrsize( m_*n_ );
   for( size_t i=0; i<sqrsize; ++i )
      if( v_[i] != Type() )
         ++nonzeros;

   return nonzeros;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of non-zero elements in the specified row.
 *
 * \param i The index of the row.
 * \return The number of non-zero elements of row \a i.
 */
template< typename Type >  // Data type of the matrix
inline size_t MatrixMxN<Type>::nonZeros( size_t i ) const
{
   pe_USER_ASSERT( i < rows(), "Invalid row access index" );

   const size_t end( (i+1)*n_ );
   size_t nonzeros( 0 );

   for( size_t j=i*n_; j<end; ++j )
      if( v_[j] != Type() )
         ++nonzeros;

   return nonzeros;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
template< typename Type >  // Data type of the matrix
inline void MatrixMxN<Type>::reset()
{
   using pe::reset;
   const size_t sqrsize( m_*n_ );
   for( size_t i=0; i<sqrsize; ++i )
      reset( v_[i] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the \f$ M \times N \f$ matrix.
 *
 * \return void
 *
 * After the clear() function, the size of the matrix is 0.
 */
template< typename Type >  // Data type of the matrix
inline void MatrixMxN<Type>::clear()
{
   m_ = 0;
   n_ = 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the size of the matrix.
 *
 * \param m The new number of rows of the matrix.
 * \param n The new number of columns of the matrix.
 * \param preserve \a true if the old values of the matrix should be preserved, \a false if not.
 * \return void
 *
 * This function resizes the matrix using the given size to \f$ m \times n \f$. During this
 * operation, new dynamic memory may be allocated in case the capacity of the matrix is too
 * small. Therefore this function potentially changes all matrix elements. In order to preserve
 * the old matrix values, the \a preserve flag can be set to \a true. However, new matrix
 * elements are not initialized!\n
 * The following example illustrates the resize operation of a \f$ 2 \times 4 \f$ matrix to a
 * \f$ 4 \times 2 \f$ matrix. The new, uninitialized elements are marked with \a x:

                              \f[
                              \left(\begin{array}{*{4}{c}}
                              1 & 2 & 3 & 4 \\
                              5 & 6 & 7 & 8 \\
                              \end{array}\right)

                              \Longrightarrow

                              \left(\begin{array}{*{2}{c}}
                              1 & 2 \\
                              5 & 6 \\
                              x & x \\
                              x & x \\
                              \end{array}\right)
                              \f]
 */
template< typename Type >  // Data type of the matrix
void MatrixMxN<Type>::resize( size_t m, size_t n, bool preserve )
{
   if( m == m_ && n == n_ ) return;

   if( preserve )
   {
      Type* pe_RESTRICT v = new Type[m*n];
      const size_t min_m( pe::min( m, m_ ) );
      const size_t min_n( pe::min( n, n_ ) );

      for( size_t i=0; i<min_m; ++i )
         for( size_t j=0; j<min_n; ++j )
            v[i*n+j] = v_[i*n_+j];

      std::swap( v_, v );
      delete [] v;
      capacity_ = m*n;
   }
   else if( m*n > capacity_ ) {
      Type* pe_RESTRICT v = new Type[m*n];
      std::swap( v_, v );
      delete [] v;
      capacity_ = m*n;
   }

   m_ = m;
   n_ = n;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extending the size of the matrix.
 *
 * \param m Number of additional rows.
 * \param n Number of additional columns.
 * \param preserve \a true if the old values of the matrix should be preserved, \a false if not.
 * \return void
 *
 * This function increases the matrix size by \a m rows and \a n columns. During this operation,
 * new dynamic memory may be allocated in case the capacity of the matrix is too small. Therefore
 * this function potentially changes all matrix elements. In order to preserve the old matrix
 * values, the \a preserve flag can be set to \a true. However, new matrix elements are not
 * initialized!
 */
template< typename Type >  // Data type of the matrix
inline void MatrixMxN<Type>::extend( size_t m, size_t n, bool preserve )
{
   resize( m_+m, n_+n, preserve );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of the matrix.
 *
 * \param elements The new minimum capacity of the sparse matrix.
 * \return void
 *
 * This function increases the capacity of the sparse matrix to at least \a elements elements.
 * The current values of the matrix elements are preserved.
 */
template< typename Type >  // Data type of the matrix
inline void MatrixMxN<Type>::reserve( size_t elements )
{
   if( elements > capacity_ )
   {
      // Allocating a new array
      Type* pe_RESTRICT tmp = new Type[elements];

      // Replacing the old array
      std::copy( v_, v_+m_*n_, tmp );
      std::swap( tmp, v_ );
      delete [] tmp;

      capacity_ = elements;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transposing the matrix.
 *
 * \return Reference to the transposed matrix.
 */
template< typename Type >  // Data type of the matrix
inline MatrixMxN<Type>& MatrixMxN<Type>::transpose()
{
   MatrixMxN tmp( trans(*this) );
   swap( tmp );
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
inline MatrixMxN<Type>& MatrixMxN<Type>::invert()
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
inline const MatrixMxN<Type> MatrixMxN<Type>::getInverse() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   return MatrixMxN();
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

                        \f[\left(\begin{array}{*{5}{c}}
                        aa     & 0      & 0      & \cdots & 0  \\
                        0      & bb     & 0      & \cdots & 0  \\
                        0      & 0      & cc     & \cdots & 0  \\
                        \vdots & \vdots & \vdots & \ddots & 0  \\
                        0      & 0      & 0      & 0      & mn \\
                        \end{array}\right)\f]
 */
template< typename Type >  // Data type of the matrix
inline bool MatrixMxN<Type>::isDiagonal() const
{
   const size_t iend( m_-1 );

   for( size_t i=0; i<iend; ++i ) {
      for( size_t j=i+1; j<n_; ++j ) {
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
inline bool MatrixMxN<Type>::isSymmetric() const
{
   const size_t iend( m_-1 );

   for( size_t i=0; i<iend; ++i ) {
      for( size_t j=i+1; j<n_; ++j ) {
         if( !equal( v_[i*n_+j], v_[j*n_+i] ) )
            return false;
      }
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the matrix by the scalar value \a scalar (\f$ A=B*s \f$).
 *
 * \param scalar The scalar value for the matrix scaling.
 * \return Reference to the matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the scalar value
inline MatrixMxN<Type>& MatrixMxN<Type>::scale( Other scalar )
{
   const size_t sqrsize( m_*n_ );
   for( size_t i=0; i<sqrsize; ++i )
      v_[i] *= scalar;

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two matrices.
 *
 * \param m The matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the matrix
inline void MatrixMxN<Type>::swap( MatrixMxN& m ) /* throw() */
{
   std::swap( m_, m.m_ );
   std::swap( n_, m.n_ );
   std::swap( capacity_, m.capacity_ );
   std::swap( v_, m.v_ );
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
inline bool MatrixMxN<Type>::isAliased( const Other* alias ) const
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
inline void MatrixMxN<Type>::assign( const DenseMatrix<MT>& rhs )
{
   const size_t end( (~rhs).columns() & size_t(-2) );

   for( size_t i=0; i<(~rhs).rows(); ++i ) {
      for( size_t j=0; j<end; j+=2 ) {
         v_[i*n_+j  ] = (~rhs)(i,j  );
         v_[i*n_+j+1] = (~rhs)(i,j+1);
      }
      if( end < (~rhs).columns() ) {
         v_[i*n_+end] = (~rhs)(i,end);
      }
   }
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
inline void MatrixMxN<Type>::assign( const SparseMatrix<MT>& rhs )
{
   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<(~rhs).rows(); ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*n_+element->index()] = element->value();
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
inline void MatrixMxN<Type>::addAssign( const DenseMatrix<MT>& rhs )
{
   const size_t end( (~rhs).columns() & size_t(-2) );

   for( size_t i=0; i<(~rhs).rows(); ++i ) {
      for( size_t j=0; j<end; j+=2 ) {
         v_[i*n_+j  ] += (~rhs)(i,j  );
         v_[i*n_+j+1] += (~rhs)(i,j+1);
      }
      if( end < (~rhs).columns() ) {
         v_[i*n_+end] += (~rhs)(i,end);
      }
   }
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
inline void MatrixMxN<Type>::addAssign( const SparseMatrix<MT>& rhs )
{
   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<(~rhs).rows(); ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*n_+element->index()] += element->value();
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
inline void MatrixMxN<Type>::subAssign( const DenseMatrix<MT>& rhs )
{
   const size_t end( (~rhs).columns() & size_t(-2) );

   for( size_t i=0; i<(~rhs).rows(); ++i ) {
      for( size_t j=0; j<end; j+=2 ) {
         v_[i*n_+j  ] -= (~rhs)(i,j  );
         v_[i*n_+j+1] -= (~rhs)(i,j+1);
      }
      if( end < (~rhs).columns() ) {
         v_[i*n_+end] -= (~rhs)(i,end);
      }
   }
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
inline void MatrixMxN<Type>::subAssign( const SparseMatrix<MT>& rhs )
{
   typedef typename MT::ConstIterator  ConstIterator;

   for( size_t i=0; i<(~rhs).rows(); ++i )
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element )
         v_[i*n_+element->index()] -= element->value();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be multiplied.
 * \return void
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void MatrixMxN<Type>::multAssign( const DenseMatrix<MT>& rhs )
{
   MatrixMxN tmp( *this * (~rhs) );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be multiplied.
 * \return void
 * \exception std::invalid_argument Matrix sizes do not match.
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void MatrixMxN<Type>::multAssign( const SparseMatrix<MT>& rhs )
{
   MatrixMxN tmp( *this * (~rhs) );
   swap( tmp );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name MatrixMxN operators */
//@{
template< typename Type >
inline bool isnan( const MatrixMxN<Type>& m );

template< typename Type >
inline void reset( MatrixMxN<Type>& m );

template< typename Type >
inline void clear( MatrixMxN<Type>& m );

template< typename Type >
inline bool isDefault( const MatrixMxN<Type>& m );

template< typename Type >
inline const MatrixMxN<Type> inv( const MatrixMxN<Type>& m );

template< typename Type >
inline const DMatDMatMultExpr< MatrixMxN<Type>, MatrixMxN<Type> > sq( const MatrixMxN<Type>& m );

template< typename Type >
inline void swap( MatrixMxN<Type>& a, MatrixMxN<Type>& b ) /* throw() */;
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
inline bool isnan( const MatrixMxN<Type>& m )
{
   for( size_t i=0; i<m.rows(); ++i ) {
      for( size_t j=0; j<m.columns(); ++j )
         if( isnan( m(i,j) ) ) return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given dense matrix.
 * \ingroup dense_matrix_MxN
 *
 * \param m The dense matrix to be resetted.
 * \return void
 */
template< typename Type >  // Data type of the matrix
inline void reset( MatrixMxN<Type>& m )
{
   m.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given dense matrix.
 * \ingroup dense_matrix_MxN
 *
 * \param m The dense matrix to be cleared.
 * \return void
 */
template< typename Type >  // Data type of the matrix
inline void clear( MatrixMxN<Type>& m )
{
   m.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given dense matrix is in default state.
 * \ingroup dense_matrix_MxN
 *
 * \param m The dense matrix to be tested for its default state.
 * \return \a true in case the given matrix is component-wise zero, \a false otherwise.
 */
template< typename Type >  // Data type of the matrix
inline bool isDefault( const MatrixMxN<Type>& m )
{
   const size_t sqrsize( m.rows()*m.columns() );
   for( size_t i=0; i<sqrsize; ++i )
      if( !isDefault( m[i] ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the given dense matrix.
 * \ingroup dense_matrix_MxN
 *
 * \param m The dense matrix to be inverted.
 * \return The inverse of the matrix.
 *
 * This function returns the inverse of the given dense matrix. It has the same effect as
 * calling the getInverse() member function of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline const MatrixMxN<Type> inv( const MatrixMxN<Type>& m )
{
   return m.getInverse();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Squaring the given dense matrix.
 * \ingroup dense_matrix_MxN
 *
 * \param m The dense matrix to be squared.
 * \return The result of the square operation.
 *
 * This function squares the given dense matrix \a m. This function has the same effect as
 * multiplying the matrix with itself (\f$ m * m \f$).
 */
template< typename Type >  // Data type of the matrix
inline const DMatDMatMultExpr< MatrixMxN<Type>, MatrixMxN<Type> >
   sq( const MatrixMxN<Type>& m )
{
   return DMatDMatMultExpr< MatrixMxN<Type>, MatrixMxN<Type> >( ~m, ~m );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two matrices.
 * \ingroup dense_matrix_MxN
 *
 * \param a The first matrix to be swapped.
 * \param b The second matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the matrices
inline void swap( MatrixMxN<Type>& a, MatrixMxN<Type>& b ) /* throw() */
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
struct MathTrait< MatrixMxN<T1>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::DivType  >  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T2 );
};

template< typename T1, typename T2 >
struct MathTrait< T1, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T1 );
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, Vector2<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,true>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, Vector3<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector3<T1,true>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, Vector6<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector6<T1,true>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, Matrix2x2<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix2x2<T1>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, Matrix3x3<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix3x3<T1>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, Matrix6x6<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix6x6<T1>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, MatrixMxN<T2> >
{
   typedef MatrixMxN< typename MathTrait<T1,T2>::HighType >  HighType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::LowType  >  LowType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
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
/*!\brief MxN real matrix.
 * \ingroup dense_matrix_MxN
 */
typedef MatrixMxN<real>  MatN;
//*************************************************************************************************

} // namespace pe

#endif
