//=================================================================================================
/*!
 *  \file pe/math/SparseMatrixMxN.h
 *  \brief Implementation of a sparse MxN matrix
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

#ifndef _PE_MATH_SPARSEMATRIXMXN_H_
#define _PE_MATH_SPARSEMATRIXMXN_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <functional>
#include <stdexcept>
#include <vector>
#include <pe/math/MathTrait.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/sparse/MatrixAccessProxy.h>
#include <pe/math/sparse/SparseElement.h>
#include <pe/math/SparseMatrix.h>
#include <pe/math/SparseVectorN.h>
#include <pe/math/VectorN.h>
#include <pe/system/Precision.h>
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

template< typename >       class Matrix3x3;
template< typename >       class Matrix6x6;
template< typename, bool > class SparseVectorN;
template< typename, bool > class Vector2;
template< typename, bool > class Vector3;
template< typename, bool > class Vector6;
template< typename, bool > class VectorN;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup sparse_matrix_MxN SparseMatrixMxN
 * \ingroup sparse_matrix
 */
/*!\brief Efficient implementation of a \f$ M \times N \f$ sparse matrix.
 * \ingroup sparse_matrix_MxN
 *
 * The SparseMatrixMxN class is the representation of a sparse \f$ M \times N \f$ matrix with
 * a maximum of \f$ M \cdot N \f$ non-zero, dynamically allocated elements. The order of the
 * elements is as following:

               \f[\left(\begin{array}{*{5}{c}}
               0           & 1             & 2             & \cdots & N-1         \\
               N           & N+1           & N+2           & \cdots & 2 \cdot N-1 \\
               \vdots      & \vdots        & \vdots        & \ddots & \vdots      \\
               M \cdot N-N & M \cdot N-N+1 & M \cdot N-N+2 & \cdots & M \cdot N-1 \\
               \end{array}\right)\f]

 * Inserting/accessing elements in a sparse matrix can be done by several alternative functions.
 * The following example demonstrates all options:

   \code
   // Creating a 4x3 sparse matrix with 4 rows and 3 columns
   SparseMatrixMxN<double> A( 4, 3 );

   // The function call operator provides access to all possible elements of the sparse matrix,
   // including the zero elements. In case the function call operator is used to access an element
   // that is currently not stored in the sparse matrix, the element is inserted into the matrix.
   A(1,2) = 2.0;

   // An alternative for inserting elements into the matrix is the insert() function. However,
   // it inserts the element only in case the element is not already contained in the matrix.
   A.insert( 2, 1, 3.7 );

   // A very efficient way to add new elements to a sparse matrix is the append() function.
   // Note that append() requires that the appended element's index is strictly larger than
   // the currently largest non-zero index of the specified row and that the rows's capacity
   // is large enough to hold the new element.
   v.reserve( 3, 3 );
   v.append( 3, 1, -2.1 );

   // In order to traverse all non-zero elements currently stored in the matrix, the begin()
   // and end() functions can be used. In the example, all non-zero elements of the 2nd row
   // are traversed.
   for( SparseMatrixMxN<double>::Iterator i=A.begin(1); i!=A.end(1); ++i )
      ... = i->value();  // Access to the value of the non-zero element
      ... = i->index();  // Access to the index of the non-zero element
   \endcode

 * SparseMatrixMxN can be used with any non-cv-qualified element type. The arithmetic operators
 * for matrix/matrix, matrix/vector and matrix/element operations with the same element type work
 * for any element type as long as the element type supports the arithmetic operation. Arithmetic
 * operations between matrices, vectors and elements of different element types are only supported
 * for all data types supported by the MathTrait class template (for details see the MathTrait
 * class description).

   \code
   SparseMatrixMxN< double > a, b, c;
   SparseMatrixMxN< float  > d;
   SparseMatrixMxN< std::complex<double> > e, f, g;
   SparseMatrixMxN< std::complex<float>  > h;

   ...         // Appropriate resizing

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type >  // Data type of the sparse matrix
class SparseMatrixMxN : public SparseMatrix< SparseMatrixMxN<Type> >
{
private:
   //**Type definitions****************************************************************************
   typedef SparseElement<Type>  ElementBase;  //!< Base class for the sparse matrix element.
   //**********************************************************************************************

   //**Private class Element***********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Index-value-pair for the SparseMatrixMxN class.
   */
   struct Element : public ElementBase
   {
      using ElementBase::operator=;
      friend class SparseMatrixMxN;
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Private class FindIndex*********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Helper class for the lower_bound() function.
   */
   struct FindIndex : public std::binary_function<Element,size_t,bool>
   {
      inline bool operator()( const Element& element, size_t index ) const {
         return element.index() < index;
      }
      inline bool operator()( size_t index, const Element& element ) const {
         return index < element.index();
      }
      inline bool operator()( const Element& element1, const Element& element2 ) const {
         return element1.index() < element2.index();
      }
   };
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef SparseMatrixMxN<Type>    This;            //!< Type of this SparseMatrixMxN instance.
   typedef This                     ResultType;      //!< Result type for expression template evaluations.
   typedef Type                     ElementType;     //!< Type of the sparse matrix elements.
   typedef const SparseMatrixMxN&   CompositeType;   //!< Data type for composite expression templates.
   typedef MatrixAccessProxy<This>  Reference;       //!< Reference to a sparse matrix value.
   typedef const Type&              ConstReference;  //!< Reference to a constant sparse matrix value.
   typedef Element*                 Iterator;        //!< Iterator over non-constant elements.
   typedef const Element*           ConstIterator;   //!< Iterator over constant elements.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                           explicit inline SparseMatrixMxN();
                           explicit inline SparseMatrixMxN( size_t m, size_t n );
                           explicit inline SparseMatrixMxN( size_t m, size_t n, size_t nonzeros );
                           explicit        SparseMatrixMxN( size_t m, size_t n, const std::vector<size_t>& nonzeros );
                                    inline SparseMatrixMxN( const SparseMatrixMxN& sm );
   template< typename MT >          inline SparseMatrixMxN( const DenseMatrix<MT>&  dm );
   template< typename MT >          inline SparseMatrixMxN( const SparseMatrix<MT>& sm );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~SparseMatrixMxN();
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
                           inline SparseMatrixMxN& operator= ( const SparseMatrixMxN&  rhs );
   template< typename MT > inline SparseMatrixMxN& operator= ( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline SparseMatrixMxN& operator= ( const SparseMatrix<MT>& rhs );
                           inline Reference        operator()( size_t i, size_t j );
                           inline ConstReference   operator()( size_t i, size_t j ) const;
   template< typename MT > inline SparseMatrixMxN& operator+=( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline SparseMatrixMxN& operator+=( const SparseMatrix<MT>& rhs );
   template< typename MT > inline SparseMatrixMxN& operator-=( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline SparseMatrixMxN& operator-=( const SparseMatrix<MT>& rhs );
   template< typename MT > inline SparseMatrixMxN& operator*=( const DenseMatrix<MT>&  rhs );
   template< typename MT > inline SparseMatrixMxN& operator*=( const SparseMatrix<MT>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, SparseMatrixMxN >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, SparseMatrixMxN >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t                rows()               const;
                              inline size_t                columns()            const;
                              inline size_t                capacity()           const;
                              inline size_t                capacity( size_t i ) const;
                              inline size_t                nonZeros()           const;
                              inline size_t                nonZeros( size_t i ) const;
                              inline Iterator              begin( size_t i );
                              inline ConstIterator         begin( size_t i )    const;
                              inline Iterator              end  ( size_t i );
                              inline ConstIterator         end  ( size_t i )    const;
                              inline void                  reset();
                              inline void                  clear();
                                     void                  append ( size_t i, size_t j, const Type& value );
                                     Type&                 insert ( size_t i, size_t j, const Type& value );
                              inline Iterator              find   ( size_t i, size_t j );
                              inline ConstIterator         find   ( size_t i, size_t j ) const;
                                     void                  resize ( size_t m, size_t n, bool preserve=true );
                              inline void                  reserve( size_t nonzeros );
                                     void                  reserve( size_t i, size_t nonzeros );
                              inline SparseMatrixMxN&      transpose();
                              inline SparseMatrixMxN&      invert();
                              inline const SparseMatrixMxN getInverse()   const;
                                     bool                  isDiagonal()   const;
                                     bool                  isSymmetric()  const;
   template< typename Other > inline SparseMatrixMxN&      scale( Other scalar );
   template< typename Other > inline SparseMatrixMxN&      scaleDiagonal( Other scalar );
                              inline void                  swap( SparseMatrixMxN& sm ) /* throw() */;
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
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline size_t extendCapacity() const;
   inline void   reserveElements( size_t nonzeros );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t m_;         //!< The current number of rows of the sparse matrix.
   size_t n_;         //!< The current number of columns of the sparse matrix.
   size_t capacity_;  //!< The current capacity of the pointer array.
   Iterator* begin_;  //!< Pointers to the first non-zero element of each row.
   Iterator* end_;    //!< Pointers one past the last non-zero element of each row.

   static const Type zero_;  //!< Neutral element for accesses to zero elements.
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_NOT_BE_CONST   ( Type );
   pe_CONSTRAINT_MUST_NOT_BE_VOLATILE( Type );
   pe_CONSTRAINT_MUST_HAVE_SAME_SIZE ( ElementBase, Element );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

template< typename Type >
const Type SparseMatrixMxN<Type>::zero_ = Type();




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for SparseMatrixMxN.
 */
template< typename Type >  // Data type of the sparse matrix
inline SparseMatrixMxN<Type>::SparseMatrixMxN()
   : m_       ( 0 )             // The current number of rows of the sparse matrix
   , n_       ( 0 )             // The current number of columns of the sparse matrix
   , capacity_( 0 )             // The current capacity of the pointer array
   , begin_( new Iterator[2] )  // Pointers to the first non-zero element of each row
   , end_  ( begin_+1 )         // Pointers one past the last non-zero element of each row
{
   begin_[0] = end_[0] = NULL;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a matrix of size \f$ M \times N \f$.
 *
 * \param m The number of rows of the matrix.
 * \param n The number of columns of the matrix.
 *
 * The matrix is initialized to the zero matrix and has no free capacity.
 */
template< typename Type >  // Data type of the sparse matrix
inline SparseMatrixMxN<Type>::SparseMatrixMxN( size_t m, size_t n )
   : m_       ( m )                 // The current number of rows of the sparse matrix
   , n_       ( n )                 // The current number of columns of the sparse matrix
   , capacity_( m )                 // The current capacity of the pointer array
   , begin_( new Iterator[2*m+2] )  // Pointers to the first non-zero element of each row
   , end_  ( begin_+(m+1) )         // Pointers one past the last non-zero element of each row
{
   for( size_t i=0; i<2*m_+2; ++i )
      begin_[i] = NULL;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a matrix of size \f$ M \times N \f$.
 *
 * \param m The number of rows of the matrix.
 * \param n The number of columns of the matrix.
 * \param nonzeros The number of expected non-zero elements.
 *
 * The matrix is initialized to the zero matrix.
 */
template< typename Type >  // Data type of the sparse matrix
inline SparseMatrixMxN<Type>::SparseMatrixMxN( size_t m, size_t n, size_t nonzeros )
   : m_       ( m )                 // The current number of rows of the sparse matrix
   , n_       ( n )                 // The current number of columns of the sparse matrix
   , capacity_( m )                 // The current capacity of the pointer array
   , begin_( new Iterator[2*m+2] )  // Pointers to the first non-zero element of each row
   , end_  ( begin_+(m+1) )         // Pointers one past the last non-zero element of each row
{
   begin_[0] = new Element[nonzeros];
   for( size_t i=1; i<(2*m_+1); ++i )
      begin_[i] = begin_[0];
   end_[m_] = begin_[0]+nonzeros;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a matrix of size \f$ M \times N \f$.
 *
 * \param m The number of rows of the matrix.
 * \param n The number of columns of the matrix.
 * \param nonzeros The expected number of non-zero elements in each row.
 *
 * The matrix is initialized to the zero matrix.
 */
template< typename Type >  // Data type of the sparse matrix
SparseMatrixMxN<Type>::SparseMatrixMxN( size_t m, size_t n, const std::vector<size_t>& nonzeros )
   : m_       ( m )                  // The current number of rows of the sparse matrix
   , n_       ( n )                  // The current number of columns of the sparse matrix
   , capacity_( m )                  // The current capacity of the pointer array
   , begin_( new Iterator[2*m_+2] )  // Pointers to the first non-zero element of each row
   , end_  ( begin_+(m_+1) )         // Pointers one past the last non-zero element of each row
{
   pe_USER_ASSERT( nonzeros.size() == m, "Size of capacity vector and number of rows don't match" );

   size_t newCapacity( 0 );
   for( std::vector<size_t>::const_iterator it=nonzeros.begin(); it!=nonzeros.end(); ++it )
      newCapacity += *it;

   begin_[0] = end_[0] = new Element[newCapacity];
   for( size_t i=0; i<m_; ++i ) {
      begin_[i+1] = end_[i+1] = begin_[i] + nonzeros[i];
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for SparseMatrixMxN.
 *
 * \param sm Sparse matrix to be copied.
 */
template< typename Type >  // Data type of the sparse matrix
inline SparseMatrixMxN<Type>::SparseMatrixMxN( const SparseMatrixMxN& sm )
   : m_       ( sm.m_ )                 // The current number of rows of the sparse matrix
   , n_       ( sm.n_ )                 // The current number of columns of the sparse matrix
   , capacity_( sm.m_ )                 // The current capacity of the pointer array
   , begin_   ( new Iterator[2*m_+2] )  // Pointers to the first non-zero element of each row
   , end_     ( begin_+(m_+1) )         // Pointers one past the last non-zero element of each row
{
   const size_t nonzeros( sm.nonZeros() );

   begin_[0] = new Element[nonzeros];
   for( size_t i=0; i<m_; ++i )
      begin_[i+1] = end_[i] = std::copy( sm.begin(i), sm.end(i), begin_[i] );
   end_[m_] = begin_[0]+nonzeros;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from dense matrices.
 *
 * \param dm Dense matrix to be copied.
 */
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the foreign dense matrix
inline SparseMatrixMxN<Type>::SparseMatrixMxN( const DenseMatrix<MT>& dm )
   : m_       ( (~dm).rows() )          // The current number of rows of the sparse matrix
   , n_       ( (~dm).columns() )       // The current number of columns of the sparse matrix
   , capacity_( m_ )                    // The current capacity of the pointer array
   , begin_   ( new Iterator[2*m_+2] )  // Pointers to the first non-zero element of each row
   , end_     ( begin_+(m_+1) )         // Pointers one past the last non-zero element of each row
{
   using pe::assign;

   for( size_t i=0; i<2*m_+2; ++i )
      begin_[i] = NULL;

   assign( *this, ~dm );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different sparse matrices.
 *
 * \param sm Sparse matrix to be copied.
 */
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the foreign sparse matrix
inline SparseMatrixMxN<Type>::SparseMatrixMxN( const SparseMatrix<MT>& sm )
   : m_       ( (~sm).rows() )          // The current number of rows of the sparse matrix
   , n_       ( (~sm).columns() )       // The current number of columns of the sparse matrix
   , capacity_( m_ )                    // The current capacity of the pointer array
   , begin_   ( new Iterator[2*m_+2] )  // Pointers to the first non-zero element of each row
   , end_     ( begin_+(m_+1) )         // Pointers one past the last non-zero element of each row
{
   using pe::assign;

   const size_t nonzeros( (~sm).nonZeros() );

   begin_[0] = new Element[nonzeros];
   for( size_t i=0; i<m_; ++i )
      begin_[i+1] = end_[i] = begin_[0];
   end_[m_] = begin_[0]+nonzeros;

   assign( *this, ~sm );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for SparseMatrixMxN.
 */
template< typename Type >  // Data type of the sparse matrix
inline SparseMatrixMxN<Type>::~SparseMatrixMxN()
{
   delete [] begin_[0];
   delete [] begin_;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for SparseMatrixMxN.
 *
 * \param rhs Sparse matrix to be copied.
 * \return Reference to the assigned sparse matrix.
 *
 * The sparse matrix is resized according to the given sparse matrix and initialized as a
 * copy of this matrix.
 */
template< typename Type >  // Data type of the sparse matrix
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator=( const SparseMatrixMxN& rhs )
{
   if( &rhs == this ) return *this;

   const size_t nonzeros( rhs.nonZeros() );

   if( rhs.m_ > capacity_ || nonzeros > capacity() )
   {
      Iterator* newBegin( new Iterator[2*rhs.m_+2] );
      Iterator* newEnd  ( newBegin+(rhs.m_+1) );

      newBegin[0] = new Element[nonzeros];
      for( size_t i=0; i<rhs.m_; ++i ) {
         newBegin[i+1] = newEnd[i] = std::copy( rhs.begin_[i], rhs.end_[i], newBegin[i] );
      }
      newEnd[rhs.m_] = newBegin[0]+nonzeros;

      std::swap( begin_, newBegin );
      end_ = newEnd;
      delete [] newBegin[0];
      delete [] newBegin;
      capacity_ = rhs.m_;
   }
   else {
     for( size_t i=0; i<rhs.m_; ++i ) {
         begin_[i+1] = end_[i] = std::copy( rhs.begin_[i], rhs.end_[i], begin_[i] );
      }
   }

   m_ = rhs.m_;
   n_ = rhs.n_;

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for dense matrices.
 *
 * \param rhs Dense matrix to be copied.
 * \return Reference to the assigned matrix.
 *
 * The matrix is resized according to the given \f$ M \times N \f$ matrix and initialized as a
 * copy of this matrix.
 */
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator=( const DenseMatrix<MT>& rhs )
{
   using pe::assign;

   if( IsExpression<MT>::value && (~rhs).isAliased( this ) ) {
      SparseMatrixMxN tmp( rhs );
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
/*!\brief Assignment operator for different sparse matrices.
 *
 * \param rhs Sparse matrix to be copied.
 * \return Reference to the assigned matrix.
 *
 * The matrix is resized according to the given \f$ M \times N \f$ matrix and initialized as a
 * copy of this matrix.
 */
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator=( const SparseMatrix<MT>& rhs )
{
   using pe::assign;

   if( ( IsExpression<MT>::value && (~rhs).isAliased( this ) ) ||
       (~rhs).rows()     > capacity_ ||
       (~rhs).nonZeros() > capacity() ) {
      SparseMatrixMxN tmp( rhs );
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
/*!\brief 2D-access to the sparse matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
 * \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::Reference
   SparseMatrixMxN<Type>::operator()( size_t i, size_t j )
{
   pe_USER_ASSERT( i < rows()   , "Invalid row access index"    );
   pe_USER_ASSERT( j < columns(), "Invalid column access index" );

   return Reference( *this, i, j );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access to the sparse matrix elements.
 *
 * \param i Access index for the row. The index has to be in the range \f$[0..M-1]\f$.
 * \param j Access index for the column. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::ConstReference
   SparseMatrixMxN<Type>::operator()( size_t i, size_t j ) const
{
   pe_USER_ASSERT( i < rows()   , "Invalid row access index"    );
   pe_USER_ASSERT( j < columns(), "Invalid column access index" );

   const ConstIterator pos( std::lower_bound( begin_[i], end_[i], j, FindIndex() ) );

   if( pos == end_[i] || pos->index_ != j )
      return zero_;
   else
      return pos->value_;
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
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator+=( const DenseMatrix<MT>& rhs )
{
   using pe::addAssign;

   if( (~rhs).rows() != m_ || (~rhs).columns() != n_ )
      throw std::invalid_argument( "Matrix sizes do not match" );

   addAssign( *this, ~rhs );
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
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator+=( const SparseMatrix<MT>& rhs )
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
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator-=( const DenseMatrix<MT>& rhs )
{
   using pe::subAssign;

   if( (~rhs).rows() != m_ || (~rhs).columns() != n_ )
      throw std::invalid_argument( "Matrix sizes do not match" );

   subAssign( *this, ~rhs );
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
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator-=( const SparseMatrix<MT>& rhs )
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
 * \brief (\f$ A*=B \f$).
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
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator*=( const DenseMatrix<MT>& rhs )
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
 * \brief (\f$ A*=B \f$).
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
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::operator*=( const SparseMatrix<MT>& rhs )
{
   using pe::multAssign;

   if( (~rhs).rows() != n_ )
      throw std::invalid_argument( "Matrix sizes do not match" );

   multAssign( *this, ~rhs );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication between a sparse matrix and
 * \brief a scalar value (\f$ A*=s \f$).
 *
 * \param rhs The right-hand side scalar value for the multiplication.
 * \return Reference to the sparse matrix.
 */
template< typename Type >   // Data type of the sparse matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, SparseMatrixMxN<Type> >::Type&
   SparseMatrixMxN<Type>::operator*=( Other rhs )
{
   for( size_t i=0; i<m_; ++i ) {
      const Iterator endElem( end(i) );
      for( Iterator elem=begin(i); elem<endElem; ++elem )
         elem->value_ *= rhs;
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division assignment operator for the division of a sparse matrix by a scalar value
 * \brief (\f$ A/=s \f$).
 *
 * \param rhs The right-hand side scalar value for the division.
 * \return Reference to the matrix.
 */
template< typename Type >   // Data type of the matrix
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, SparseMatrixMxN<Type> >::Type&
   SparseMatrixMxN<Type>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      for( size_t i=0; i<m_; ++i ) {
         const Iterator endElem( end(i) );
         for( Iterator elem=begin(i); elem<endElem; ++elem )
            elem->value_ = static_cast<Type>( static_cast<DT>( elem->value_ ) * tmp );
      }
   }
   else {
      for( size_t i=0; i<m_; ++i ) {
         const Iterator endElem( end(i) );
         for( Iterator elem=begin(i); elem<endElem; ++elem )
            elem->value_ /= rhs;
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
/*!\brief Returns the current number of rows of the sparse matrix.
 *
 * \return The number of rows of the sparse matrix.
 */
template< typename Type >  // Data type of the sparse matrix
inline size_t SparseMatrixMxN<Type>::rows() const
{
   return m_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current number of columns of the sparse matrix.
 *
 * \return The number of columns of the sparse matrix.
 */
template< typename Type >  // Data type of the sparse matrix
inline size_t SparseMatrixMxN<Type>::columns() const
{
   return n_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum capacity of the sparse matrix.
 *
 * \return The capacity of the sparse matrix.
 */
template< typename Type >  // Data type of the sparse matrix
inline size_t SparseMatrixMxN<Type>::capacity() const
{
   return end_[m_] - begin_[0];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current capacity of the specified row.
 *
 * \param i The index of the row.
 * \return The current capacity of row \a i.
 */
template< typename Type >  // Data type of the sparse matrix
inline size_t SparseMatrixMxN<Type>::capacity( size_t i ) const
{
   pe_USER_ASSERT( i < rows(), "Invalid row access index" );
   return begin_[i+1] - begin_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of non-zero elements in the sparse matrix
 *
 * \return The number of non-zero elements in the sparse matrix.
 */
template< typename Type >  // Data type of the sparse matrix
inline size_t SparseMatrixMxN<Type>::nonZeros() const
{
   size_t nonzeros( 0 );

   for( size_t i=0; i<m_; ++i )
      nonzeros += nonZeros( i );

   return nonzeros;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of non-zero elements in the specified row.
 *
 * \param i The index of the row.
 * \return The number of non-zero elements of row \a i.
 */
template< typename Type >  // Data type of the sparse matrix
inline size_t SparseMatrixMxN<Type>::nonZeros( size_t i ) const
{
   pe_USER_ASSERT( i < rows(), "Invalid row access index" );
   return end_[i] - begin_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first non-zero element of row \a i.
 *
 * \return Iterator to the first non-zero element of row \a i.
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::Iterator SparseMatrixMxN<Type>::begin( size_t i )
{
   pe_USER_ASSERT( i < m_, "Invalid sparse matrix row access index" );
   return begin_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first non-zero element of row \a i.
 *
 * \return Iterator to the first non-zero element of row \a i.
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::ConstIterator SparseMatrixMxN<Type>::begin( size_t i ) const
{
   pe_USER_ASSERT( i < m_, "Invalid sparse matrix row access index" );
   return begin_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last non-zero element of row \a i.
 *
 * \return Iterator just past the last non-zero element of row \a i.
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::Iterator SparseMatrixMxN<Type>::end( size_t i )
{
   pe_USER_ASSERT( i < m_, "Invalid sparse matrix row access index" );
   return end_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last non-zero element of row \a i.
 *
 * \return Iterator just past the last non-zero element of row \a i.
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::ConstIterator SparseMatrixMxN<Type>::end( size_t i ) const
{
   pe_USER_ASSERT( i < m_, "Invalid sparse matrix row access index" );
   return end_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
template< typename Type >  // Data type of the sparse matrix
inline void SparseMatrixMxN<Type>::reset()
{
   for( size_t i=0; i<m_; ++i )
      end_[i] = begin_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the sparse matrix.
 *
 * \return void
 *
 * After the clear() function, the size of the sparse matrix is 0.
 */
template< typename Type >  // Data type of the sparse matrix
inline void SparseMatrixMxN<Type>::clear()
{
   end_[0] = end_[m_];
   m_ = 0;
   n_ = 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Appending an element to the specified row of the sparse matrix.
 *
 * \param i The row index of the new element. The index has to be in the range \f$[0..M-1]\f$.
 * \param j The column index of the new element. The index has to be in the range \f$[0..N-1]\f$.
 * \param value The value of the element to be appended.
 * \return void
 *
 * This function provides a very efficient way to fill a sparse matrix with elements. It appends
 * a new element to the end of the specified row without any additional parameter verification
 * or memory allocation. Therefore it is strictly necessary to keep the following preconditions
 * in mind:
 *
 *  - the index of the new element must be strictly larger than the largest index of non-zero
 *    elements in the specified row of the sparse matrix
 *  - the current number of non-zero elements in row \a i must be smaller than the capacity of
 *    row \a i.
 *
 * Ignoring these preconditions might result in undefined behavior!
 *
 * \b Note: Although append() does not allocate new memory, it still invalidates all iterators
 * returned by the end() functions!
 */
template< typename Type >  // Data type of the sparse matrix
void SparseMatrixMxN<Type>::append( size_t i, size_t j, const Type& value )
{
   pe_USER_ASSERT( i < m_, "Invalid row access index"    );
   pe_USER_ASSERT( j < n_, "Invalid column access index" );
   pe_USER_ASSERT( end_[i] < begin_[i+1], "Not enough reserved space in the current row" );
   pe_USER_ASSERT( begin_[i] == end_[i] || j > ( end_[i]-1 )->index_, "Index is not strictly increasing" );

   end_[i]->value_ = value;
   end_[i]->index_ = j;
   ++end_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting an element into the sparse matrix.
 *
 * \param i The row index of the new element. The index has to be in the range \f$[0..M-1]\f$.
 * \param j The column index of the new element. The index has to be in the range \f$[0..N-1]\f$.
 * \param value The value of the element to be inserted.
 * \return void
 * \exception std::invalid_argument Invalid sparse matrix access index.
 *
 * This function insert a new element into the sparse matrix. However, duplicate elements are
 * not allowed. In case the sparse matrix already contains an element with row index \a i and
 * column index \a j, a \a std::invalid_argument exception is thrown.
 */
template< typename Type >  // Data type of the sparse matrix
Type& SparseMatrixMxN<Type>::insert( size_t i, size_t j, const Type& value )
{
   pe_USER_ASSERT( i < rows()   , "Invalid row access index"    );
   pe_USER_ASSERT( j < columns(), "Invalid column access index" );

   const Iterator pos( std::lower_bound( begin_[i], end_[i], j, FindIndex() ) );

   if( pos != end_[i] && pos->index_ == j )
      throw std::invalid_argument( "Bad access index" );

   if( begin_[i+1] - end_[i] != 0 ) {
      std::copy_backward( pos, end_[i], end_[i]+1 );
      pos->value_ = value;
      pos->index_ = j;
      ++end_[i];

      return pos->value_;
   }
   else if( end_[m_] - begin_[m_] != 0 ) {
      std::copy_backward( pos, end_[m_-1], end_[m_-1]+1 );

      pos->value_ = value;
      pos->index_ = j;

      for( size_t k=i+1; k<m_+1; ++k ) {
         ++begin_[k];
         ++end_[k-1];
      }

      return pos->value_;
   }
   else {
      size_t newCapacity( extendCapacity() );

      Iterator* newBegin = new Iterator[2*m_+2];
      Iterator* newEnd   = newBegin+m_+1;

      newBegin[0] = new Element[newCapacity];

      for( size_t k=0; k<i; ++k ) {
         const size_t nonzeros( end_[k] - begin_[k] );
         const size_t total( begin_[k+1] - begin_[k] );
         newEnd  [k]   = newBegin[k] + nonzeros;
         newBegin[k+1] = newBegin[k] + total;
      }
      newEnd  [i]   = newBegin[i] + ( end_[i] - begin_[i] ) + 1;
      newBegin[i+1] = newBegin[i] + ( begin_[i+1] - begin_[i] ) + 1;
      for( size_t k=i+1; k<m_; ++k ) {
         const size_t nonzeros( end_[k] - begin_[k] );
         const size_t total( begin_[k+1] - begin_[k] );
         newEnd  [k]   = newBegin[k] + nonzeros;
         newBegin[k+1] = newBegin[k] + total;
      }

      newEnd[m_]  = newBegin[0]+newCapacity;

      Iterator tmp = std::copy( begin_[0], pos, newBegin[0] );
      tmp->value_ = value;
      tmp->index_ = j;
      std::copy( pos, end_[m_-1], tmp+1 );

      std::swap( newBegin, begin_ );
      end_ = newEnd;
      delete [] newBegin[0];
      delete [] newBegin;

      return tmp->value_;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Searches for a specific matrix element.
 *
 * \param i The row index of the search element. The index has to be in the range \f$[0..M-1]\f$.
 * \param j The column index of the search element. The index has to be in the range \f$[0..N-1]\f$.
 * \return Iterator to the element in case the index is found, end() iterator otherwise.
 *
 * This function can be used to check whether a specific element is contained in the sparse
 * matrix. It specifically searches for the element with row index \a i and column index \a j.
 * In case the element is found, the function returns an iterator to the element. Otherwise an
 * iterator just past the last non-zero element of row \a i (the end() iterator) is returned.
 * Note that the returned sparse matrix iterator is subject to invalidation due to inserting
 * operations via the subscript operator or the insert() function!
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::Iterator
   SparseMatrixMxN<Type>::find( size_t i, size_t j )
{
   return const_cast<Iterator>( const_cast<const This&>( *this ).find( i, j ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Searches for a specific matrix element.
 *
 * \param i The row index of the search element. The index has to be in the range \f$[0..M-1]\f$.
 * \param j The column index of the search element. The index has to be in the range \f$[0..N-1]\f$.
 * \return Iterator to the element in case the index is found, end() iterator otherwise.
 *
 * This function can be used to check whether a specific element is contained in the sparse
 * matrix. It specifically searches for the element with row index \a i and column index \a j.
 * In case the element is found, the function returns an iterator to the element. Otherwise an
 * iterator just past the last non-zero element of row \a i (the end() iterator) is returned.
 * Note that the returned sparse matrix iterator is subject to invalidation due to inserting
 * operations via the subscript operator or the insert() function!
 */
template< typename Type >  // Data type of the sparse matrix
inline typename SparseMatrixMxN<Type>::ConstIterator
   SparseMatrixMxN<Type>::find( size_t i, size_t j ) const
{
   const Iterator pos( std::lower_bound( begin_[i], end_[i], j, FindIndex() ) );
   if( pos != end_[i] && pos->index_ == j )
      return pos;
   else return end_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the size of the sparse matrix.
 *
 * \param m The new number of rows of the sparse matrix.
 * \param n The new number of columns of the sparse matrix.
 * \param preserve \a true if the old values of the matrix should be preserved, \a false if not.
 * \return void
 *
 * This function resizes the matrix using the given size to \f$ m \times n \f$. During this
 * operation, new dynamic memory may be allocated in case the capacity of the matrix is too
 * small. Therefore this function potentially changes all matrix elements. In order to preserve
 * the old matrix values, the \a preserve flag can be set to \a true.
 */
template< typename Type >  // Data type of the sparse matrix
void SparseMatrixMxN<Type>::resize( size_t m, size_t n, bool preserve )
{
   if( m == m_ && n == n_ ) return;

   if( m > capacity_ )
   {
      Iterator* newBegin( new Iterator[2*m+2] );
      Iterator* newEnd  ( newBegin+m+1 );

      newBegin[0] = begin_[0];

      if( preserve ) {
         for( size_t i=0; i<m_; ++i ) {
            newEnd  [i]   = end_  [i];
            newBegin[i+1] = begin_[i+1];
         }
         for( size_t i=m_; i<m; ++i ) {
            newBegin[i+1] = newEnd[i] = begin_[m_];
         }
      }
      else {
         for( size_t i=0; i<m; ++i ) {
            newBegin[i+1] = newEnd[i] = begin_[0];
         }
      }

      newEnd[m] = end_[m_];

      std::swap( newBegin, begin_ );
      delete [] newBegin;

      end_ = newEnd;
      capacity_ = m;
   }
   else if( m > m_ )
   {
      end_[m] = end_[m_];

      if( preserve ) {
         for( size_t i=0; i<m_; ++i )
            end_[i] = end_[i];
      }
      else {
         for( size_t i=0; i<m_; ++i )
            end_[i] = begin_[i];
      }

      for( size_t i=m_; i<m; ++i )
         begin_[i+1] = end_[i] = begin_[m_];
   }
   else
   {
      if( preserve ) {
         for( size_t i=0; i<m; ++i )
            end_[i] = std::lower_bound( begin_[i], end_[i], n, FindIndex() );
      }
      else {
         for( size_t i=0; i<m; ++i )
            end_[i] = begin_[i];
      }

      end_[m] = end_[m_];
   }

   m_ = m;
   n_ = n;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of the sparse matrix.
 *
 * \param nonzeros The new minimum capacity of the sparse matrix.
 * \return void
 *
 * This function increases the capacity of the sparse matrix to at least \a nonzeros elements.
 * The current values of the matrix elements and the individual capacities of the matrix rows
 * are preserved.
 */
template< typename Type >  // Data type of the sparse matrix
void SparseMatrixMxN<Type>::reserve( size_t nonzeros )
{
   if( nonzeros > capacity() )
      reserveElements( nonzeros );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of a specific row of the sparse matrix.
 *
 * \param i The row index of the new element. The index has to be in the range \f$[0..M-1]\f$.
 * \param nonzeros The new minimum capacity of the specified row.
 * \return void
 *
 * This function increases the capacity of row \a i of the sparse matrix to at least \a nonzeros
 * elements. The current values of the sparse matrix and all other individual row capacities are
 * preserved.
 */
template< typename Type >  // Data type of the sparse matrix
void SparseMatrixMxN<Type>::reserve( size_t i, size_t nonzeros )
{
   pe_USER_ASSERT( i < rows(), "Invalid row access index" );

   const size_t current( capacity(i) );

   if( current >= nonzeros ) return;

   const ptrdiff_t additional( nonzeros - current );

   if( end_[m_] - begin_[m_] < additional )
   {
      const size_t newCapacity( begin_[m_] - begin_[0] + additional );
      pe_INTERNAL_ASSERT( newCapacity > capacity(), "Invalid capacity value" );

      Iterator* newBegin( new Iterator[2*m_+2] );
      Iterator* newEnd  ( newBegin+m_+1 );

      newBegin[0 ] = new Element[newCapacity];
      newEnd  [m_] = newBegin[0]+newCapacity;

      for( size_t k=0; k<i; ++k ) {
         newEnd  [k  ] = std::copy( begin_[k], end_[k], newBegin[k] );
         newBegin[k+1] = newBegin[k] + capacity(k);
      }
      newEnd  [i  ] = std::copy( begin_[i], end_[i], newBegin[i] );
      newBegin[i+1] = newBegin[i] + nonzeros;
      for( size_t k=i+1; k<m_; ++k ) {
         newEnd  [k  ] = std::copy( begin_[k], end_[k], newBegin[k] );
         newBegin[k+1] = newBegin[k] + capacity(k);
      }

      pe_INTERNAL_ASSERT( newBegin[m_] == newEnd[m_], "Invalid pointer calculations" );

      std::swap( newBegin, begin_ );
      delete [] newBegin[0];
      delete [] newBegin;
      end_ = newEnd;
   }
   else
   {
      begin_[m_] += additional;
      for( size_t j=m_-1; j>i; --j ) {
         begin_[j]  = std::copy_backward( begin_[j], end_[j], end_[j]+additional );
         end_  [j] += additional;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transposing the matrix.
 *
 * \return Reference to the transposed matrix.
 */
template< typename Type >  // Data type of the sparse matrix
SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::transpose()
{
   SparseMatrixMxN tmp( trans( *this ) );
   swap( tmp );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the sparse matrix.
 *
 * \return Reference to the inverted sparse matrix.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data types will result in a compile time error.
 */
template< typename Type >  // Data type of the sparse matrix
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::invert()
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the inverse of the sparse matrix.
 *
 * \return The inverse of the sparse matrix.
 *
 * \b Note: This function is only defined for matrices of floating point type. The attempt to
 * use this function with matrices of integral data types will result in a compile time error.
 */
template< typename Type >  // Data type of the matrix
inline const SparseMatrixMxN<Type> SparseMatrixMxN<Type>::getInverse() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   return SparseMatrixMxN();
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
template< typename Type >  // Data type of the sparse matrix
bool SparseMatrixMxN<Type>::isDiagonal() const
{
   for( size_t i=0; i<rows(); ++i ) {
      for( ConstIterator element=begin_[i]; element!=end_[i]; ++element )
         if( element->index_ != i && !isDefault( element->value_ ) )
            return false;
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
inline bool SparseMatrixMxN<Type>::isSymmetric() const
{
   if( m_ != n_ ) return false;

   for( size_t i=0; i<rows(); ++i ) {
      for( ConstIterator element=begin_[i]; element!=end_[i]; ++element )
      {
         const size_t index( element->index_ );

         if( isDefault( element->value_ ) )
            continue;

         const Iterator pos( std::lower_bound( begin_[index], end_[index], i, FindIndex() ) );
         if( pos == end_[index] || pos->index_ != i || !equal( pos->value_, element->value_ ) )
            return false;
      }
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the sparse matrix by the scalar value \a scalar (\f$ A=B*s \f$).
 *
 * \param scalar The scalar value for the matrix scaling.
 * \return Reference to the sparse matrix.
 */
template< typename Type >   // Data type of the sparse matrix
template< typename Other >  // Data type of the scalar value
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::scale( Other scalar )
{
   for( size_t i=0; i<m_; ++i )
      for( Iterator element=begin_[i]; element!=end_[i]; ++element )
         element->value_ *= scalar;

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling the diagonal of the sparse matrix by the scalar value \a scalar.
 *
 * \param scalar The scalar value for the diagonal scaling.
 * \return Reference to the sparse matrix.
 */
template< typename Type >   // Data type of the sparse matrix
template< typename Other >  // Data type of the scalar value
inline SparseMatrixMxN<Type>& SparseMatrixMxN<Type>::scaleDiagonal( Other scalar )
{
   const size_t size( pe::min( m_, n_ ) );

   for( size_t i=0; i<size; ++i ) {
      Iterator pos = std::lower_bound( begin_[i], end_[i], i, FindIndex() );
      if( pos != end_[i] && pos->index_ == i )
         pos->value_ *= scalar;
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two sparse matrices.
 *
 * \param sm The sparse matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the sparse matrix
inline void SparseMatrixMxN<Type>::swap( SparseMatrixMxN& sm ) /* throw() */
{
   std::swap( m_, sm.m_ );
   std::swap( n_, sm.n_ );
   std::swap( capacity_, sm.capacity_ );
   std::swap( begin_, sm.begin_ );
   std::swap( end_  , sm.end_   );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating a new matrix capacity.
 *
 * \return The new sparse matrix capacity.
 *
 * This function calculates a new matrix capacity based on the current capacity of the sparse
 * matrix. Note that the new capacity is restricted to the interval \f$[7..M \cdot N]\f$.
 */
template< typename Type >  // Data type of the sparse matrix
inline size_t SparseMatrixMxN<Type>::extendCapacity() const
{
   size_t nonzeros( 2*capacity()+1 );
   nonzeros = pe::max( nonzeros, size_t(7) );
   nonzeros = pe::min( nonzeros, m_*n_ );

   pe_INTERNAL_ASSERT( nonzeros > capacity(), "Invalid capacity value" );

   return nonzeros;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reserving the specified number of sparse matrix elements.
 *
 * \param nonzeros The number of matrix elements to be reserved.
 * \return void
 */
template< typename Type >  // Data type of the sparse matrix
void SparseMatrixMxN<Type>::reserveElements( size_t nonzeros )
{
   Iterator* newBegin = new Iterator[2*m_+2];
   Iterator* newEnd   = newBegin+m_+1;

   newBegin[0] = new Element[nonzeros];

   for( size_t k=0; k<m_; ++k ) {
      pe_INTERNAL_ASSERT( begin_[k] <= end_[k], "Invalid row pointers" );
      newEnd  [k]   = std::copy( begin_[k], end_[k], newBegin[k] );
      newBegin[k+1] = newBegin[k] + ( begin_[k+1] - begin_[k] );
   }

   newEnd[m_] = newBegin[0]+nonzeros;

   std::swap( newBegin, begin_ );
   delete [] newBegin[0];
   delete [] newBegin;
   end_ = newEnd;
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
template< typename Type >   // Data type of the sparse matrix
template< typename Other >  // Data type of the foreign expression
inline bool SparseMatrixMxN<Type>::isAliased( const Other* alias ) const
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
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void SparseMatrixMxN<Type>::assign( const DenseMatrix<MT>& rhs )
{
   size_t nonzeros( 0 );

   for( size_t i=1; i<=m_; ++i )
      begin_[i] = end_[i] = end_[m_];

   for( size_t i=0; i<m_; ++i )
   {
      begin_[i] = end_[i] = begin_[0]+nonzeros;

      for( size_t j=0; j<n_; ++j ) {
         if( !isDefault( (~rhs)(i,j) ) ) {
            if( nonzeros++ == capacity() ) {
               reserveElements( extendCapacity() );
               for( size_t k=i+1; k<=m_; ++k )
                  begin_[k] = end_[k] = end_[m_];
            }
            append( i, j, (~rhs)(i,j) );
         }
      }
   }

   begin_[m_] = begin_[0]+nonzeros;
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
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void SparseMatrixMxN<Type>::assign( const SparseMatrix<MT>& rhs )
{
   for( size_t i=0; i<(~rhs).rows(); ++i ) {
      begin_[i+1] = end_[i] = std::copy( (~rhs).begin(i), (~rhs).end(i), begin_[i] );
   }
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
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void SparseMatrixMxN<Type>::addAssign( const DenseMatrix<MT>& rhs )
{
   SparseMatrixMxN tmp( *this + (~rhs) );
   swap( tmp );
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
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void SparseMatrixMxN<Type>::addAssign( const SparseMatrix<MT>& rhs )
{
   SparseMatrixMxN tmp( *this + (~rhs) );
   swap( tmp );
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
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void SparseMatrixMxN<Type>::subAssign( const DenseMatrix<MT>& rhs )
{
   SparseMatrixMxN tmp( *this - (~rhs) );
   swap( tmp );
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
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void SparseMatrixMxN<Type>::subAssign( const SparseMatrix<MT>& rhs )
{
   SparseMatrixMxN tmp( *this - (~rhs) );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a dense matrix.
 *
 * \param rhs The right-hand side dense matrix to be multiplied.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side dense matrix
inline void SparseMatrixMxN<Type>::multAssign( const DenseMatrix<MT>& rhs )
{
   SparseMatrixMxN tmp( *this * (~rhs) );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a sparse matrix.
 *
 * \param rhs The right-hand side sparse matrix to be multiplied.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type >  // Data type of the sparse matrix
template< typename MT >    // Type of the right-hand side sparse matrix
inline void SparseMatrixMxN<Type>::multAssign( const SparseMatrix<MT>& rhs )
{
   SparseMatrixMxN tmp( *this * (~rhs) );
   swap( tmp );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name SparseMatrixMxN operators */
//@{
template< typename Type >
inline bool isnan( const SparseMatrixMxN<Type>& m );

template< typename Type >
inline void reset( SparseMatrixMxN<Type>& m );

template< typename Type >
inline void clear( SparseMatrixMxN<Type>& m );

template< typename Type >
inline bool isDefault( const SparseMatrixMxN<Type>& m );

template< typename Type >
inline const SparseMatrixMxN<Type> inv( const SparseMatrixMxN<Type>& m );

template< typename Type >
inline const SparseMatrixMxN<typename MathTrait<Type,Type>::MultType> sq( const SparseMatrixMxN<Type>& m );

template< typename Type >
inline void swap( SparseMatrixMxN<Type>& a, SparseMatrixMxN<Type>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks the given sparse matrix for not-a-number elements.
 * \ingroup sparse_matrix_MxN
 *
 * \param m The sparse matrix to be checked for not-a-number elements.
 * \return \a true if at least one element of the sparse matrix is not-a-number, \a false otherwise.
 */
template< typename Type >  // Data type of the sparse matrix
inline bool isnan( const SparseMatrixMxN<Type>& m )
{
   typedef typename SparseMatrixMxN<Type>::ConstIterator  ConstIterator;

   for( size_t i=0; i<m.rows(); ++i ) {
      for( ConstIterator element=m.begin(i); element!=m.end(i); ++element )
         if( isnan( *element ) ) return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given sparse matrix.
 * \ingroup sparse_matrix_MxN
 *
 * \param m The sparse matrix to be resetted.
 * \return void
 */
template< typename Type >  // Data type of the sparse matrix
inline void reset( SparseMatrixMxN<Type>& m )
{
   m.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given sparse matrix.
 * \ingroup sparse_matrix_MxN
 *
 * \param m The sparse matrix to be cleared.
 * \return void
 */
template< typename Type >  // Data type of the sparse matrix
inline void clear( SparseMatrixMxN<Type>& m )
{
   m.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given sparse matrix is in default state.
 * \ingroup sparse_matrix_MxN
 *
 * \param m The sparse matrix to be tested for its default state.
 * \return \a true in case the given matrix is component-wise zero, \a false otherwise.
 */
template< typename Type >  // Data type of the sparse matrix
inline bool isDefault( const SparseMatrixMxN<Type>& m )
{
   typedef typename SparseMatrixMxN<Type>::ConstIterator  ConstIterator;

   for( size_t i=0; i<m.rows(); ++i ) {
      for( ConstIterator element=m.begin(i); element!=m.end(i); ++element )
         if( !isDefault( element->value() ) ) return false;
   }
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the given sparse matrix.
 * \ingroup sparse_matrix_MxN
 *
 * \param m The sparse matrix to be inverted.
 * \return The inverse of the matrix.
 *
 * This function returns the inverse of the given sparse matrix. It has the same effect as
 * calling the getInverse() member function of the matrix.
 */
template< typename Type >  // Data type of the matrix
inline const SparseMatrixMxN<Type> inv( const SparseMatrixMxN<Type>& m )
{
   return m.getInverse();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Squaring the given sparse matrix.
 * \ingroup sparse_matrix_MxN
 *
 * \param m The sparse matrix to be squared.
 * \return The result of the square operation.
 *
 * This function squares the given sparse matrix \a m. This function has the same effect as
 * multiplying the matrix with itself (\f$ m * m \f$).
 */
template< typename Type >  // Data type of the matrix
inline const SparseMatrixMxN<typename MathTrait<Type,Type>::MultType> sq( const SparseMatrixMxN<Type>& m )
{
   return m * m;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two sparse matrices.
 * \ingroup sparse_matrix_MxN
 *
 * \param a The first sparse matrix to be swapped.
 * \param b The second sparse matrix to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type >  // Data type of the sparse matrices
inline void swap( SparseMatrixMxN<Type>& a, SparseMatrixMxN<Type>& b ) /* throw() */
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
struct MathTrait< SparseMatrixMxN<T1>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::DivType  >  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T2 );
};

template< typename T1, typename T2 >
struct MathTrait< T1, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T1 );
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, Vector2<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,true>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, Vector3<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector3<T1,true>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, Vector6<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector6<T1,true>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                 HighType;
   typedef INVALID_NUMERICAL_TYPE                                 LowType;
   typedef INVALID_NUMERICAL_TYPE                                 AddType;
   typedef INVALID_NUMERICAL_TYPE                                 SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                 DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                HighType;
   typedef INVALID_NUMERICAL_TYPE                                LowType;
   typedef INVALID_NUMERICAL_TYPE                                AddType;
   typedef INVALID_NUMERICAL_TYPE                                SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE                                       HighType;
   typedef INVALID_NUMERICAL_TYPE                                       LowType;
   typedef INVALID_NUMERICAL_TYPE                                       AddType;
   typedef INVALID_NUMERICAL_TYPE                                       SubType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::MultType, false >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                       DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                                      HighType;
   typedef INVALID_NUMERICAL_TYPE                                      LowType;
   typedef INVALID_NUMERICAL_TYPE                                      AddType;
   typedef INVALID_NUMERICAL_TYPE                                      SubType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::MultType, true >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                      DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, Matrix2x2<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix2x2<T1>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, Matrix3x3<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix3x3<T1>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix3x3< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, Matrix6x6<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Matrix6x6<T1>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef Matrix6x6< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, MatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< MatrixMxN<T1>, SparseMatrixMxN<T2> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseMatrixMxN<T1>, SparseMatrixMxN<T2> >
{
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::HighType >  HighType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::LowType  >  LowType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::AddType  >  AddType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::SubType  >  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
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
/*!\brief Sparse MxN real matrix.
 * \ingroup sparse_matrix_MxN
 */
typedef SparseMatrixMxN<real>  SMatN;
//*************************************************************************************************

} // namespace pe

#endif
