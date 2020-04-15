//=================================================================================================
/*!
 *  \file pe/math/SparseVectorN.h
 *  \brief Implementation of an arbitrarily sized sparse vector
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

#ifndef _PE_MATH_SPARSEVECTORN_H_
#define _PE_MATH_SPARSEVECTORN_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <pe/math/CMathTrait.h>
#include <pe/math/Functions.h>
#include <pe/math/MathTrait.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/sparse/SparseElement.h>
#include <pe/math/sparse/VectorAccessProxy.h>
#include <pe/math/SparseMatrixMxN.h>
#include <pe/math/SparseVector.h>
#include <pe/math/TransposeTrait.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>
#include <pe/util/constraints/Const.h>
#include <pe/util/constraints/FloatingPoint.h>
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

template< typename, bool > struct DenseVector;
template< typename >       class  MatrixMxN;
template< typename >       class  SparseMatrixMxN;
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
/*!\defgroup sparse_vector_N SparseVectorN
 * \ingroup sparse_vector
 */
/*!\brief Efficient implementation of an arbitrary sized sparse vector.
 * \ingroup sparse_vector_N
 *
 * The SparseVectorN class is the representation of an arbitrarily sized sparse vector with
 * a maximum of N non-zero, dynamically allocated elements. The order of the elements is as
 * following:

                             \f[\left(\begin{array}{*{5}{c}}
                             0 & 1 & 2 & \cdots & N-1 \\
                             \end{array}\right)\f]

 * Inserting/accessing elements in a sparse vector can be done by several alternative functions.
 * The following example demonstrates all options:

   \code
   // Creating a sparse vector of size 5
   SparseVectorN<double> v( 5 );

   // The subscript operator provides access to all possible elements of the sparse vector,
   // including the zero elements. In case the subscript operator is used to access an element
   // that is currently not stored in the sparse vector, the element is inserted into the
   // vector.
   v[0] = 2.0;

   // An alternative for inserting elements into the vector is the insert() function. However,
   // it inserts the element only in case the element is not already contained in the vector.
   v.insert( 2, 3.7 );

   // A very efficient way to add new elements to a sparse vector is the append() function.
   // Note that append() requires that the appended element's index is strictly larger than
   // the currently largest non-zero index of the vector and that the vector's capacity
   // is large enough to hold the new element.
   v.reserve( 3 );
   v.append( 4, -2.1 );

   // In order to traverse all non-zero elements currently stored in the vector, the begin()
   // and end() functions can be used.
   for( SparseVectorN<double>::Iterator i=v.begin(); i!=v.end(); ++i )
      ... = i->value();  // Access to the value of the non-zero element
      ... = i->index();  // Access to the index of the non-zero element
   \endcode

 * SparseVectorN can be used with any non-cv-qualified element type. The arithmetic operators for
 * vector/vector and vector/element operations with the same element type work for any element
 * type as long as the element type supports the arithmetic operation. Arithmetic operations
 * between vectors and elements of different element types are only supported for all data types
 * supported by the MathTrait class template (for details see the MathTrait class description).

   \code
   SparseVectorN< double > a, b, c;
   SparseVectorN< float  > d;
   SparseVectorN< std::complex<double> > e, f, g;
   SparseVectorN< std::complex<float>  > h;

   ...         // Appropriate resizing

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type      // Data type of the vector
        , bool TF = false >  // Transposition flag
class SparseVectorN : public SparseVector< SparseVectorN<Type,TF>, TF >
{
private:
   //**Type definitions****************************************************************************
   typedef SparseElement<Type>  ElementBase;  //!< Base class for the sparse vector element.
   //**********************************************************************************************

   //**Private class Element***********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Index-value-pair for the SparseVectorN class.
   */
   struct Element : public ElementBase
   {
      using ElementBase::operator=;
      friend class SparseVectorN;
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
   typedef SparseVectorN<Type,TF>   This;            //!< Type of this SparseVectorN instance.
   typedef This                     ResultType;      //!< Result type for expression template evaluations.
   typedef Type                     ElementType;     //!< Type of the sparse vector elements.
   typedef const SparseVectorN&     CompositeType;   //!< Data type for composite expression templates.
   typedef VectorAccessProxy<This>  Reference;       //!< Reference to a sparse vector value.
   typedef const Type&              ConstReference;  //!< Reference to a constant sparse vector value.
   typedef Element*                 Iterator;        //!< Iterator over non-constant elements.
   typedef const Element*           ConstIterator;   //!< Iterator over constant elements.

   //! Transpose type for expression template evaluations.
   typedef SparseVectorN< typename TransposeTrait<Type>::Type, !TF >  TransposeType;

   //! Sparse vector length return type.
   /*! Return type of the SparseVectorN<Type>::length function. */
   typedef typename CMathTrait<Type>::Type  LengthType;
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                           explicit inline SparseVectorN();
                           explicit inline SparseVectorN( size_t size );
                           explicit inline SparseVectorN( size_t size, size_t nonzeros );
                                    inline SparseVectorN( const SparseVectorN& sv );
   template< typename VT >          inline SparseVectorN( const DenseVector<VT,TF>&  dv );
   template< typename VT >          inline SparseVectorN( const SparseVector<VT,TF>& sv );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~SparseVectorN();
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
                           inline SparseVectorN& operator= ( const SparseVectorN& rhs );
   template< typename VT > inline SparseVectorN& operator= ( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline SparseVectorN& operator= ( const SparseVector<VT,TF>& rhs );
                           inline Reference      operator[]( size_t index );
                           inline ConstReference operator[]( size_t index ) const;
   template< typename VT > inline SparseVectorN& operator+=( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline SparseVectorN& operator+=( const SparseVector<VT,TF>& rhs );
   template< typename VT > inline SparseVectorN& operator-=( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline SparseVectorN& operator-=( const SparseVector<VT,TF>& rhs );
   template< typename VT > inline SparseVectorN& operator*=( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline SparseVectorN& operator*=( const SparseVector<VT,TF>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, SparseVectorN >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, SparseVectorN >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t              size()          const;
                              inline size_t              capacity()      const;
                              inline size_t              nonZeros()      const;
                              inline Iterator            begin();
                              inline ConstIterator       begin()         const;
                              inline Iterator            end();
                              inline ConstIterator       end()           const;
                              inline void                reset();
                              inline void                clear();
                                     void                append( size_t index, const Type& value );
                                     Type&               insert( size_t index, const Type& value );
                              inline Iterator            find  ( size_t index );
                              inline ConstIterator       find  ( size_t index ) const;
                              inline void                resize( size_t n, bool preserve=true );
                              inline void                reserve( size_t n );
                              inline LengthType          length()        const;
                              inline const Type          sqrLength()     const;
                              inline SparseVectorN&      normalize();
                              inline const SparseVectorN getNormalized() const;
   template< typename Other > inline SparseVectorN&      scale( Other scalar );
                              inline const Type          min()           const;
                              inline const Type          max()           const;
                              inline void                swap( SparseVectorN& sv ) /* throw() */;
   //@}
   //**********************************************************************************************

   //**Expression template evaluation functions****************************************************
   /*!\name Expression template evaluation functions */
   //@{
   template< typename Other > inline bool isAliased ( const Other* alias ) const;
   template< typename VT >    inline void assign    ( const DenseVector <VT,TF>& rhs );
   template< typename VT >    inline void assign    ( const SparseVector<VT,TF>& rhs );
   template< typename VT >    inline void addAssign ( const DenseVector <VT,TF>& rhs );
   template< typename VT >    inline void addAssign ( const SparseVector<VT,TF>& rhs );
   template< typename VT >    inline void subAssign ( const DenseVector <VT,TF>& rhs );
   template< typename VT >    inline void subAssign ( const SparseVector<VT,TF>& rhs );
   template< typename VT >    inline void multAssign( const DenseVector <VT,TF>& rhs );
   template< typename VT >    inline void multAssign( const SparseVector<VT,TF>& rhs );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline size_t extendCapacity() const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t size_;             //!< The current size/dimension of the sparse vector.
   size_t capacity_;         //!< The maximum capacity of the sparse vector.
   Iterator begin_;          //!< Pointer to the first non-zero element of the sparse vector.
   Iterator end_;            //!< Pointer one past the last non-zero element of the sparse vector.

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

template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
const Type SparseVectorN<Type,TF>::zero_ = Type();




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for SparseVectorN.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline SparseVectorN<Type,TF>::SparseVectorN()
   : size_    ( 0 )     // The current size/dimension of the sparse vector
   , capacity_( 0 )     // The maximum capacity of the sparse vector
   , begin_   ( NULL )  // Pointer to the first non-zero element of the sparse vector
   , end_     ( NULL )  // Pointer to the last non-zero element of the sparse vector
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a sparse vector of size \a n.
 *
 * \param n The size of the vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline SparseVectorN<Type,TF>::SparseVectorN( size_t n )
   : size_    ( n )     // The current size/dimension of the sparse vector
   , capacity_( 0 )     // The maximum capacity of the sparse vector
   , begin_   ( NULL )  // Pointer to the first non-zero element of the sparse vector
   , end_     ( NULL )  // Pointer to the last non-zero element of the sparse vector
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a sparse vector of size \a n.
 *
 * \param n The size of the vector.
 * \param nonzeros The number of expected non-zero elements.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline SparseVectorN<Type,TF>::SparseVectorN( size_t n, size_t nonzeros )
   : size_    ( n )                       // The current size/dimension of the sparse vector
   , capacity_( nonzeros )                // The maximum capacity of the sparse vector
   , begin_   ( new Element[capacity_] )  // Pointer to the first non-zero element of the sparse vector
   , end_     ( begin_ )                  // Pointer to the last non-zero element of the sparse vector
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for SparseVectorN.
 *
 * \param sv Sparse vector to be copied.
 *
 * The copy constructor is explicitly defined due to the required dynamic memory management
 * and in order to enable/facilitate NRV optimization.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline SparseVectorN<Type,TF>::SparseVectorN( const SparseVectorN& sv )
   : size_    ( sv.size_ )                // The current size/dimension of the sparse vector
   , capacity_( sv.nonZeros() )           // The maximum capacity of the sparse vector
   , begin_   ( new Element[capacity_] )  // Pointer to the first non-zero element of the sparse vector
   , end_     ( begin_+capacity_ )        // Pointer to the last non-zero element of the sparse vector
{
   std::copy( sv.begin_, sv.end_, begin_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from dense vectors.
 *
 * \param dv Dense vector to be copied.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the foreign dense vector
inline SparseVectorN<Type,TF>::SparseVectorN( const DenseVector<VT,TF>& dv )
   : size_    ( (~dv).size() )  // The current size/dimension of the sparse vector
   , capacity_( 0 )             // The maximum capacity of the sparse vector
   , begin_   ( NULL )          // Pointer to the first non-zero element of the sparse vector
   , end_     ( NULL )          // Pointer to the last non-zero element of the sparse vector
{
   using pe::assign;
   assign( *this, ~dv );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different sparse vectors.
 *
 * \param sv Sparse vector to be copied.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the foreign sparse vector
inline SparseVectorN<Type,TF>::SparseVectorN( const SparseVector<VT,TF>& sv )
   : size_    ( (~sv).size() )            // The current size/dimension of the sparse vector
   , capacity_( (~sv).nonZeros() )        // The maximum capacity of the sparse vector
   , begin_   ( new Element[capacity_] )  // Pointer to the first non-zero element of the sparse vector
   , end_     ( begin_ )                  // Pointer to the last non-zero element of the sparse vector
{
   using pe::assign;
   assign( *this, ~sv );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for SparseVectorN.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline SparseVectorN<Type,TF>::~SparseVectorN()
{
   delete [] begin_;
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Copy assignment operator for SparseVectorN.
 *
 * \param rhs Sparse vector to be copied.
 * \return Reference to the assigned sparse vector.
 *
 * The sparse vector is resized according to the given sparse vector and initialized as a
 * copy of this vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator=( const SparseVectorN& rhs )
{
   if( &rhs == this ) return *this;

   const size_t nonzeros( rhs.nonZeros() );

   if( nonzeros > capacity_ ) {
      Iterator newBegin( new Element[nonzeros] );
      end_ = std::copy( rhs.begin_, rhs.end_, newBegin );
      std::swap( begin_, newBegin );
      delete [] newBegin;

      size_     = rhs.size_;
      capacity_ = nonzeros;
   }
   else {
      end_  = std::copy( rhs.begin_, rhs.end_, begin_ );
      size_ = rhs.size_;
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for dense vectors.
 *
 * \param rhs Dense vector to be copied.
 * \return Reference to the assigned sparse vector.
 *
 * The vector is resized according to the given dense vector and initialized as a copy of
 * this vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator=( const DenseVector<VT,TF>& rhs )
{
   using pe::assign;

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      SparseVectorN tmp( rhs );
      swap( tmp );
   }
   else {
      size_ = (~rhs).size();
      end_  = begin_;
      assign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different sparse vectors.
 *
 * \param rhs Sparse vector to be copied.
 * \return Reference to the assigned sparse vector.
 *
 * The vector is resized according to the given sparse vector and initialized as a copy of
 * this vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator=( const SparseVector<VT,TF>& rhs )
{
   using pe::assign;

   if( ( IsExpression<VT>::value && (~rhs).isAliased( this ) ) ||
       (~rhs).nonZeros() > capacity_ ) {
      SparseVectorN tmp( rhs );
      swap( tmp );
   }
   else {
      size_ = (~rhs).size();
      end_  = begin_;
      assign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the sparse vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 *
 * This function returns a reference to the accessed value at position \a index. In case the
 * sparse vector does not yet store an element for index \a index, a new element is inserted
 * into the sparse vector. An alternative for traversing the non-zero elements of the sparse
 * vector are the begin() and end() functions.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::Reference
   SparseVectorN<Type,TF>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < size_, "Invalid sparse vector access index" );

   return Reference( *this, index );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the sparse vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::ConstReference
   SparseVectorN<Type,TF>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < size_, "Invalid sparse vector access index" );

   const Iterator pos( std::lower_bound( begin_, end_, index, FindIndex() ) );

   if( pos == end_ || pos->index_ != index )
      return zero_;
   else
      return pos->value_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a dense vector (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side dense vector to be added to the sparse vector.
 * \return Reference to the sparse vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator+=( const DenseVector<VT,TF>& rhs )
{
   using pe::addAssign;
   addAssign( *this, ~rhs );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a sparse vector (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side sparse vector to be added to the sparse vector.
 * \return Reference to the sparse vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator+=( const SparseVector<VT,TF>& rhs )
{
   using pe::addAssign;
   addAssign( *this, ~rhs );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a dense vector
 *        (\f$ \vec{a}-=\vec{b} \f$).
 *
 * \param rhs The right-hand side dense vector to be subtracted from the sparse vector.
 * \return Reference to the sparse vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator-=( const DenseVector<VT,TF>& rhs )
{
   using pe::subAssign;
   subAssign( *this, ~rhs );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a sparse vector
 *        (\f$ \vec{a}-=\vec{b} \f$).
 *
 * \param rhs The right-hand side sparse vector to be subtracted from the sparse vector.
 * \return Reference to the sparse vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator-=( const SparseVector<VT,TF>& rhs )
{
   using pe::subAssign;
   subAssign( *this, ~rhs );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a dense vector
 *        (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side dense vector to be multiplied with the sparse vector.
 * \return Reference to the sparse vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator*=( const DenseVector<VT,TF>& rhs )
{
   using pe::multAssign;
   multAssign( *this, ~rhs );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a sparse vector
 *        (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side sparse vector to be multiplied with the sparse vector.
 * \return Reference to the sparse vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::operator*=( const SparseVector<VT,TF>& rhs )
{
   using pe::multAssign;
   multAssign( *this, ~rhs );
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication between a sparse vector and
 *        a scalar value (\f$ \vec{a}*=s \f$).
 *
 * \param rhs The right-hand side scalar value for the multiplication.
 * \return Reference to the sparse vector.
 *
 * This operator can only be used for built-in data types. Additionally, the elements of the
 * sparse vector must support the multiplication assignment operator for the given scalar
 * built-in data type.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, SparseVectorN<Type,TF> >::Type&
   SparseVectorN<Type,TF>::operator*=( Other rhs )
{
   for( Iterator element=begin_; element<end_; ++element )
      element->value_ *= rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division assignment operator for the division of a sparse vector by a scalar value
 *        (\f$ \vec{a}/=s \f$).
 *
 * \param rhs The right-hand side scalar value for the division.
 * \return Reference to the sparse vector.
 *
 * This operator can only be used for built-in data types. Additionally, the elements of the
 * sparse vector must either support the multiplication assignment operator for the given
 * floating point data type or the division assignment operator for the given integral data
 * type.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, SparseVectorN<Type,TF> >::Type&
   SparseVectorN<Type,TF>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      for( Iterator element=begin_; element!=end_; ++element )
         element->value_ = static_cast<Type>( static_cast<DT>( element->value_ ) * tmp );
   }
   else {
      for( Iterator element=begin_; element!=end_; ++element )
         element->value_ /= rhs;
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
/*!\brief Returns the current size/dimension of the sparse vector.
 *
 * \return The size of the sparse vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t SparseVectorN<Type,TF>::size() const
{
   return size_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum capacity of the sparse vector.
 *
 * \return The capacity of the sparse vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t SparseVectorN<Type,TF>::capacity() const
{
   return capacity_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of non-zero elements in the sparse vector.
 *
 * \return The number of non-zero elements in the sparse vector.
 *
 * Note that the number of non-zero elements is always smaller than the current size of the
 * sparse vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t SparseVectorN<Type,TF>::nonZeros() const
{
   return end_ - begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first non-zero element of the sparse vector.
 *
 * \return Iterator to the first non-zero element of the sparse vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::Iterator SparseVectorN<Type,TF>::begin()
{
   return Iterator( begin_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first non-zero element of the sparse vector.
 *
 * \return Iterator to the first non-zero element of the sparse vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::ConstIterator SparseVectorN<Type,TF>::begin() const
{
   return ConstIterator( begin_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last non-zero element of the sparse vector.
 *
 * \return Iterator just past the last non-zero element of the sparse vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::Iterator SparseVectorN<Type,TF>::end()
{
   return Iterator( end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last non-zero element of the sparse vector.
 *
 * \return Iterator just past the last non-zero element of the sparse vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::ConstIterator SparseVectorN<Type,TF>::end() const
{
   return ConstIterator( end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void SparseVectorN<Type,TF>::reset()
{
   end_ = begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the sparse vector.
 *
 * \return void
 *
 * After the clear() function, the size of the sparse vector is 0.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void SparseVectorN<Type,TF>::clear()
{
   size_ = 0;
   end_  = begin_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Appending an element to the sparse vector.
 *
 * \param index The index of the new element. The index has to be in the range \f$[0..N-1]\f$.
 * \param value The value of the element to be appended.
 * \return void
 *
 * This function provides a very efficient way to fill a sparse vector with elements. It
 * appends a new element to the end of the sparse vector without any additional check or
 * memory allocation. Therefore it is strictly necessary to keep the following preconditions
 * in mind:
 *
 *  - the index of the new element must be strictly larger than the largest index of non-zero
 *    elements in the sparse vector
 *  - the current number of non-zero elements must be smaller than the capacity of the vector
 *
 * Ignoring these preconditions might result in undefined behavior!
 *
 * \b Note: Although append() does not allocate new memory, it still invalidates all iterators
 * returned by the end() functions!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
void SparseVectorN<Type,TF>::append( size_t index, const Type& value )
{
   pe_USER_ASSERT( index < size_, "Invalid sparse vector access index" );
   pe_USER_ASSERT( nonZeros() < capacity(), "Not enough reserved space" );
   pe_USER_ASSERT( begin_ == end_ || (end_-1)->index_ < index, "Index is not strictly increasing" );

   end_->value_ = value;
   end_->index_ = index;
   ++end_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inserting an element into the sparse vector.
 *
 * \param index The index of the new element. The index has to be in the range \f$[0..N-1]\f$.
 * \param value The value of the element to be inserted.
 * \return Reference to the inserted value.
 * \exception std::invalid_argument Invalid sparse vector access index.
 *
 * This function insert a new element into the sparse vector. However, duplicate elements are
 * not allowed. In case the sparse vector already contains an element with index \a index, a
 * \a std::invalid_argument exception is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
Type& SparseVectorN<Type,TF>::insert( size_t index, const Type& value )
{
   pe_USER_ASSERT( index < size_, "Invalid sparse vector access index" );

   const Iterator pos( std::lower_bound( begin_, end_, index, FindIndex() ) );

   if( pos != end_ && pos->index_ == index )
      throw std::invalid_argument( "Bad access index" );

   if( nonZeros() != capacity_ ) {
      std::copy_backward( pos, end_, end_+1 );
      pos->value_ = value;
      pos->index_ = index;
      ++end_;

      return pos->value_;
   }
   else {
      size_t newCapacity( extendCapacity() );

      Iterator newBegin = new Element[newCapacity];
      Iterator tmp      = std::copy( begin_, pos, newBegin );
      tmp->value_ = value;
      tmp->index_ = index;
      end_ = std::copy( pos, end_, tmp+1 );

      std::swap( newBegin, begin_ );
      delete [] newBegin;
      capacity_ = newCapacity;

      return tmp->value_;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Searches for a specific vector element.
 *
 * \param index The index of the search element. The index has to be in the range \f$[0..N-1]\f$.
 * \return Iterator to the element in case the index is found, end() iterator otherwise.
 *
 * This function can be used to check whether a specific element is contained in the sparse
 * vector. It specifically searches for the element with index \a index. In case the element
 * is found, the function returns an iterator to the element. Otherwise an iterator just past
 * the last non-zero element of the sparse vector (the end() iterator) is returned. Note that
 * the returned sparse vector iterator is subject to invalidation due to inserting operations
 * via the subscript operator or the insert() function!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::Iterator SparseVectorN<Type,TF>::find( size_t index )
{
   return const_cast<Iterator>( const_cast<const This&>( *this ).find( index ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Searches for a specific vector element.
 *
 * \param index The index of the search element. The index has to be in the range \f$[0..N-1]\f$.
 * \return Iterator to the element in case the index is found, end() iterator otherwise.
 *
 * This function can be used to check whether a specific element is contained in the sparse
 * vector. It specifically searches for the element with index \a index. In case the element
 * is found, the function returns an iterator to the element. Otherwise an iterator just past
 * the last non-zero element of the sparse vector (the end() iterator) is returned. Note that
 * the returned sparse vector iterator is subject to invalidation due to inserting operations
 * via the subscript operator or the insert() function!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline typename SparseVectorN<Type,TF>::ConstIterator SparseVectorN<Type,TF>::find( size_t index ) const
{
   const Iterator pos( std::lower_bound( begin_, end_, index, FindIndex() ) );
   if( pos != end_ && pos->index_ == index )
      return pos;
   else return end_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the size of the sparse vector.
 *
 * \param n The new size of the sparse vector.
 * \param preserve \a true if the old values of the vector should be preserved, \a false if not.
 * \return void
 *
 * This function resizes the sparse vector using the given size to \a n. During this operation,
 * new dynamic memory may be allocated in case the capacity of the sparse vector is too small.
 * Therefore this function potentially changes all vector elements. In order to preserve the old
 * vector values, the \a preserve flag can be set to \a true.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void SparseVectorN<Type,TF>::resize( size_t n, bool preserve )
{
   if( preserve ) {
      end_ = std::lower_bound( begin_, end_, n, FindIndex() );
   }
   else {
      end_ = begin_;
   }

   size_ = n;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of the sparse vector.
 *
 * \param n The new minimum capacity of the sparse vector.
 * \return void
 *
 * This function increases the capacity of the sparse vector to at least \a n elements. The
 * current values of the vector elements are preserved.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void SparseVectorN<Type,TF>::reserve( size_t n )
{
   if( n > capacity_ ) {
      const size_t newCapacity( n );

      // Allocating a new data and index array
      Iterator newBegin  = new Element[newCapacity];

      // Replacing the old data and index array
      end_ = std::copy( begin_, end_, newBegin );
      std::swap( newBegin, begin_ );
      capacity_ = newCapacity;
      delete [] newBegin;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the sparse vector length \f$|\vec{a}|\f$.
 *
 * \return The length of the sparse vector.
 *
 * This function calculates the actual length of the vector. The return type of the length()
 * function depends on the actual type of the vector instance:
 *
 * <table border="0" cellspacing="0" cellpadding="1">
 *    <tr>
 *       <td width="250px"> \b Type </td>
 *       <td width="100px"> \b LengthType </td>
 *    </tr>
 *    <tr>
 *       <td>float</td>
 *       <td>float</td>
 *    </tr>
 *    <tr>
 *       <td>integral data types and double</td>
 *       <td>double</td>
 *    </tr>
 *    <tr>
 *       <td>long double</td>
 *       <td>long double</td>
 *    </tr>
 * </table>
 *
 * \b Note: This operation is only defined for built-in data types. In case \a Type is a user
 * defined data type the attempt to use the length() function results in a compile time error!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
#ifndef WIN32
inline typename SparseVectorN<Type,TF>::LengthType SparseVectorN<Type,TF>::length() const
#else
inline typename CMathTrait<Type>::Type SparseVectorN<Type,TF>::length() const
#endif
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( Type );

   LengthType sum( 0 );
   for( Iterator element=begin_; element!=end_; ++element )
      sum += element->value_ * element->value_;
   return std::sqrt( sum );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the vector square length \f$|\vec{a}|^2\f$.
 *
 * \return The square length of the vector.
 *
 * This function calculates the actual square length of the vector.
 *
 * \b Note: This operation is only defined for built-in data types. In case \a Type is a user
 * defined data type the attempt to use the length() function results in a compile time error!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Type SparseVectorN<Type,TF>::sqrLength() const
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( Type );

   Type sum( 0 );
   for( Iterator element=begin_; element!=end_; ++element )
      sum += element->value_ * element->value_;
   return sum;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Normalization of the sparse vector (\f$|\vec{a}|=1\f$).
 *
 * \return Reference to the sparse vector.
 *
 * Normalization of the sparse vector to a length of 1. This operation is only defined for
 * floating point vectors. The attempt to use this function for an integral vector results
 * in a compile time error.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::normalize()
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   const Type len( length() );

   if( len == Type(0) )
      return *this;

   const Type ilen( Type(1) / len );

   for( Iterator element=begin_; element!=end_; ++element )
      element->value_ *= ilen;

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the normalized sparse vector (\f$|\vec{a}|=1\f$).
 *
 * \return The normalized sparse vector.
 *
 * The function returns the normalized sparse vector. This operation is only defined for
 * floating point vectors. The attempt to use this function for an integral vector results
 * in a compile time error.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const SparseVectorN<Type,TF> SparseVectorN<Type,TF>::getNormalized() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   const Type len( length() );

   if( len == Type(0) )
      return *this;

   const Type ilen( Type(1) / len );
   SparseVectorN tmp( *this );

   for( Iterator element=tmp.begin_; element!=tmp.end_; ++element ) {
      element->value_ *= ilen;
   }

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the sparse vector by the scalar value \a scalar (\f$ \vec{a}=\vec{b}*s \f$).
 *
 * \param scalar The scalar value for the vector scaling.
 * \return Reference to the sparse vector.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the scalar value
inline SparseVectorN<Type,TF>& SparseVectorN<Type,TF>::scale( Other scalar )
{
   for( Iterator element=begin_; element!=end_; ++element )
      element->value_ *= scalar;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the smallest element of the sparse vector.
 *
 * \return The smallest sparse vector element.
 *
 * In case the vector currently has a size of 0, the returned value is 0.
 *
 * \b Note: In case the sparse vector is not completely filled, the zero elements are also
 * taken into account. Example: the following sparse vector has only 2 non-zero elements.
 * However, the minimum of this vector is 0:

                              \f[
                              \left(\begin{array}{*{4}{c}}
                              1 & 0 & 3 & 0 \\
                              \end{array}\right)
                              \f]
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Type SparseVectorN<Type,TF>::min() const
{
   if( begin_ == end_ ) {
      return Type();
   }
   else if( nonZeros() == size_ ) {
      Type minimum( begin_->value_ );
      for( Iterator element=begin_+1; element!=end_; ++element )
         minimum = pe::min( minimum, element->value_ );
      return minimum;
   }
   else {
      Type minimum( 0 );
      for( Iterator element=begin_; element!=end_; ++element )
         minimum = pe::min( minimum, element->value_ );
      return minimum;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the largest element of the sparse vector.
 *
 * \return The largest sparse vector element.
 *
 * In case the vector currently has a size of 0, the returned value is 0.
 *
 * \b Note: In case the sparse vector is not completely filled, the zero elements are also
 * taken into account. Example: the following sparse vector has only 2 non-zero elements.
 * However, the maximum of this vector is 0:

                              \f[
                              \left(\begin{array}{*{4}{c}}
                              -1 & 0 & -3 & 0 \\
                              \end{array}\right)
                              \f]
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Type SparseVectorN<Type,TF>::max() const
{
   using pe::max;

   if( begin_ == end_ ) {
      return Type();
   }
   else if( nonZeros() == size_ ) {
      Type maximum( begin_->value_ );
      for( Iterator element=begin_+1; element!=end_; ++element )
         maximum = max( maximum, element->value_ );
      return maximum;
   }
   else {
      Type maximum( 0 );
      for( Iterator element=begin_; element!=end_; ++element )
         maximum = max( maximum, element->value_ );
      return maximum;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two sparse vectors.
 *
 * \param sv The sparse vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void SparseVectorN<Type,TF>::swap( SparseVectorN& sv ) /* throw() */
{
   std::swap( size_, sv.size_ );
   std::swap( capacity_, sv.capacity_ );
   std::swap( begin_, sv.begin_ );
   std::swap( end_, sv.end_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating a new vector capacity.
 *
 * \return The new sparse vector capacity.
 *
 * This function calculates a new vector capacity based on the current capacity of the sparse
 * vector. Note that the new capacity is restricted to the interval \f$[7..size]\f$.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t SparseVectorN<Type,TF>::extendCapacity() const
{
   using pe::max;
   using pe::min;

   size_t nonzeros( 2*capacity_+1 );
   nonzeros = max( nonzeros, size_t(7) );
   nonzeros = min( nonzeros, size_ );

   pe_INTERNAL_ASSERT( nonzeros > capacity_, "Invalid capacity value" );

   return nonzeros;
}
//*************************************************************************************************




//=================================================================================================
//
//  EXPRESSION TEMPLATE EVALUATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the vector is aliased with the given address \a alias.
 *
 * \param alias The alias to be checked.
 * \return \a true in case the alias corresponds to this vector, \a false if not.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the foreign expression
inline bool SparseVectorN<Type,TF>::isAliased( const Other* alias ) const
{
   return static_cast<const void*>( this ) == static_cast<const void*>( alias );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a dense vector.
 *
 * \param rhs The right-hand side dense vector to be assigned.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline void SparseVectorN<Type,TF>::assign( const DenseVector<VT,TF>& rhs )
{
   size_t nonzeros( 0 );

   for( size_t i=0; i<size_; ++i ) {
      if( !isDefault( (~rhs)[i] ) ) {
         if( nonzeros++ == capacity_ )
            reserve( extendCapacity() );
         append( i, (~rhs)[i] );
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a sparse vector.
 *
 * \param rhs The right-hand side sparse vector to be assigned.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline void SparseVectorN<Type,TF>::assign( const SparseVector<VT,TF>& rhs )
{
   // Using the following formulation instead of a std::copy function call of the form
   //
   //          end_ = std::copy( (~rhs).begin(), (~rhs).end(), begin_ );
   //
   // results in much less requirements on the ConstIterator type provided from the right-hand
   // sparse vector type
   for( typename VT::ConstIterator element=(~rhs).begin(); element!=(~rhs).end(); ++element )
      append( element->index(), element->value() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a dense vector.
 *
 * \param rhs The right-hand side dense vector to be added.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline void SparseVectorN<Type,TF>::addAssign( const DenseVector<VT,TF>& rhs )
{
   SparseVectorN<Type,TF> tmp( *this + rhs );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a sparse vector.
 *
 * \param rhs The right-hand side sparse vector to be added.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline void SparseVectorN<Type,TF>::addAssign( const SparseVector<VT,TF>& rhs )
{
   SparseVectorN<Type,TF> tmp( *this + rhs );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a dense vector.
 *
 * \param rhs The right-hand side dense vector to be subtracted.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline void SparseVectorN<Type,TF>::subAssign( const DenseVector<VT,TF>& rhs )
{
   SparseVectorN<Type,TF> tmp( *this - rhs );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a sparse vector.
 *
 * \param rhs The right-hand side sparse vector to be subtracted.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline void SparseVectorN<Type,TF>::subAssign( const SparseVector<VT,TF>& rhs )
{
   SparseVectorN<Type,TF> tmp( *this - rhs );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a dense vector.
 *
 * \param rhs The right-hand side dense vector to be multiplied.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline void SparseVectorN<Type,TF>::multAssign( const DenseVector<VT,TF>& rhs )
{
   SparseVectorN<Type,TF> tmp( *this * rhs );
   swap( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a sparse vector.
 *
 * \param rhs The right-hand side sparse vector to be multiplied.
 * \return void
 *
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline void SparseVectorN<Type,TF>::multAssign( const SparseVector<VT,TF>& rhs )
{
   SparseVectorN<Type,TF> tmp( *this * rhs );
   swap( tmp );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name SparseVectorN operators */
//@{
template< typename Type, bool TF >
inline bool isnan( const SparseVectorN<Type,TF>& v );

template< typename Type, bool TF >
inline void reset( SparseVectorN<Type,TF>& v );

template< typename Type, bool TF >
inline void clear( SparseVectorN<Type,TF>& v );

template< typename Type, bool TF >
inline bool isDefault( const SparseVectorN<Type,TF>& v );

template< typename Type, bool TF >
inline const typename MathTrait<Type,Type>::MultType sq( const SparseVectorN<Type,TF>& v );

template< typename Type, bool TF >
inline void swap( SparseVectorN<Type,TF>& a, SparseVectorN<Type,TF>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks the given sparse vector for not-a-number elements.
 * \ingroup sparse_vector_N
 *
 * \param v The sparse vector to be checked for not-a-number elements.
 * \return \a true if at least one element of the vector is not-a-number, \a false otherwise.
 *
 * This function checks the N-dimensional sparse vector for not-a-number (NaN) elements. If at
 * least one element of the vector is not-a-number, the function returns \a true, otherwise it
 * returns \a false.

   \code
   pe::SVecN a;
   // ... Resizing and initialization
   if( isnan( a ) ) { ... }
   \endcode
 */
template< typename Type  // Data type of the sparse vector
        , bool TF >      // Transposition flag
inline bool isnan( const SparseVectorN<Type,TF>& v )
{
   typedef typename SparseVectorN<Type,TF>::ConstIterator  ConstIterator;

   const ConstIterator end( v.end() );
   for( ConstIterator element=v.begin(); element!=end; ++element ) {
      if( isnan( element->value() ) ) return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given sparse vector.
 * \ingroup sparse_vector_N
 *
 * \param v The sparse vector to be resetted.
 * \return void
 */
template< typename Type  // Data type of the sparse vector
        , bool TF >      // Transposition flag
inline void reset( SparseVectorN<Type,TF>& v )
{
   v.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given sparse vector.
 * \ingroup sparse_vector_N
 *
 * \param v The sparse vector to be cleared.
 * \return void
 */
template< typename Type  // Data type of the sparse vector
        , bool TF >      // Transposition flag
inline void clear( SparseVectorN<Type,TF>& v )
{
   v.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given sparse vector is in default state.
 * \ingroup sparse_vector_N
 *
 * \param v The sparse vector to be tested for its default state.
 * \return \a true in case the given vector is component-wise zero, \a false otherwise.
 *
 * This function checks whether the N-dimensional sparse vector is in default state. For instance,
 * in case the vector is instantiated for a built-in integral or floating point data type, the
 * function returns \a true in case all vector elements are 0 and \a false in case any vector
 * element is not 0. Following example demonstrates the use of the \a isDefault function:

   \code
   pe::SVecN a;
   // ... Resizing and initialization
   if( isDefault( a ) ) { ... }
   \endcode
 */
template< typename Type  // Data type of the sparse vector
        , bool TF >      // Transposition flag
inline bool isDefault( const SparseVectorN<Type,TF>& v )
{
   typedef typename SparseVectorN<Type,TF>::ConstIterator  ConstIterator;

   for( ConstIterator element=v.begin(); element!=v.end(); ++element )
      if( !isDefault( element->value() ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Squaring the given sparse vector.
 * \ingroup sparse_vector_N
 *
 * \param v The sparse vector to be squared.
 * \return The result of the square operation.
 *
 * This function calculates the component product of the given sparse vector. It has the same
 * effect as multiplying the vector with itself (\f$ v * v \f$).
 */
template< typename Type  // Data type of the sparse vector
        , bool TF >      // Transposition flag
inline const SparseVectorN< typename MathTrait<Type,Type>::MultType, TF > sq( const SparseVectorN<Type>& v )
{
   return v * v;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two sparse vectors.
 * \ingroup sparse_vector_N
 *
 * \param a The first sparse vector to be swapped.
 * \param b The second sparse vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type  // Data type of the sparse vector
        , bool TF >      // Transposition flag
inline void swap( SparseVectorN<Type,TF>& a, SparseVectorN<Type,TF>& b ) /* throw() */
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
template< typename T1, typename T2, bool TF >
struct MathTrait< SparseVectorN<T1,TF>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                                    HighType;
   typedef INVALID_NUMERICAL_TYPE                                    LowType;
   typedef INVALID_NUMERICAL_TYPE                                    AddType;
   typedef INVALID_NUMERICAL_TYPE                                    SubType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::DivType , TF >  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T2 );
};

template< typename T1, typename T2, bool TF >
struct MathTrait< T1, SparseVectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                                    HighType;
   typedef INVALID_NUMERICAL_TYPE                                    LowType;
   typedef INVALID_NUMERICAL_TYPE                                    AddType;
   typedef INVALID_NUMERICAL_TYPE                                    SubType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                    DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T1 );
};

template< typename T1, typename T2, bool TF >
struct MathTrait< SparseVectorN<T1,TF>, Vector2<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector2< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector2< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector2< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, Vector2<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,false>, Vector2<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< Vector2<T1,TF>, SparseVectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector2< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector2< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector2< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,true>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,false>, SparseVectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< SparseVectorN<T1,TF>, Vector3<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector3< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector3< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, Vector3<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,false>, Vector3<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< Vector3<T1,TF>, SparseVectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector3< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector3< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector3<T1,true>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector3<T1,false>, SparseVectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< SparseVectorN<T1,TF>, Vector6<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector6< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector6< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, Vector6<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,false>, Vector6<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< Vector6<T1,TF>, SparseVectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector6< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector6< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector6<T1,true>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector6<T1,false>, SparseVectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< SparseVectorN<T1,TF>, VectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                                    HighType;
   typedef INVALID_NUMERICAL_TYPE                                    LowType;
   typedef VectorN< typename MathTrait<T1,T2>::AddType , TF >        AddType;
   typedef VectorN< typename MathTrait<T1,T2>::SubType , TF >        SubType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                    DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,false>, VectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< VectorN<T1,TF>, SparseVectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                                    HighType;
   typedef INVALID_NUMERICAL_TYPE                                    LowType;
   typedef VectorN< typename MathTrait<T1,T2>::AddType , TF >        AddType;
   typedef VectorN< typename MathTrait<T1,T2>::SubType , TF >        SubType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                    DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,false>, SparseVectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< SparseVectorN<T1,TF>, SparseVectorN<T2,TF> >
{
   typedef SparseVectorN< typename MathTrait<T1,T2>::HighType, TF >  HighType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::LowType , TF >  LowType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef SparseVectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                    DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,true>, SparseVectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< SparseVectorN<T1,false>, SparseVectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                                  HighType;
   typedef INVALID_NUMERICAL_TYPE                                  LowType;
   typedef INVALID_NUMERICAL_TYPE                                  AddType;
   typedef INVALID_NUMERICAL_TYPE                                  SubType;
   typedef SparseMatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                                  DivType;
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  TRANSPOSETRAIT SPECIALIZATION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
template< typename T, bool TF >
struct TransposeTrait< SparseVectorN<T,TF> >
{
   typedef SparseVectorN<typename TransposeTrait<T>::Type,!TF>  Type;
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sparse real vector.
 * \ingroup sparse_vector_N
 */
typedef SparseVectorN<real>  SVecN;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transpose sparse real vector.
 * \ingroup sparse_vector_N
 */
typedef SparseVectorN<real,true>  SVecNT;
//*************************************************************************************************

} // namespace pe

#endif
