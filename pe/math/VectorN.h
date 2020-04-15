//=================================================================================================
/*!
 *  \file pe/math/VectorN.h
 *  \brief Implementation of an arbitrary sized vector
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

#ifndef _PE_MATH_VECTORN_H_
#define _PE_MATH_VECTORN_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <fstream>
#include <ostream>
#include <stdexcept>
#include <pe/math/CMathTrait.h>
#include <pe/math/DenseVector.h>
#include <pe/math/MathTrait.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/TransposeTrait.h>
#include <pe/system/Precision.h>
#include <pe/system/Restrict.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>
#include <pe/util/constraints/Const.h>
#include <pe/util/constraints/FloatingPoint.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/constraints/Volatile.h>
#include <pe/util/EnableIf.h>
#include <pe/util/Null.h>
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

template< typename >       class  MatrixMxN;
template< typename, bool > struct SparseVector;
template< typename, bool > class  Vector2;
template< typename, bool > class  Vector3;
template< typename, bool > class  Vector6;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup dense_vector_N VectorN
 * \ingroup dense_vector
 */
/*!\brief Efficient implementation of an arbitrary sized vector.
 * \ingroup dense_vector_N
 *
 * The VectorN class is the representation of a vector with an arbitrary number N of dynamically
 * allocated elements. The elements can be accessed directly with the subscript operator. The
 * order of the elements is as following:

                             \f[\left(\begin{array}{*{5}{c}}
                             0 & 1 & 2 & \cdots & N-1 \\
                             \end{array}\right)\f]

 * VectorN can be used with any non-cv-qualified element type. The arithmetic operators for
 * vector/vector and vector/element operations with the same element type work for any element
 * type as long as the element type supports the arithmetic operation. Arithmetic operations
 * between vectors and elements of different element types are only supported for all data types
 * supported by the MathTrait class template (for details see the MathTrait class description).

   \code
   VectorN< double > a, b, c;
   VectorN< float  > d;
   VectorN< std::complex<double> > e, f, g;
   VectorN< std::complex<float>  > h;

   ...         // Appropriate resizing

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type      // Data type of the vector
        , bool TF = false >  // Transposition flag
class VectorN : public DenseVector< VectorN<Type,TF>, TF >
{
public:
   //**Type definitions****************************************************************************
   typedef VectorN<Type,TF>  This;           //!< Type of this VectorN instance.
   typedef This              ResultType;     //!< Result type for expression template evaluations.
   typedef Type              ElementType;    //!< Type of the vector elements.
   typedef const VectorN&    CompositeType;  //!< Data type for composite expression templates.

   //! Transpose type for expression template evaluations.
   typedef VectorN< typename TransposeTrait<Type>::Type, !TF >  TransposeType;

   //! Vector length return type.
   /*! Return type of the VectorN<Type>::length function. */
   typedef typename CMathTrait<Type>::Type  LengthType;
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                           explicit inline VectorN();
                           explicit inline VectorN( size_t n );
                           explicit inline VectorN( size_t n, Type init );
                                    inline VectorN( const VectorN& v );
   template< typename VT >          inline VectorN( const DenseVector<VT,TF>&  dv );
   template< typename VT >          inline VectorN( const SparseVector<VT,TF>& sv );

   template< typename Other, size_t N >
   explicit inline VectorN( const Other (&rhs)[N] );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~VectorN();
   //@}
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   template< typename Other, size_t N >
   inline VectorN& operator=( const Other (&rhs)[N] );

                           inline VectorN&    operator= ( Type rhs );
                           inline VectorN&    operator= ( const VectorN& rhs );
   template< typename VT > inline VectorN&    operator= ( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline VectorN&    operator= ( const SparseVector<VT,TF>& rhs );
                           inline Type&       operator[]( size_t index );
                           inline const Type& operator[]( size_t index ) const;
   template< typename VT > inline VectorN&    operator+=( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline VectorN&    operator+=( const SparseVector<VT,TF>& rhs );
   template< typename VT > inline VectorN&    operator-=( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline VectorN&    operator-=( const SparseVector<VT,TF>& rhs );
   template< typename VT > inline VectorN&    operator*=( const DenseVector<VT,TF>&  rhs );
   template< typename VT > inline VectorN&    operator*=( const SparseVector<VT,TF>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, VectorN >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, VectorN >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t        size()                              const;
                              inline size_t        capacity()                          const;
                              inline size_t        nonZeros()                          const;
                              inline void          reset();
                              inline void          clear();
                              inline void          resize( size_t n, bool preserve=true );
                              inline void          extend( size_t n, bool preserve=true );
                              inline void          reserve( size_t n );
                              inline LengthType    length()                            const;
                              inline const Type    sqrLength()                         const;
                              inline VectorN&      normalize();
                              inline const VectorN getNormalized()                     const;
   template< typename Other > inline VectorN&      scale( Other scalar );
                              inline Type          min()                               const;
                              inline Type          max()                               const;
                              inline void          swap( VectorN& v ) /* throw() */;
   //@}
   //**********************************************************************************************

   //**Read/Write functions************************************************************************
   /*!\name Read/Write functions */
   //@{
   void read ( const char* file );
   void write( const char* file, std::streamsize prec=6 ) const;
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
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t size_;           //!< The current size/dimension of the vector.
   size_t capacity_;       //!< The maximum capacity of the vector.
   Type* pe_RESTRICT v_;   //!< The dynamically allocated vector elements.
                           /*!< Access to the vector elements is gained via the subscript operator.
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
/*!\brief The default constructor for VectorN.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>::VectorN()
   : size_    ( 0 )     // The current size/dimension of the vector
   , capacity_( 0 )     // The maximum capacity of the vector
   , v_       ( NULL )  // The vector elements
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a vector of size \a n. No element initialization is performed!
 *
 * \param n The size of the vector.
 *
 * \b Note: This constructor is only responsible to allocate the required dynamic memory. No
 *          element initialization is performed!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>::VectorN( size_t n )
   : size_    ( n )            // The current size/dimension of the vector
   , capacity_( n )            // The maximum capacity of the vector
   , v_       ( new Type[n] )  // The vector elements
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a homogeneous initialization of all \a n vector elements.
 *
 * \param n The size of the vector.
 * \param init The initial value of the vector elements.
 *
 * All vector elements are initialized with the specified value.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>::VectorN( size_t n, Type init )
   : size_    ( n )            // The current size/dimension of the vector
   , capacity_( n )            // The maximum capacity of the vector
   , v_       ( new Type[n] )  // The vector elements
{
   for( size_t i=0; i<size_; ++i )
      v_[i] = init;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for VectorN.
 *
 * \param v Vector to be copied.
 *
 * The copy constructor is explicitly defined due to the required dynamic memory management
 * and in order to enable/facilitate NRV optimization.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>::VectorN( const VectorN& v )
   : size_    ( v.size_ )          // The current size/dimension of the vector
   , capacity_( v.size_ )          // The maximum capacity of the vector
   , v_       ( new Type[size_] )  // The vector elements
{
   for( size_t i=0; i<size_; ++i )
      v_[i] = v.v_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different dense vectors.
 *
 * \param dv Dense vector to be copied.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the foreign dense vector
inline VectorN<Type,TF>::VectorN( const DenseVector<VT,TF>& dv )
   : size_    ( (~dv).size() )     // The current size/dimension of the vector
   , capacity_( size_ )            // The maximum capacity of the vector
   , v_       ( new Type[size_] )  // The vector elements
{
   using pe::assign;
   assign( *this, ~dv );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from sparse vectors.
 *
 * \param sv Sparse vector to be copied.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the foreign sparse vector
inline VectorN<Type,TF>::VectorN( const SparseVector<VT,TF>& sv )
   : size_    ( (~sv).size() )     // The current size/dimension of the vector
   , capacity_( size_ )            // The maximum capacity of the vector
   , v_       ( new Type[size_] )  // The vector elements
{
   using pe::assign;

   if( IsBuiltin<Type>::value )
      reset();

   assign( *this, ~sv );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Array initialization of all vector elements.
 *
 * \param rhs N-dimensional array for the initialization.
 *
 * This assignment operator offers the option to directly initialize the elements of the vector:

   \code
   const real init[4] = { 1, 2, 3 };
   VectorN<real> v( init );
   \endcode

 * The vector is sized accoring to the size of the array and initialized with the given values.
 * Missing values are initialized with zero (as e.g. the fourth element in the example).
 */
template< typename Type   // Data type of the vector
        , bool TF >       // Transposition flag
template< typename Other  // Data type of the initialization array
        , size_t N >      // Dimension of the initialization array
inline VectorN<Type,TF>::VectorN( const Other (&rhs)[N] )
   : size_    ( N )            // The current size/dimension of the vector
   , capacity_( N )            // The maximum capacity of the vector
   , v_       ( new Type[N] )  // The vector elements
{
   for( size_t i=0; i<N; ++i )
      v_[i] = rhs[i];
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for VectorN.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>::~VectorN()
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
/*!\brief Array assignment to all vector elements.
 *
 * \param rhs N-dimensional array for the assignment.
 * \return Reference to the assigned vector.
 *
 * This assignment operator offers the option to directly set all elements of the vector:

   \code
   const real init[4] = { 1, 2, 3 };
   VectorN<real> v;
   v = init;
   \endcode

 * The vector is resized accoring to the size of the array and initialized with the given values.
 * Missing values are initialized with zero (as e.g. the fourth element in the example).
 */
template< typename Type   // Data type of the vector
        , bool TF >       // Transposition flag
template< typename Other  // Data type of the initialization array
        , size_t N >      // Dimension of the initialization array
inline VectorN<Type,TF>& VectorN<Type,TF>::operator=( const Other (&rhs)[N] )
{
   resize( N, false );

   for( size_t i=0; i<N; ++i )
      v_[i] = rhs[i];

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Homogenous assignment to all vector elements.
 *
 * \param rhs Scalar value to be assigned to all vector elements.
 * \return Reference to the assigned vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>& VectorN<Type,TF>::operator=( Type rhs )
{
   for( size_t i=0; i<size_; ++i )
      v_[i] = rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for VectorN.
 *
 * \param rhs Vector to be copied.
 * \return Reference to the assigned vector.
 *
 * The vector is resized according to the given N-dimensional vector and initialized as a
 * copy of this vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>& VectorN<Type,TF>::operator=( const VectorN& rhs )
{
   if( &rhs == this ) return *this;

   resize( rhs.size_, false );

   for( size_t i=0; i<size_; ++i )
      v_[i] = rhs.v_[i];

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different dense vectors.
 *
 * \param rhs Dense vector to be copied.
 * \return Reference to the assigned vector.
 *
 * The vector is resized according to the given N-dimensional vector and initialized as a
 * copy of this vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator=( const DenseVector<VT,TF>& rhs )
{
   using pe::assign;

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      VectorN tmp( rhs );
      swap( tmp );
   }
   else {
      resize( (~rhs).size(), false );
      assign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for sparse vectors.
 *
 * \param rhs Sparse vector to be copied.
 * \return Reference to the assigned vector.
 *
 * The vector is resized according to the given N-dimensional vector and initialized as a
 * copy of this vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator=( const SparseVector<VT,TF>& rhs )
{
   using pe::assign;

   resize( (~rhs).size(), false );
   reset();
   assign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Type& VectorN<Type,TF>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < size_, "Invalid vector access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..N-1]\f$.
 * \return Reference to the accessed value.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Type& VectorN<Type,TF>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < size_, "Invalid vector access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a dense vector (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side dense vector to be added to the vector.
 * \return Reference to the vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator+=( const DenseVector<VT,TF>& rhs )
{
   using pe::addAssign;

   if( (~rhs).size() != size_ )
      throw std::invalid_argument( "Vector sizes do not match" );

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      VectorN tmp( rhs );
      for( size_t i=0; i<size_; ++i )
         v_[i] += tmp[i];
   }
   else {
      addAssign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition assignment operator for the addition of a sparse vector (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side sparse vector to be added to the vector.
 * \return Reference to the vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator+=( const SparseVector<VT,TF>& rhs )
{
   using pe::addAssign;

   if( (~rhs).size() != size_ )
      throw std::invalid_argument( "Vector sizes do not match" );

   addAssign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a dense vector
 *        (\f$ \vec{a}-=\vec{b} \f$).
 *
 * \param rhs The right-hand side dense vector to be subtracted from the vector.
 * \return Reference to the vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator-=( const DenseVector<VT,TF>& rhs )
{
   using pe::subAssign;

   if( (~rhs).size() != size_ )
      throw std::invalid_argument( "Vector sizes do not match" );

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      VectorN tmp( rhs );
      for( size_t i=0; i<size_; ++i )
         v_[i] -= tmp[i];
   }
   else {
      subAssign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction assignment operator for the subtraction of a sparse vector
 *        (\f$ \vec{a}-=\vec{b} \f$).
 *
 * \param rhs The right-hand side sparse vector to be subtracted from the vector.
 * \return Reference to the vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Data type of the right-hand side sparse vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator-=( const SparseVector<VT,TF>& rhs )
{
   using pe::subAssign;

   if( (~rhs).size() != size_ )
      throw std::invalid_argument( "Vector sizes do not match" );

   subAssign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a dense vector
 *        (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side dense vector to be multiplied with the vector.
 * \return Reference to the vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator*=( const DenseVector<VT,TF>& rhs )
{
   using pe::multAssign;

   if( (~rhs).size() != size_ )
      throw std::invalid_argument( "Vector sizes do not match" );

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      VectorN tmp( rhs );
      for( size_t i=0; i<size_; ++i )
         v_[i] *= tmp[i];
   }
   else {
      multAssign( *this, ~rhs );
   }

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication of a sparse vector
 *        (\f$ \vec{a}+=\vec{b} \f$).
 *
 * \param rhs The right-hand side sparse vector to be multiplied with the vector.
 * \return Reference to the vector.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * In case the current sizes of the two vectors don't match, a \a std::invalid_argument exception
 * is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline VectorN<Type,TF>& VectorN<Type,TF>::operator*=( const SparseVector<VT,TF>& rhs )
{
   using pe::multAssign;

   if( (~rhs).size() != size_ )
      throw std::invalid_argument( "Vector sizes do not match" );

   multAssign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication assignment operator for the multiplication between a vector and
 *        a scalar value (\f$ \vec{a}*=s \f$).
 *
 * \param rhs The right-hand side scalar value for the multiplication.
 * \return Reference to the vector.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, VectorN<Type,TF> >::Type&
   VectorN<Type,TF>::operator*=( Other rhs )
{
   for( size_t i=0; i<size_; ++i )
      v_[i] *= rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division assignment operator for the division of a vector by a scalar value
 *        (\f$ \vec{a}/=s \f$).
 *
 * \param rhs The right-hand side scalar value for the division.
 * \return Reference to the vector.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the right-hand side scalar
inline typename EnableIf< IsNumeric<Other>, VectorN<Type,TF> >::Type&
   VectorN<Type,TF>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      for( size_t i=0; i<size_; ++i )
         v_[i] = static_cast<Type>( static_cast<DT>( v_[i] ) * tmp );
   }
   else {
      for( size_t i=0; i<size_; ++i )
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
/*!\brief Returns the current size/dimension of the vector.
 *
 * \return The size of the vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t VectorN<Type,TF>::size() const
{
   return size_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum capacity of the vector.
 *
 * \return The capacity of the vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t VectorN<Type,TF>::capacity() const
{
   return capacity_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of non-zero elements in the vector.
 *
 * \return The number of non-zero elements in the vector.
 *
 * Note that the number of non-zero elements is always smaller than the current size of
 * the vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t VectorN<Type,TF>::nonZeros() const
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
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void VectorN<Type,TF>::reset()
{
   using pe::reset;
   for( size_t i=0; i<size_; ++i )
      reset( v_[i] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the vector.
 *
 * \return void
 *
 * After the clear() function, the size of the vector is 0.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void VectorN<Type,TF>::clear()
{
   size_ = 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the size of the vector.
 *
 * \param n The new size of the vector.
 * \param preserve \a true if the old values of the vector should be preserved, \a false if not.
 * \return void
 *
 * This function resizes the vector using the given size to \a n. During this operation, new
 * dynamic memory may be allocated in case the capacity of the vector is too small. Therefore
 * this function potentially changes all vector elements. In order to preserve the old vector
 * values, the \a preserve flag can be set to \a true. However, new vector elements are not
 * initialized!\n
 * The following example illustrates the resize operation of a vector of size 2 to a vector of
 * size 4. The new, uninitialized elements are marked with \a x:

                              \f[
                              \left(\begin{array}{*{2}{c}}
                              1 & 2 \\
                              \end{array}\right)

                              \Longrightarrow

                              \left(\begin{array}{*{4}{c}}
                              1 & 2 & x & x \\
                              \end{array}\right)
                              \f]
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void VectorN<Type,TF>::resize( size_t n, bool preserve )
{
   using pe::min;

   if( n == size_ ) return;

   if( preserve )
   {
      Type* pe_RESTRICT v = new Type[n];
      const size_t minsize( min( n, size_ ) );

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
/*!\brief Extending the size of the vector.
 *
 * \param n Number of additional vector elements.
 * \param preserve \a true if the old values of the vector should be preserved, \a false if not.
 * \return void
 *
 * This function increases the vector size by \a n elements. During this operation, new dynamic
 * memory may be allocated in case the capacity of the vector is too small. Therefore this
 * function potentially changes all vector elements. In order to preserve the old vector values,
 * the \a preserve flag can be set to \a true. However, new vector elements are not initialized!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void VectorN<Type,TF>::extend( size_t n, bool preserve )
{
   resize( size_+n, preserve );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the minimum capacity of the vector.
 *
 * \param n The new minimum capacity of the vector.
 * \return void
 *
 * This function increases the capacity of the vector to at least \a n elements. The current
 * values of the vector elements are preserved.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void VectorN<Type,TF>::reserve( size_t n )
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
/*!\brief Calculation of the vector length \f$|\vec{a}|\f$.
 *
 * \return The length of the vector.
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
inline typename VectorN<Type,TF>::LengthType VectorN<Type,TF>::length() const
#else
inline typename CMathTrait<Type>::Type VectorN<Type,TF>::length() const
#endif
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( Type );

   LengthType sum( 0 );
   for( size_t i=0; i<size_; ++i )
      sum += v_[i] * v_[i];
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
inline const Type VectorN<Type,TF>::sqrLength() const
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( Type );

   Type sum( 0 );
   for( size_t i=0; i<size_; ++i )
      sum += v_[i] * v_[i];
   return sum;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Normalization of the vector (\f$|\vec{a}|=1\f$).
 *
 * \return Reference to the vector.
 *
 * Normalization of the vector to a length of 1. This operation is only defined for floating
 * point vectors. The attempt to use this function for an integral vector results in a compile
 * time error.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline VectorN<Type,TF>& VectorN<Type,TF>::normalize()
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   const Type len( length() );

   if( len == Type(0) )
      return *this;

   const Type ilen( Type(1) / len );

   for( size_t i=0; i<size_; ++i )
      v_[i] *= ilen;

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the normalized vector (\f$|\vec{a}|=1\f$).
 *
 * \return The normalized vector.
 *
 * The function returns the normalized vector. This operation is only defined for floating
 * point vectors. The attempt to use this function for an integral vector results in a compile
 * time error.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const VectorN<Type,TF> VectorN<Type,TF>::getNormalized() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   const Type len( length() );

   if( len == Type(0) )
      return *this;

   const Type ilen( Type(1) / len );
   VectorN tmp( size_ );

   for( size_t i=0; i<size_; ++i )
      tmp[i] = v_[i] * ilen;

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the vector by the scalar value \a scalar (\f$ \vec{a}=\vec{b}*s \f$).
 *
 * \param scalar The scalar value for the vector scaling.
 * \return Reference to the vector.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the scalar value
inline VectorN<Type,TF>& VectorN<Type,TF>::scale( Other scalar )
{
   for( size_t i=0; i<size_; ++i )
      v_[i] *= scalar;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the smallest element of the vector.
 *
 * \return The smallest vector element.
 *
 * In case the vector currently has a size of 0, the returned value is 0.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Type VectorN<Type,TF>::min() const
{
   using pe::min;

   if( size_ == 0 ) return Type(0);

   Type minimum( v_[0] );
   for( size_t i=1; i<size_; ++i )
      minimum = min( minimum, v_[i] );
   return minimum;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the largest element of the vector.
 *
 * \return The largest vector element.
 *
 * In case the vector currently has a size of 0, the returned value is 0.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Type VectorN<Type,TF>::max() const
{
   using pe::max;

   if( size_ == 0 ) return Type(0);

   Type maximum( v_[0] );
   for( size_t i=1; i<size_; ++i )
      maximum = max( maximum, v_[i] );
   return maximum;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two vectors.
 *
 * \param v The vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void VectorN<Type,TF>::swap( VectorN& v ) /* throw() */
{
   std::swap( size_, v.size_ );
   std::swap( capacity_, v.capacity_ );
   std::swap( v_, v.v_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  READ/WRITE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Reading the vector from a file.
 *
 * \param file The name of the input file.
 * \return void
 * \exception std::runtime_error Input error.
 *
 * This function reads a vector from the specified file \a file. The file has to contain the
 * vector data in the following format:

   \code
   #size
   v[0]
   v[1]
   v[2]
   ...
   \endcode

 * where \f$ v[i], i \in [0..N-1], \f$ specifies the vector elements. In case the output file
 * could not be opened or an input error occured, a \a std::runtime_error is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
void VectorN<Type,TF>::read( const char* file )
{
   std::ifstream in( file, std::ifstream::in );
   if( !in.is_open() ) {
      throw std::runtime_error( "File could not be opened!" );
   }

   size_t vsize(0);
   if( !(in >> vsize) || vsize == 0 ) {
      throw std::runtime_error( "Vector size could not be extracted!" );
   }

   VectorN tmp( vsize );

   for( size_t i=0; i<vsize; ++i ) {
      if( !(in >> tmp.v_[i]) ) {
         throw std::runtime_error( "Error during vector extraction!" );
      }
   }

   swap( tmp );

   in.close();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writing the vector to a file.
 *
 * \param file The name of the output file.
 * \param prec The number of non-zero digits displayed in the output file.
 * \return void
 * \exception std::runtime_error Output error.
 *
 * This function writes the vector to the specified file \a file using the following format:

   \code
   #size
   v[0]
   v[1]
   v[2]
   ...
   \endcode

 * where \f$ v[i], i \in [0..N-1], \f$ specifies the vector elements. In case the output file
 * could not be opened, a \a std::runtime_error is thrown.
 *
 * \b Note: All previous data is replaced by the vector data!
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
void VectorN<Type,TF>::write( const char* file, std::streamsize prec ) const
{
   std::ofstream out( file, std::ofstream::out | std::ostream::trunc );
   if( !out.is_open() ) {
      throw std::runtime_error( "File could not be opened!" );
   }

   out << size_ << "\n";

   out.precision( prec );
   for( size_t i=0; i<size_; ++i )
      out << v_[i] << "\n";

   out.close();
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
inline bool VectorN<Type,TF>::isAliased( const Other* alias ) const
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
inline void VectorN<Type,TF>::assign( const DenseVector<VT,TF>& rhs )
{
   const size_t end( (~rhs).size() & size_t(-2) );
   for( size_t i=0; i<end; i+=2 ) {
      v_[i  ] = (~rhs)[i  ];
      v_[i+1] = (~rhs)[i+1];
   }
   if( end < (~rhs).size() )
      v_[end] = (~rhs)[end];
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
inline void VectorN<Type,TF>::assign( const SparseVector<VT,TF>& rhs )
{
   typedef typename VT::ConstIterator  ConstIterator;

   for( ConstIterator element=(~rhs).begin(); element!=(~rhs).end(); ++element )
      v_[element->index()] = element->value();
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
inline void VectorN<Type,TF>::addAssign( const DenseVector<VT,TF>& rhs )
{
   const size_t end( (~rhs).size() & size_t(-2) );
   for( size_t i=0; i<end; i+=2 ) {
      v_[i  ] += (~rhs)[i  ];
      v_[i+1] += (~rhs)[i+1];
   }
   if( end < (~rhs).size() )
      v_[end] += (~rhs)[end];
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
inline void VectorN<Type,TF>::addAssign( const SparseVector<VT,TF>& rhs )
{
   typedef typename VT::ConstIterator  ConstIterator;

   for( ConstIterator element=(~rhs).begin(); element!=(~rhs).end(); ++element )
      v_[element->index()] += element->value();
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
inline void VectorN<Type,TF>::subAssign( const DenseVector<VT,TF>& rhs )
{
   const size_t end( (~rhs).size() & size_t(-2) );
   for( size_t i=0; i<end; i+=2 ) {
      v_[i  ] -= (~rhs)[i  ];
      v_[i+1] -= (~rhs)[i+1];
   }
   if( end < (~rhs).size() )
      v_[end] -= (~rhs)[end];
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
inline void VectorN<Type,TF>::subAssign( const SparseVector<VT,TF>& rhs )
{
   typedef typename VT::ConstIterator  ConstIterator;

   for( ConstIterator element=(~rhs).begin(); element!=(~rhs).end(); ++element )
      v_[element->index()] -= element->value();
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
inline void VectorN<Type,TF>::multAssign( const DenseVector<VT,TF>& rhs )
{
   const size_t end( (~rhs).size() & size_t(-2) );
   for( size_t i=0; i<end; i+=2 ) {
      v_[i  ] *= (~rhs)[i  ];
      v_[i+1] *= (~rhs)[i+1];
   }
   if( end < (~rhs).size() )
      v_[end] *= (~rhs)[end];
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
inline void VectorN<Type,TF>::multAssign( const SparseVector<VT,TF>& rhs )
{
   typedef typename VT::ConstIterator  ConstIterator;

   const VectorN tmp( *this );
   const ConstIterator end( (~rhs).end() );

   reset();

   for( ConstIterator element=(~rhs).begin(); element!=end; ++element )
      v_[element->index()] = tmp[element->index()] * element->value();
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name VectorN operators */
//@{
template< typename Type, bool TF >
inline bool isnan( const VectorN<Type,TF>& v );

template< typename Type, bool TF >
inline void reset( VectorN<Type,TF>& v );

template< typename Type, bool TF >
inline void clear( VectorN<Type,TF>& v );

template< typename Type, bool TF >
inline bool isDefault( const VectorN<Type,TF>& v );

template< typename Type, bool TF >
inline const DVecDVecMultExpr<VectorN<Type,TF>,VectorN<Type,TF>,TF> sq( const VectorN<Type,TF>& v );

template< typename Type, bool TF >
inline void swap( VectorN<Type,TF>& a, VectorN<Type,TF>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks the given dense vector for not-a-number elements.
 * \ingroup dense_vector_N
 *
 * \param v The vector to be checked for not-a-number elements.
 * \return \a true if at least one element of the vector is not-a-number, \a false otherwise.
 *
 * This function checks the N-dimensional dense vector for not-a-number (NaN) elements. If at
 * least one element of the vector is not-a-number, the function returns \a true, otherwise it
 * returns \a false.

   \code
   pe::VecN a;
   // ... Resizing and initialization
   if( isnan( a ) ) { ... }
   \endcode
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline bool isnan( const VectorN<Type,TF>& v )
{
   for( size_t i=0; i<v.size(); ++i ) {
      if( isnan( v[i] ) ) return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given dense vector.
 * \ingroup dense_vector_N
 *
 * \param v The dense vector to be resetted.
 * \return void
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void reset( VectorN<Type,TF>& v )
{
   v.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given dense vector.
 * \ingroup dense_vector_N
 *
 * \param v The dense vector to be cleared.
 * \return void
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void clear( VectorN<Type,TF>& v )
{
   v.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given dense vector is in default state.
 * \ingroup dense_vector_N
 *
 * \param v The dense vector to be tested for its default state.
 * \return \a true in case the given vector is component-wise zero, \a false otherwise.
 *
 * This function checks whether the N-dimensional vector is in default state. For instance,
 * in case the vector is instantiated for a built-in integral or floating point data type,
 * the function returns \a true in case all vector elements are 0 and \a false in case any
 * vector element is not 0. Following example demonstrates the use of the \a isDefault
 * function:

   \code
   pe::VecN a;
   // ... Resizing and initialization
   if( isDefault( a ) ) { ... }
   \endcode
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline bool isDefault( const VectorN<Type,TF>& v )
{
   for( size_t i=0; i<v.size(); ++i )
      if( !isDefault( v[i] ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Squaring the given dense vector.
 * \ingroup dense_vector_N
 *
 * \param v The dense vector to be squared.
 * \return The result of the square operation.
 *
 * This function calculates the component product of the given dense vector. It has the same
 * effect as multiplying the vector with itself (\f$ v * v \f$).
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const DVecDVecMultExpr<VectorN<Type,TF>,VectorN<Type,TF>,TF> sq( const VectorN<Type,TF>& v )
{
   return v * v;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two vectors.
 * \ingroup dense_vector_N
 *
 * \param a The first vector to be swapped.
 * \param b The second vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void swap( VectorN<Type,TF>& a, VectorN<Type,TF>& b ) /* throw() */
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
struct MathTrait< VectorN<T1,TF>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef INVALID_NUMERICAL_TYPE                              AddType;
   typedef INVALID_NUMERICAL_TYPE                              SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef VectorN< typename MathTrait<T1,T2>::DivType , TF >  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T2 );
};

template< typename T1, typename T2, bool TF >
struct MathTrait< T1, VectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef INVALID_NUMERICAL_TYPE                              AddType;
   typedef INVALID_NUMERICAL_TYPE                              SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T1 );
};

template< typename T1, typename T2, bool TF >
struct MathTrait< VectorN<T1,TF>, Vector2<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector2< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector2< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector2< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, Vector2<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,false>, Vector2<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< Vector2<T1,TF>, VectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector2< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector2< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector2< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,true>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,false>, VectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< VectorN<T1,TF>, Vector3<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector3< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector3< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, Vector3<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,false>, Vector3<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< Vector3<T1,TF>, VectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector3< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector3< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector3< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector3<T1,true>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector3<T1,false>, VectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< VectorN<T1,TF>, Vector6<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector6< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector6< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, Vector6<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,false>, Vector6<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< Vector6<T1,TF>, VectorN<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef Vector6< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector6< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector6< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector6<T1,true>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector6<T1,false>, VectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
};

template< typename T1, typename T2, bool TF >
struct MathTrait< VectorN<T1,TF>, VectorN<T2,TF> >
{
   typedef VectorN< typename MathTrait<T1,T2>::HighType, TF >  HighType;
   typedef VectorN< typename MathTrait<T1,T2>::LowType , TF >  LowType;
   typedef VectorN< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef VectorN< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef VectorN< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,true>, VectorN<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< VectorN<T1,false>, VectorN<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef MatrixMxN< typename MathTrait<T1,T2>::MultType >  MultType;
   typedef INVALID_NUMERICAL_TYPE                            DivType;
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
struct TransposeTrait< VectorN<T,TF> >
{
   typedef VectorN<typename TransposeTrait<T>::Type,!TF>  Type;
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief N-dimensional real vector.
 * \ingroup dense_vector_N
 */
typedef VectorN<real,false>  VecN;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief N-dimensional transpose real vector.
 * \ingroup dense_vector_N
 */
typedef VectorN<real,true>  VecNT;
//*************************************************************************************************

} // namespace pe

#endif
