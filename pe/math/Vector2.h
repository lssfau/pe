//=================================================================================================
/*!
 *  \file pe/math/Vector2.h
 *  \brief Header file for the implementation of a 2D vector
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

#ifndef _PE_MATH_VECTOR2_H_
#define _PE_MATH_VECTOR2_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <istream>
#include <ostream>
#include <stdexcept>
#include <boost/type_traits/remove_reference.hpp>
#include <pe/math/CMathTrait.h>
#include <pe/math/DenseVector.h>
#include <pe/math/Functions.h>
#include <pe/math/MathTrait.h>
#include <pe/math/Matrix2x2.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/IsNaN.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/TransposeTrait.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>
#include <pe/util/constraints/Const.h>
#include <pe/util/constraints/FloatingPoint.h>
#include <pe/util/constraints/Numeric.h>
#include <pe/util/constraints/SameType.h>
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
template< typename, bool > struct SparseVector;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup dense_vector_2 Vector2
 * \ingroup dense_vector
 */
/*!\brief Efficient implementation of a 2-dimensional vector.
 * \ingroup dense_vector_2
 *
 * The Vector2 class is the representation of a 2D vector with a total of 2 statically allocated
 * elements of arbitrary type. The naming convention of the elements is as following:

                             \f[\left(\begin{array}{*{2}{c}}
                             x & y \\
                             \end{array}\right)\f]

 * These elements can be accessed directly with the subscript operator. The numbering of the
 * vector elements is

                             \f[\left(\begin{array}{*{2}{c}}
                             0 & 1 \\
                             \end{array}\right)\f]

 * Vector2 can be used with any non-cv-qualified element type. The arithmetic operators for
 * vector/vector and vector/element operations with the same element type work for any element
 * type as long as the element type supports the arithmetic operation. Arithmetic operations
 * between vectors and elements of different element types are only supported for all data types
 * supported by the MathTrait class template (for details see the MathTrait class description).

   \code
   Vector2< double > a, b, c;
   Vector2< float  > d;
   Vector2< std::complex<double> > e, f, g;
   Vector2< std::complex<float>  > h;

   c = a + b;  // OK: Same element type, supported
   c = a + d;  // OK: Different element types, supported by the MathTrait class template

   g = e + f;  // OK: Same element type, supported
   g = e + h;  // Error: Different element types, not supported by the MathTrait class template
   \endcode
 */
template< typename Type      // Data type of the vector
        , bool TF = false >  // Transposition flag
class Vector2 : public DenseVector< Vector2<Type,TF>, TF >
{
public:
   //**Type definitions****************************************************************************
   typedef Vector2<Type,TF>  This;           //!< Type of this Vector2 instance.
   typedef This              ResultType;     //!< Result type for expression template evaluations.
   typedef Type              ElementType;    //!< Type of the vector elements.
   typedef const Vector2&    CompositeType;  //!< Data type for composite expression templates.

   //! Transpose type for expression template evaluations.
   typedef Vector2< typename TransposeTrait<Type>::Type, !TF >  TransposeType;

   //! Vector length return type.
   /*! Return type of the Vector2<Type>::length function. */
   typedef typename CMathTrait<Type>::Type  LengthType;
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
                              explicit inline Vector2();
                              explicit inline Vector2( Type init );
                              explicit inline Vector2( Type x, Type y );
                                       inline Vector2( const Vector2& v );
   template< typename Other >          inline Vector2( const Vector2<Other,TF>& v );
   template< typename VT >             inline Vector2( const DenseVector<VT,TF>&  dv );
   template< typename VT >             inline Vector2( const SparseVector<VT,TF>& sv );
   template< typename Other > explicit inline Vector2( const Other (&rhs)[2] );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
                              inline Vector2&    operator= ( Type rhs );
                              inline Vector2&    operator= ( const Vector2& rhs );
   template< typename Other > inline Vector2&    operator= ( const Vector2<Other,TF>&   rhs );
   template< typename VT >    inline Vector2&    operator= ( const DenseVector<VT,TF>&  rhs );
   template< typename VT >    inline Vector2&    operator= ( const SparseVector<VT,TF>& rhs );
   template< typename Other > inline Vector2&    operator= ( const Other (&rhs)[2] );
                              inline Type&       operator[]( size_t index );
                              inline const Type& operator[]( size_t index ) const;
   template< typename VT >    inline Vector2&    operator+=( const DenseVector<VT,TF>&  rhs );
   template< typename VT >    inline Vector2&    operator+=( const SparseVector<VT,TF>& rhs );
   template< typename VT >    inline Vector2&    operator-=( const DenseVector<VT,TF>&  rhs );
   template< typename VT >    inline Vector2&    operator-=( const SparseVector<VT,TF>& rhs );
   template< typename VT >    inline Vector2&    operator*=( const DenseVector<VT,TF>&  rhs );
   template< typename VT >    inline Vector2&    operator*=( const SparseVector<VT,TF>& rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, Vector2 >::Type&
      operator*=( Other rhs );

   template< typename Other >
   inline typename EnableIf< IsNumeric<Other>, Vector2 >::Type&
      operator/=( Other rhs );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
                              inline size_t        size()          const;
                              inline Vector2&      set( Type x, Type y );
                              inline void          reset();
                              inline LengthType    length()        const;
                              inline Type          sqrLength()     const;
                              inline Vector2&      normalize();
                              inline const Vector2 getNormalized() const;
                              inline const Vector2 perp()          const;
   template< typename Other > inline Vector2&      scale( Other scalar );
                              inline Type          min()           const;
                              inline Type          max()           const;
                              inline void          swap( Vector2& v ) /* throw() */;
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
   Type v_[2];  //!< The two statically allocated vector elements.
                /*!< Access to the vector values is gained via the subscript operator.
                     The order of the elements is
                     \f[\left(\begin{array}{*{2}{c}}
                     0 & 1 \\
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
/*!\brief The default constructor for Vector2.
 *
 * All vector elements are initialized to the default value (i.e. 0 for integral data types).
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Vector2<Type,TF>::Vector2()
{
   v_[0] = v_[1] = Type();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a homogenous initialization of all elements.
 *
 * \param init Initial value for all vector elements.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Vector2<Type,TF>::Vector2( Type init )
{
   v_[0] = v_[1] = init;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for a direct initialization of all vector elements.
 *
 * \param x The initial value for the x-component.
 * \param y The initial value for the y-component.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Vector2<Type,TF>::Vector2( Type x, Type y )
{
   v_[0] = x;
   v_[1] = y;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The copy constructor for Vector2.
 *
 * \param v Vector to be copied.
 *
 * The copy constructor is explicitly defined in order to enable/facilitate NRV optimization.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Vector2<Type,TF>::Vector2( const Vector2& v )
{
   v_[0] = v.v_[0];
   v_[1] = v.v_[1];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different Vector2 instances.
 *
 * \param v Vector to be copied.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the foreign vector
inline Vector2<Type,TF>::Vector2( const Vector2<Other,TF>& v )
{
   v_[0] = v[0];
   v_[1] = v[1];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from different dense vectors.
 *
 * \param dv Dense vector to be copied.
 * \exception std::invalid_argument Invalid setup of 2-dimensional vector.
 *
 * This constructor initializes the 2-dimensional vector from the given dense vector. In case
 * the size of the given vector does not match the size of the 2-dimensional vector (i.e. is
 * not 2), a \a std::invalid_argument exception is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the foreign dense vector
inline Vector2<Type,TF>::Vector2( const DenseVector<VT,TF>& dv )
{
   using pe::assign;

   if( (~dv).size() != size_t(2) )
      throw std::invalid_argument( "Invalid setup of 2-dimensional vector" );

   assign( *this, ~dv );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from sparse vectors.
 *
 * \param sv Sparse vector to be copied.
 * \exception std::invalid_argument Invalid setup of 2-dimensional vector.
 *
 * This constructor initializes the 2-dimensional vector from the given sparse vector. In case
 * the size of the given vector does not match the size of the 2-dimensional vector (i.e. is
 * not 2), a \a std::invalid_argument exception is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the foreign sparse vector
inline Vector2<Type,TF>::Vector2( const SparseVector<VT,TF>& sv )
{
   using pe::assign;

   if( (~sv).size() != size_t(2) )
      throw std::invalid_argument( "Invalid setup of 2-dimensional vector" );

   if( IsBuiltin<Type>::value )
      reset();

   assign( *this, ~sv );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Array initialization of all vector elements.
 *
 * \param rhs 2-dimensional array for the initialization.
 *
 * This assignment operator offers the option to directly initialize the elements of the vector:

   \code
   const real init[2] = { 1, 2 };
   Vector2<real> v( init );
   \endcode

 * The vector is initialized with the given values. Missing values are initialized with zero.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the initialization array
inline Vector2<Type,TF>::Vector2( const Other (&rhs)[2] )
{
   v_[0] = rhs[0];
   v_[1] = rhs[1];
}
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Homogenous assignment to all vector elements.
 *
 * \param rhs Scalar value to be assigned to all vector elements.
 * \return Reference to the assigned vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Vector2<Type,TF>& Vector2<Type,TF>::operator=( Type rhs )
{
   v_[0] = v_[1] = rhs;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Copy assignment operator for Vector2.
 *
 * \param rhs Vector to be copied.
 * \return Reference to the assigned vector.
 *
 * Explicit definition of a copy assignment operator for performance reasons.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Vector2<Type,TF>& Vector2<Type,TF>::operator=( const Vector2<Type,TF>& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   v_[0] = rhs.v_[0];
   v_[1] = rhs.v_[1];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different Vector2 instances.
 *
 * \param rhs Vector to be copied.
 * \return Reference to the assigned vector.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the foreign vector
inline Vector2<Type,TF>& Vector2<Type,TF>::operator=( const Vector2<Other,TF>& rhs )
{
   // This implementation is faster than the synthesized default copy assignment operator and
   // faster than an implementation with the C library function 'memcpy' in combination with a
   // protection against self-assignment. Additionally, this version goes without a protection
   // against self-assignment.
   v_[0] = rhs[0];
   v_[1] = rhs[1];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Assignment operator for different dense vectors.
 *
 * \param rhs Dense vector to be copied.
 * \return Reference to the assigned vector.
 * \exception std::invalid_argument Invalid assignment to 2-dimensional vector.
 *
 * This constructor initializes the vector as a copy of the given dense vector. In case the
 * size of the given dense vector is not 2, a \a std::invalid_argument exception is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side dense vector
inline Vector2<Type,TF>& Vector2<Type,TF>::operator=( const DenseVector<VT,TF>& rhs )
{
   using pe::assign;

   if( (~rhs).size() != size_t(2) )
      throw std::invalid_argument( "Invalid assignment to 2-dimensional vector" );

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      Vector2 tmp( rhs );
      swap( tmp );
   }
   else {
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
 * \exception std::invalid_argument Invalid assignment to 2-dimensional vector.
 *
 * This constructor initializes the vector as a copy of the given sparse vector. In case the
 * size of the given sparse vector is not 2, a \a std::invalid_argument exception is thrown.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
template< typename VT >  // Type of the right-hand side sparse vector
inline Vector2<Type,TF>& Vector2<Type,TF>::operator=( const SparseVector<VT,TF>& rhs )
{
   using pe::assign;

   if( (~rhs).size() != size_t(2) )
      throw std::invalid_argument( "Invalid assignment to 2-dimensional vector" );

   reset();
   assign( *this, ~rhs );

   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Array assignment to all vector elements.
 *
 * \param rhs 2-dimensional array for the assignment.
 * \return Reference to the assigned vector.
 *
 * This assignment operator offers the option to directly set all elements of the vector:

   \code
   const real init[2] = { 1, 2 };
   Vector2<real> v;
   v = init;
   \endcode

 * The vector is initialized with the given values. Missing values are initialized with zero.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the initialization array
inline Vector2<Type,TF>&
   Vector2<Type,TF>::operator=( const Other (&rhs)[2] )
{
   v_[0] = rhs[0];
   v_[1] = rhs[1];
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..1]\f$.
 * \return Reference to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Type& Vector2<Type,TF>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < 2, "Invalid vector access index" );
   return v_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to the vector elements.
 *
 * \param index Access index. The index has to be in the range \f$[0..1]\f$.
 * \return Reference-to-const to the accessed value.
 *
 * In case pe_USER_ASSERT() is active, this operator performs an index check.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Type& Vector2<Type,TF>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < 2, "Invalid vector access index" );
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
inline Vector2<Type,TF>& Vector2<Type,TF>::operator+=( const DenseVector<VT,TF>& rhs )
{
   using pe::addAssign;

   if( (~rhs).size() != size_t(2) )
      throw std::invalid_argument( "Vector sizes do not match" );

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      Vector2 tmp( rhs );
      v_[0] += tmp[0];
      v_[1] += tmp[1];
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
inline Vector2<Type,TF>& Vector2<Type,TF>::operator+=( const SparseVector<VT,TF>& rhs )
{
   using pe::addAssign;

   if( (~rhs).size() != size_t(2) )
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
inline Vector2<Type,TF>& Vector2<Type,TF>::operator-=( const DenseVector<VT,TF>& rhs )
{
   using pe::subAssign;

   if( (~rhs).size() != size_t(2) )
      throw std::invalid_argument( "Vector sizes do not match" );

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      Vector2 tmp( rhs );
      v_[0] -= tmp[0];
      v_[1] -= tmp[1];
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
inline Vector2<Type,TF>& Vector2<Type,TF>::operator-=( const SparseVector<VT,TF>& rhs )
{
   using pe::subAssign;

   if( (~rhs).size() != size_t(2) )
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
inline Vector2<Type,TF>& Vector2<Type,TF>::operator*=( const DenseVector<VT,TF>& rhs )
{
   using pe::multAssign;

   if( (~rhs).size() != size_t(2) )
      throw std::invalid_argument( "Vector sizes do not match" );

   if( IsExpression<VT>::value && (~rhs).isAliased( this ) ) {
      Vector2 tmp( rhs );
      v_[0] *= tmp[0];
      v_[1] *= tmp[1];
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
inline Vector2<Type,TF>& Vector2<Type,TF>::operator*=( const SparseVector<VT,TF>& rhs )
{
   using pe::multAssign;

   if( (~rhs).size() != size_t(2) )
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
inline typename EnableIf< IsNumeric<Other>, Vector2<Type,TF> >::Type&
   Vector2<Type,TF>::operator*=( Other rhs )
{
   v_[0] *= rhs;
   v_[1] *= rhs;
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
inline typename EnableIf< IsNumeric<Other>, Vector2<Type,TF> >::Type&
   Vector2<Type,TF>::operator/=( Other rhs )
{
   pe_USER_ASSERT( rhs != Other(0), "Division by zero detected" );

   typedef typename MathTrait<Type,Other>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( rhs ) );
      v_[0] = static_cast<Type>( static_cast<DT>( v_[0] ) * tmp );
      v_[1] = static_cast<Type>( static_cast<DT>( v_[1] ) * tmp );
      return *this;
   }
   else {
      v_[0] /= rhs;
      v_[1] /= rhs;
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
/*!\brief Returns the current size/dimension of the vector.
 *
 * \return The size of the vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline size_t Vector2<Type,TF>::size() const
{
   return size_t(2);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the value of the vector elements.
 *
 * \param x The value for the x-component.
 * \param y The value for the y-component.
 * \return Reference to the vector.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Vector2<Type,TF>& Vector2<Type,TF>::set( Type x, Type y )
{
   v_[0] = x;
   v_[1] = y;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reset to the default initial values.
 *
 * \return void
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void Vector2<Type,TF>::reset()
{
   using pe::reset;
   reset( v_[0] );
   reset( v_[1] );
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
inline typename Vector2<Type,TF>::LengthType Vector2<Type,TF>::length() const
#else
inline typename CMathTrait<Type>::Type Vector2<Type,TF>::length() const
#endif
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( Type );
   return std::sqrt( v_[0]*v_[0] + v_[1]*v_[1] );
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
inline Type Vector2<Type,TF>::sqrLength() const
{
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( Type );
   return ( v_[0]*v_[0] + v_[1]*v_[1] );
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
inline Vector2<Type,TF>& Vector2<Type,TF>::normalize()
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   const Type len( std::sqrt( v_[0]*v_[0] + v_[1]*v_[1] ) );

   if( len == Type(0) )
      return *this;

   const Type ilen( Type(1) / len );

   v_[0] *= ilen;
   v_[1] *= ilen;

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
inline const Vector2<Type,TF> Vector2<Type,TF>::getNormalized() const
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( Type );

   const Type len( std::sqrt( v_[0]*v_[0] + v_[1]*v_[1] ) );

   if( len == Type(0) )
      return *this;

   const Type ilen( Type(1) / len );

   return Vector2( v_[0]*ilen, v_[1]*ilen );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unary perp dot product operator for the calculation of a perpendicular vector
 *        (\f$ \vec{a}=\vec{b}^\perp \f$).
 *
 * \return The perpendicular vector.
 *
 * The "perp dot product" \f$ \vec{a}^\perp \cdot b \f$ for the vectors \f$ \vec{a} \f$ and
 * \f$ \vec{b} \f$ is a modification of the two-dimensional dot product in which \f$ \vec{a} \f$
 * is replaced by the perpendicular vector rotated 90 degrees to the left defined by Hill (1994).
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Vector2<Type,TF> Vector2<Type,TF>::perp() const
{
   return Vector2( -v_[1], v_[0] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Scaling of the vector by the scalar value \a scalar (\f$ \vec{a}*=s \f$).
 *
 * \param scalar The scalar value for the vector scaling.
 * \return Reference to the vector.
 */
template< typename Type     // Data type of the vector
        , bool TF >         // Transposition flag
template< typename Other >  // Data type of the scalar value
inline Vector2<Type,TF>& Vector2<Type,TF>::scale( Other scalar )
{
   v_[0] *= scalar;
   v_[1] *= scalar;
   return *this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the smallest element of the vector.
 *
 * \return The smallest vector element.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Type Vector2<Type,TF>::min() const
{
   using pe::min;
   return min( v_[0], v_[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the largest element of the vector.
 *
 * \return The largest vector element.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline Type Vector2<Type,TF>::max() const
{
   using pe::max;
   return max( v_[0], v_[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two 2D vectors.
 *
 * \param v The vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void Vector2<Type,TF>::swap( Vector2& v ) /* throw() */
{
   std::swap( v_[0], v.v_[0] );
   std::swap( v_[1], v.v_[1] );
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
inline bool Vector2<Type,TF>::isAliased( const Other* alias ) const
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
inline void Vector2<Type,TF>::assign( const DenseVector<VT,TF>& rhs )
{
   v_[0] = (~rhs)[0];
   v_[1] = (~rhs)[1];
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
inline void Vector2<Type,TF>::assign( const SparseVector<VT,TF>& rhs )
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
inline void Vector2<Type,TF>::addAssign( const DenseVector<VT,TF>& rhs )
{
   v_[0] += (~rhs)[0];
   v_[1] += (~rhs)[1];
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
inline void Vector2<Type,TF>::addAssign( const SparseVector<VT,TF>& rhs )
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
inline void Vector2<Type,TF>::subAssign( const DenseVector<VT,TF>& rhs )
{
   v_[0] -= (~rhs)[0];
   v_[1] -= (~rhs)[1];
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
inline void Vector2<Type,TF>::subAssign( const SparseVector<VT,TF>& rhs )
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
inline void Vector2<Type,TF>::multAssign( const DenseVector<VT,TF>& rhs )
{
   v_[0] *= (~rhs)[0];
   v_[1] *= (~rhs)[1];
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
inline void Vector2<Type,TF>::multAssign( const SparseVector<VT,TF>& rhs )
{
   typedef typename VT::ConstIterator  ConstIterator;

   const Vector2 tmp( *this );
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
/*!\name Vector2 operators */
//@{
template< typename T1, typename T2, bool TF >
inline bool operator==( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const Vector2<T1,TF>& vec, T2 scalar );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const Vector2<T2,TF>& vec );

template< typename T1, typename T2, bool TF >
inline bool operator!=( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const Vector2<T1,TF>& vec, T2 scalar );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const Vector2<T2,TF>& vec );

template< typename Type, bool TF >
std::ostream& operator<<( std::ostream& os, const Vector2<Type,TF>& v );

template< typename Type, bool TF >
std::istream& operator>>( std::istream& is, Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline bool isnan( const Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline const Vector2<Type,TF> abs( const Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline const Vector2<Type,TF> fabs( const Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline void reset( Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline void clear( Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline bool isDefault( const Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline const Vector2< typename MathTrait<Type,Type>::MultType, TF > sq( const Vector2<Type,TF>& v );

template< typename Type, bool TF >
inline void swap( Vector2<Type,TF>& a, Vector2<Type,TF>& b ) /* throw() */;
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two vectors.
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side vector for the comparison.
 * \param rhs The right-hand side vector for the comparison.
 * \return \a true if the two vectors are equal, \a false if not.
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline bool operator==( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs )
{
   // In order to compare the two vectors, the data values of the lower-order data
   // type are converted to the higher-order data type within the equal function.
   if( !equal( lhs[0], rhs[0] ) || !equal( lhs[1], rhs[1] ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a vector and a scalar value.
 * \ingroup dense_vector_2
 *
 * \param vec The left-hand side vector for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if all elements of the vector are equal to the scalar, \a false if not.
 *
 * If all values of the vector are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const Vector2<T1,TF>& vec, T2 scalar )
{
   // In order to compare the vector and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type within the equal function.
   if( !equal( vec[0], scalar ) || !equal( vec[1], scalar ) )
      return false;
   else return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a scalar value and a vector.
 * \ingroup dense_vector_2
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param vec The right-hand side vector for the comparison.
 * \return \a true if all elements of the vector are equal to the scalar, \a false if not.
 *
 * If all values of the vector are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1  // Data type of the left-hand side scalar
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const Vector2<T2,TF>& vec )
{
   return ( vec == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two vectors.
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side vector for the comparison.
 * \param rhs The right-hand side vector for the comparison.
 * \return \a true if the two vectors are not equal, \a false if they are equal.
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline bool operator!=( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a vector and a scalar value.
 * \ingroup dense_vector_2
 *
 * \param vec The left-hand side vector for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if at least one element of the vector is different from the scalar, \a false if not.
 *
 * If one value of the vector is inequal to the scalar value, the inequality test returns
 * \a true, otherwise \a false. Note that this function can only be used with built-in,
 * numerical data types!
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const Vector2<T1,TF>& vec, T2 scalar )
{
   return !( vec == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a scalar value and a vector.
 * \ingroup dense_vector_2
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param vec The right-hand side vector for the comparison.
 * \return \a true if at least one element of the vector is different from the scalar, \a false if not.
 *
 * If one value of the vector is inequal to the scalar value, the inequality test returns
 * \a true, otherwise \a false. Note that this function can only be used with built-in,
 * numerical data types!
 */
template< typename T1  // Data type of the left-hand side scalar
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const Vector2<T2,TF>& vec )
{
   return !( vec == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for 2-dimensional vectors.
 * \ingroup dense_vector_2
 *
 * \param os Reference to the output stream.
 * \param v Reference to a constant vector object.
 * \return Reference to the output stream.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
std::ostream& operator<<( std::ostream& os, const Vector2<Type,TF>& v )
{
   return os << "<" << v[0] << "," << v[1] << ">";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global input operator for 2-dimensional vectors.
 * \ingroup dense_vector_2
 *
 * \param is Reference to the input stream.
 * \param v Reference to a vector object.
 * \return The input stream.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
std::istream& operator>>( std::istream& is, Vector2<Type,TF>& v )
{
   if( !is ) return is;

   char bracket1, bracket2, comma;
   Type x(0.0), y(0.0);
   const std::istream::pos_type pos( is.tellg() );
   const std::istream::fmtflags oldFlags( is.flags() );

   // Setting the 'skip whitespaces' flag
   is >> std::skipws;

   // Extracting the vector
   if( !(is >> bracket1 >> x >> comma >> y >> bracket2) ||
       bracket1 != '<' || comma != ',' || bracket2 != '>' ) {
      is.clear();
      is.seekg( pos );
      is.setstate( std::istream::failbit );
      is.flags( oldFlags );
      return is;
   }

   // Transfering the input to the vector values
   v.set( x, y );

   // Resetting the flags
   is.flags( oldFlags );

   return is;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks the given vector for not-a-number elements.
 * \ingroup dense_vector_2
 *
 * \param v The vector to be checked for not-a-number elements.
 * \return \a true if at least one element of the vector is not-a-number, \a false otherwise.
 *
 * This function checks the 2D vector for not-a-number (NaN) elements. If at least one element
 * of the vector is not-a-number, the function returns \a true, otherwise it returns \a false.

   \code
   pe::Vec2 a;
   // ... Initialization
   if( isnan( a ) ) { ... }
   \endcode
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline bool isnan( const Vector2<Type,TF>& v )
{
   if( isnan( v[0] ) || isnan( v[1] ) )
      return true;
   else return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a vector containing the absolute values of each single element of \a v.
 * \ingroup dense_vector_2
 *
 * \param v The integral input vector.
 * \return The absolute value of each single element of \a v.
 *
 * The \a abs function calculates the absolute value of each element of the input vector \a v.\n
 * The following example demonstrates the use of the \a abs function:

   \code
   pe::Vec2 a, b;
   // ... Initialization
   b = abs( a );
   \endcode
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Vector2<Type,TF> abs( const Vector2<Type,TF>& v )
{
   using std::abs;
   return Vector2<Type,TF>( abs(v[0]), abs(v[1]) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a vector containing the absolute values of each single element of \a v.
 * \ingroup dense_vector_2
 *
 * \param v The floating point input vector.
 * \return The absolute value of each single element of \a v.
 *
 * The \a fabs function calculates the absolute value of each element of the input vector \a v.\n
 * The following example demonstrates the use of the \a fabs function:

   \code
   pe::Vec2 a, b;
   // ... Initialization
   b = fabs( a );
   \endcode
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Vector2<Type,TF> fabs( const Vector2<Type,TF>& v )
{
   using std::fabs;
   return Vector2<Type,TF>( fabs(v[0]), fabs(v[1]) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the given 2D vector.
 * \ingroup dense_vector_2
 *
 * \param v The vector to be resetted.
 * \return void
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void reset( Vector2<Type,TF>& v )
{
   v.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the given 2D vector.
 * \ingroup dense_vector_2
 *
 * \param v The vector to be cleared.
 * \return void
 *
 * Clearing a 2-dimensional vector is equivalent to resetting it via the reset() function.
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline void clear( Vector2<Type,TF>& v )
{
   v.reset();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the given 2D vector is in default state.
 * \ingroup dense_vector_2
 *
 * \param v The vector to be tested for its default state.
 * \return \a true in case the given vector is component-wise zero, \a false otherwise.
 *
 * This function checks whether the 2D vector is in default state. For instance, in case the
 * 2D vector is instantiated for a built-in integral or floating point data type, the function
 * returns \a true in case all vector elements are 0 and \a false in case any vector element
 * is not 0. Following example demonstrates the use of the \a isDefault function:

   \code
   pe::Vec2 a;
   // ... Initialization
   if( isDefault( a ) ) { ... }
   \endcode
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline bool isDefault( const Vector2<Type,TF>& v )
{
   return isDefault( v[0] ) && isDefault( v[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Squaring the given 2D vector.
 * \ingroup dense_vector_2
 *
 * \param v The vector to be squared.
 * \return The result of the square operation.
 *
 * This function calculates the component product of the given 2-dimensional vector. It has the
 * same effect as multiplying the vector with itself (\f$ v * v \f$).
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Vector2< typename MathTrait<Type,Type>::MultType, TF > sq( const Vector2<Type,TF>& v )
{
   return v * v;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Swapping the contents of two 2D vectors.
 * \ingroup dense_vector_2
 *
 * \param a The first vector to be swapped.
 * \param b The second vector to be swapped.
 * \return void
 * \exception no-throw guarantee.
 */
template< typename Type  // Data type of the vectors
        , bool TF >      // Transposition flag
inline void swap( Vector2<Type,TF>& a, Vector2<Type,TF>& b ) /* throw() */
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
/*!\name Vector2 unary arithmetic operators */
//@{
template< typename Type, bool TF >
inline const Vector2<Type,TF> operator-( const Vector2<Type,TF>& v );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unary minus operator for the inversion of a 2D vector (\f$ \vec{a} = -\vec{b} \f$).
 * \ingroup dense_vector_2
 *
 * \param v The vector to be negated.
 * \return The negation of the vector.
 *
 * This operator represents the negation of a 2-dimensional vector:

   \code
   pe::Vec2 a, b;
   // ... Initialization
   b = -a;
   \endcode
 */
template< typename Type  // Data type of the vector
        , bool TF >      // Transposition flag
inline const Vector2<Type,TF> operator-( const Vector2<Type,TF>& v )
{
   return Vector2<Type,TF>( -v[0], -v[1] );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BINARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Vector2 binary arithmetic operators */
//@{
template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait<T1,T2>::AddType, TF >
   operator+( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait<T1,T2>::SubType, TF >
   operator-( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType, TF >
   operator*( const Vector2<T1,TF>& vec, T2 scalar );

template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType, TF >
   operator*( T1 scalar, const Vector2<T2,TF>& vec );

template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait<T1,T2>::MultType, TF >
   operator*( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs );

template< typename T1, typename T2 >
inline const typename MathTrait<T1,T2>::MultType
   operator*( const Vector2<T1,true>& lhs, const Vector2<T2,false>& rhs );

template< typename T1, typename T2 >
inline const Matrix2x2< typename MathTrait<T1,T2>::MultType >
   operator*( const Vector2<T1,false>& lhs, const Vector2<T2,true>& rhs );

template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait<T1,typename T2::ElementType>::MultType, TF >
   operator*( const Vector2<T1,TF>& lhs, const SparseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait<typename T1::ElementType,T2>::MultType, TF >
   operator*( const SparseVector<T1,TF>& lhs, const Vector2<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline const Vector2< typename MathTrait< T1, T2, IsNumeric<T1>::value && IsNumeric<T2>::value >::DivType, TF >
   operator/( const Vector2<T1,TF>& vec, T2 scalar );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Addition operator for the addition of two 2D vectors (\f$ \vec{a}=\vec{b}+\vec{c} \f$).
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side 2D vector for the vector addition.
 * \param rhs The right-hand side 2D vector for the vector addition.
 * \return The sum of the two vectors.
 *
 * This operator represents the addition of two 2-dimensional vectors:

   \code
   pe::Vec2 a, b, c;
   // ... Initialization
   c = a + b;
   \endcode

 * The operator returns a 2-dimensional vector of the higher-order element type of the two
 * involved vector element types \a T1 and \a T2. Both element types \a T1 and \a T2 have
 * to be supported by the MathTrait class template.
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait<T1,T2>::AddType, TF >
   operator+( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs )
{
   typedef typename MathTrait<T1,T2>::AddType  AT;
   return Vector2<AT,TF>( lhs[0]+rhs[0], lhs[1]+rhs[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subtraction operator for the subtraction of two 2D vectors (\f$ \vec{a}=\vec{b}-\vec{c} \f$).
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side 2D vector for the vector subtraction.
 * \param rhs The right-hand side 2D vector to be subtracted from the left-hand side vector.
 * \return The difference of the two vectors.
 *
 * This operator represents the subtraction of two 2-dimensional vectors:

   \code
   pe::Vec2 a, b, c;
   // ... Initialization
   c = a - b;
   \endcode

 * The operator returns a 2-dimensional vector of the higher-order element type of the two
 * involved vector element types \a T1 and \a T2. Both element types \a T1 and \a T2 have
 * to be supported by the MathTrait class template.
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait<T1,T2>::SubType, TF >
   operator-( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs )
{
   typedef typename MathTrait<T1,T2>::SubType  ST;
   return Vector2<ST,TF>( lhs[0]-rhs[0], lhs[1]-rhs[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a 2D vector and a scalar value
 *        (\f$ \vec{a}=\vec{b}*s \f$).
 * \ingroup dense_vector_2
 *
 * \param vec The left-hand side 2D vector for the multiplication.
 * \param scalar The right-hand side scalar value for the multiplication.
 * \return The scaled result vector.
 *
 * This operator represents the multiplication of a 2-dimensional vector and a scalar value:

   \code
   pe::Vec2 a, b;
   // ... Initialization
   b = a * 1.25;
   \endcode

 * The operator returns a 2-dimensional vector of the higher-order element type of the two
 * involved data types \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported
 * by the MathTrait class template. Note that this operator only works for scalar values of
 * built-in data type.
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait<T1,T2,IsNumeric<T2>::value>::MultType, TF >
   operator*( const Vector2<T1,TF>& vec, T2 scalar )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Vector2<MT,TF>( vec[0]*scalar, vec[1]*scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the multiplication of a scalar value and a 2D vector
 *        (\f$ \vec{a}=s*\vec{b} \f$).
 * \ingroup dense_vector_2
 *
 * \param scalar The left-hand side scalar value for the multiplication.
 * \param vec The right-hand side 2D vector for the multiplication.
 * \return The scaled result vector.
 *
 * This operator represents the multiplication of a scalar value and a 2-dimensional vector:

   \code
   pe::Vec2 a, b;
   // ... Initialization
   b = 1.25 * a;
   \endcode

 * The operator returns a 2-dimensional vector of the higher-order element type of the two
 * involved data types \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported
 * by the MathTrait class template. Note that this operator only works for scalar values of
 * built-in data type.
 */
template< typename T1  // Data type of the left-hand side scalar
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait<T1,T2,IsNumeric<T1>::value>::MultType, TF >
   operator*( T1 scalar, const Vector2<T2,TF>& vec )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Vector2<MT,TF>( vec[0]*scalar, vec[1]*scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the componentwise product of two 2D vectors
 *        (\f$ \vec{c}=\vec{a}*\vec{b} \f$).
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side 2D vector for the component product.
 * \param rhs The right-hand side 2D vector for the component product.
 * \return The product of the two vectors.
 *
 * This operator represents the component product of two 2D vectors:

   \code
   pe::Vec2 a, b, c;
   // ... Initialization
   c = a * b;
   \endcode

 * The operator returns a 2-dimensional vector of the higher-order element type of the two
 * involved vector element types \a T1 and \a T2. Both element types \a T1 and \a T2 have
 * to be supported by the MathTrait class template.
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side vector
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait<T1,T2>::MultType, TF >
   operator*( const Vector2<T1,TF>& lhs, const Vector2<T2,TF>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Vector2<MT,TF>( lhs[0]*rhs[0], lhs[1]*rhs[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the scalar product (inner product) of two 2D vectors
 *        (\f$ s=\vec{a}*\vec{b} \f$).
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side 2D vector for the inner product.
 * \param rhs The right-hand side 2D vector for the inner product.
 * \return The scalar product.
 *
 * This operator represents the scalar product (inner product) of two 2D vectors:

   \code
   pe::Vec2 a, b;
   pe::real res;
   // ... Initialization
   res = trans(a) * b;
   \endcode

 * The operator returns a scalar value of the higher-order element type of the two involved
 * vector element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be
 * supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side vector
        , typename T2 >  // Data type of the right-hand side vector
inline const typename MathTrait<T1,T2>::MultType
   operator*( const Vector2<T1,true>& lhs, const Vector2<T2,false>& rhs )
{
   return ( lhs[0]*rhs[0] + lhs[1]*rhs[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the outer product of two 2D vectors
 *        (\f$ s=\vec{a}*\vec{b} \f$).
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side 2D vector for the outer product.
 * \param rhs The right-hand side transpose 2D vector for the outer product.
 * \return The resulting 2x2 matrix.
 *
 * This operator represents the outer product between a 2D vector and a transpose 2D vector:

   \code
   pe::Vec2 a, b;
   pe::Mat2 A;
   // ... Initialization
   A = a * trans(b);
   \endcode

 * The operator returns a 2x2 matrix of the higher-order element type of the two involved
 * vector element types \a T1 and \a T2. Both element types \a T1 and \a T2 have to be
 * supported by the MathTrait class template.
 */
template< typename T1    // Data type of the left-hand side vector
        , typename T2 >  // Data type of the right-hand side vector
inline const Matrix2x2< typename MathTrait<T1,T2>::MultType >
   operator*( const Vector2<T1,false>& lhs, const Vector2<T2,true>& rhs )
{
   typedef typename MathTrait<T1,T2>::MultType  MT;
   return Matrix2x2<MT>( lhs[0]*rhs[0], lhs[0]*rhs[1], lhs[1]*rhs[0], lhs[1]*rhs[1] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the componentwise product of a 2D vector and a sparse
 *        vector (\f$ \vec{a}=\vec{b}*\vec{c} \f$).
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side 2D vector for the component product.
 * \param rhs The right-hand side sparse vector for the component product.
 * \return The product of the two vectors.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the component product of a 2D vector and a sparse vector:

   \code
   pe::Vec2 a, c;
   pe::SVecN b( 2 );
   // ... Resizing and initialization
   c = a * b;
   \endcode

 * The operator returns a 2D vector of the higher-order element type of the two involved vector
 * element types \a T1 and \a T2::ElementType. Both the sparse vector type \a T2 as well as the
 * two element types \a T1 and \a T2::ElementType have to be supported by the MathTrait class
 * template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1  // Data type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait<T1,typename T2::ElementType>::MultType, TF >
   operator*( const Vector2<T1,TF>& lhs, const SparseVector<T2,TF>& rhs )
{
   using boost::remove_reference;

   typedef typename T2::CompositeType                           Rhs;            // Composite type of the right-hand side sparse vector expression
   typedef Vector2<T1,TF>                                       RT1;            // Result type of the left-hand side 3D vector expression
   typedef typename T2::ResultType                              RT2;            // Result type of the right-hand side sparse vector expression
   typedef typename remove_reference<Rhs>::type                 X2;             // Auxiliary type for the right-hand side composite type
   typedef typename X2::ElementType                             ET2;            // Element type of the right-hand side sparse vector expression
   typedef typename X2::ConstIterator                           ConstIterator;  // Iterator type of the right-hand sparse vector expression
   typedef Vector2< typename MathTrait<T1,ET2>::MultType, TF >  MT1;            // Multiplication result type
   typedef typename MathTrait<RT1,RT2>::MultType                MT2;            // Multiplication result type (MathTrait definition)

   pe_CONSTRAINT_MUST_BE_SAME_TYPE( MT1, MT2 );
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( RT1, typename RT1::ResultType );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( T2  );
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE ( MT1 );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T2 , TF );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( MT1, TF );

   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   Rhs right( ~rhs );

   MT1 tmp;

   for( ConstIterator element=right.begin(); element!=right.end(); ++element )
      tmp[element->index()] = lhs[element->index()] * element->value();

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Multiplication operator for the componentwise product of a sparse vector and a
 *        2D vector (\f$ \vec{a}=\vec{b}*\vec{c} \f$).
 * \ingroup dense_vector_2
 *
 * \param lhs The left-hand side sparse vector for the component product.
 * \param rhs The right-hand side 2D vector for the component product.
 * \return The product of the two vectors.
 * \exception std::invalid_argument Vector sizes do not match.
 *
 * This operator represents the component product of a sparse vector and a 2D vector:

   \code
   pe::SVecN a( 2 );
   pe::Vec2 b, c;
   // ... Resizing and initialization
   c = a * b;
   \endcode

 * The operator returns a 2D vector of the higher-order element type of the two involved vector
 * element types \a T1::ElementType and \a T2. Both the sparse vector type \a T1 as well as the
 * two element types \a T1::ElementType and \a T2 have to be supported by the MathTrait class
 * template.\n
 * In case the current sizes of the two given vectors don't match, a \a std::invalid_argument
 * is thrown.
 */
template< typename T1  // Type of the left-hand side sparse vector
        , typename T2  // Data type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait<typename T1::ElementType,T2>::MultType, TF >
   operator*( const SparseVector<T1,TF>& lhs, const Vector2<T2,TF>& rhs )
{
   using boost::remove_reference;

   typedef typename T1::CompositeType                           Lhs;            // Composite type of the left-hand side sparse vector expression
   typedef typename T1::ResultType                              RT1;            // Result type of the left-hand side sparse vector expression
   typedef Vector2<T2,TF>                                       RT2;            // Result type of the right-hand side 3D vector expression
   typedef typename remove_reference<Lhs>::type                 X1;             // Auxiliary type for the left-hand side composite type
   typedef typename X1::ElementType                             ET1;            // Element type of the left-hand side sparse vector expression
   typedef typename X1::ConstIterator                           ConstIterator;  // Iterator type of the left-hand sparse vector expression
   typedef Vector2< typename MathTrait<ET1,T2>::MultType, TF >  MT1;            // Multiplication result type
   typedef typename MathTrait<RT1,RT2>::MultType                MT2;            // Multiplication result type (MathTrait definition)

   pe_CONSTRAINT_MUST_BE_SAME_TYPE( MT1, MT2 );
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( RT2, typename RT2::ResultType );
   pe_CONSTRAINT_MUST_BE_SPARSE_VECTOR_TYPE( T1  );
   pe_CONSTRAINT_MUST_BE_DENSE_VECTOR_TYPE ( MT1 );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( T1 , TF );
   pe_CONSTRAINT_MUST_BE_VECTOR_WITH_TRANSPOSE_FLAG( MT1, TF );

   if( (~lhs).size() != (~rhs).size() )
      throw std::invalid_argument( "Vector sizes do not match" );

   Lhs left( ~lhs );

   MT1 tmp;

   for( ConstIterator element=left.begin(); element!=left.end(); ++element )
      tmp[element->index()] = element->value() * rhs[element->index()];

   return tmp;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Division operator for the divison of a 2D vector by a scalar value
 *        (\f$ \vec{a}=\vec{b}/s \f$).
 * \ingroup dense_vector_2
 *
 * \param vec The left-hand side 2D vector for the division.
 * \param scalar The right-hand side scalar value for the division.
 * \return The scaled result vector.
 *
 * This operator represents the division of a 2D vector by a scalar value:

   \code
   pe::Vec2 a, b;
   // ... Initialization
   b = a / 0.24;
   \endcode

 * The operator returns a 2D vector of the higher-order element type of the involved data types
 * \a T1 and \a T2. Both data types \a T1 and \a T2 have to be supported by the MathTrait class
 * template. Note that this operator is only selected in case a 2D vector with either integral
 * or floating point data elements is divided by a scalar value of built-in data type.
 *
 * \b Note: A division by zero is only checked by an user assert.
 */
template< typename T1  // Data type of the left-hand side vector
        , typename T2  // Data type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline const Vector2< typename MathTrait< T1, T2, IsNumeric<T1>::value && IsNumeric<T2>::value >::DivType, TF >
   operator/( const Vector2<T1,TF>& vec, T2 scalar )
{
   pe_USER_ASSERT( scalar != T2(0), "Division by zero detected" );

   typedef typename MathTrait<T1,T2>::DivType  DT;

   // Depending on the two involved data types, an integer division is applied or a
   // floating point division is selected.
   if( IsNumeric<DT>::value && IsFloatingPoint<DT>::value ) {
      const DT tmp( DT(1)/static_cast<DT>( scalar ) );
      return Vector2<DT,TF>( vec[0]*tmp, vec[1]*tmp );
   }

   else {
      return Vector2<DT,TF>( vec[0]/scalar, vec[1]/scalar );
   }
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
struct MathTrait< Vector2<T1,TF>, T2 >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef INVALID_NUMERICAL_TYPE                              AddType;
   typedef INVALID_NUMERICAL_TYPE                              SubType;
   typedef Vector2< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef Vector2< typename MathTrait<T1,T2>::DivType , TF >  DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T2 );
};

template< typename T1, typename T2, bool TF >
struct MathTrait< T1, Vector2<T2,TF> >
{
   typedef INVALID_NUMERICAL_TYPE                              HighType;
   typedef INVALID_NUMERICAL_TYPE                              LowType;
   typedef INVALID_NUMERICAL_TYPE                              AddType;
   typedef INVALID_NUMERICAL_TYPE                              SubType;
   typedef Vector2< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
   pe_CONSTRAINT_MUST_BE_NUMERIC_TYPE( T1 );
};

template< typename T1, typename T2, bool TF >
struct MathTrait< Vector2<T1,TF>, Vector2<T2,TF> >
{
   typedef Vector2< typename MathTrait<T1,T2>::HighType, TF >  HighType;
   typedef Vector2< typename MathTrait<T1,T2>::LowType , TF >  LowType;
   typedef Vector2< typename MathTrait<T1,T2>::AddType , TF >  AddType;
   typedef Vector2< typename MathTrait<T1,T2>::SubType , TF >  SubType;
   typedef Vector2< typename MathTrait<T1,T2>::MultType, TF >  MultType;
   typedef INVALID_NUMERICAL_TYPE                              DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,true>, Vector2<T2,false> >
{
   typedef INVALID_NUMERICAL_TYPE               HighType;
   typedef INVALID_NUMERICAL_TYPE               LowType;
   typedef INVALID_NUMERICAL_TYPE               AddType;
   typedef INVALID_NUMERICAL_TYPE               SubType;
   typedef typename MathTrait<T1,T2>::MultType  MultType;
   typedef INVALID_NUMERICAL_TYPE               DivType;
};

template< typename T1, typename T2 >
struct MathTrait< Vector2<T1,false>, Vector2<T2,true> >
{
   typedef INVALID_NUMERICAL_TYPE                            HighType;
   typedef INVALID_NUMERICAL_TYPE                            LowType;
   typedef INVALID_NUMERICAL_TYPE                            AddType;
   typedef INVALID_NUMERICAL_TYPE                            SubType;
   typedef Matrix2x2< typename MathTrait<T1,T2>::MultType >  MultType;
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
struct TransposeTrait< Vector2<T,TF> >
{
   typedef Vector2<typename TransposeTrait<T>::Type,!TF>  Type;
};
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief 2-dimensional real vector.
 * \ingroup dense_vector_2
 */
typedef Vector2<real,false>  Vec2;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2-dimensional transpose real vector.
 * \ingroup dense_vector_2
 */
typedef Vector2<real,true>  Vec2T;
//*************************************************************************************************

} // namespace pe

#endif
