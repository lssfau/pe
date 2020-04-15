//=================================================================================================
/*!
 *  \file pe/math/DenseVector.h
 *  \brief Header file for the DenseVector CRTP base class
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

#ifndef _PE_MATH_DENSEVECTOR_H_
#define _PE_MATH_DENSEVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <ostream>
#include <pe/math/constraints/DenseVector.h>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/expressions/DVecAbsExpr.h>
#include <pe/math/expressions/DVecDVecAddExpr.h>
#include <pe/math/expressions/DVecDVecDivExpr.h>
#include <pe/math/expressions/DVecDVecMultExpr.h>
#include <pe/math/expressions/DVecDVecSubExpr.h>
#include <pe/math/expressions/DVecFabsExpr.h>
#include <pe/math/expressions/DVecNegExpr.h>
#include <pe/math/expressions/DVecScalarDivExpr.h>
#include <pe/math/expressions/DVecScalarMultExpr.h>
#include <pe/math/expressions/DVecSVecAddExpr.h>
#include <pe/math/expressions/DVecSVecSubExpr.h>
#include <pe/math/expressions/DVecTransExpr.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/expressions/SVecDVecSubExpr.h>
#include <pe/math/expressions/TDVecDVecMultExpr.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/util/Assert.h>
#include <pe/util/EnableIf.h>
#include <pe/util/Types.h>
#include <pe/util/typetraits/IsNumeric.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name DenseVector global functions */
//@{
template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void assign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void assign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void addAssign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void addAssign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void subAssign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void subAssign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void multAssign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void multAssign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a dense vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side dense vector to be assigned.
 * \return void
 *
 * This function implements the default assignment of a dense vector to a dense vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void assign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a sparse vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side sparse vector to be assigned.
 * \return void
 *
 * This function implements the default assignment of a sparse vector to a dense vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void assign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a dense vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side dense vector to be added.
 * \return void
 *
 * This function implements the default addition assignment of a dense vector to a dense
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void addAssign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a sparse vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side sparse vector to be added.
 * \return void
 *
 * This function implements the default addition assignment of a sparse vector to a dense
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void addAssign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a dense vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side dense vector to be subtracted.
 * \return void
 *
 * This function implements the default subtraction assignment of a dense vector to a dense
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void subAssign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).subAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a sparse vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side sparse vector to be subtracted.
 * \return void
 *
 * This function implements the default subtraction assignment of a sparse vector to a dense
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void subAssign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).subAssign( rhs );
}
//*************************************************************************************************

// 
//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a dense vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side dense vector to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a dense vector to a dense
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void multAssign( DenseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).multAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a sparse vector to a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The target left-hand side dense vector.
 * \param rhs The right-hand side sparse vector to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a sparse vector to a dense
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side dense vector
        , bool TF1      // Transposition flag of the left-hand side dense vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void multAssign( DenseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).multAssign( rhs );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name DenseVector operators */
//@{
template< typename T1, typename T2, bool TF >
inline bool operator==( const DenseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline bool operator==( const DenseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline bool operator==( const SparseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const DenseVector<T1,TF>& vec, T2 scalar );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const DenseVector<T2,TF>& vec );

template< typename T1, typename T2, bool TF >
inline bool operator!=( const DenseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline bool operator!=( const DenseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline bool operator!=( const SparseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const DenseVector<T1,TF>& vec, T2 scalar );

template< typename T1, typename T2, bool TF >
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const DenseVector<T2,TF>& vec );

template< typename VT, bool TF >
inline std::ostream& operator<<( std::ostream& os, const DenseVector<VT,TF>& dv );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two dense vectors.
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side dense vector for the comparison.
 * \param rhs The right-hand side dense vector for the comparison.
 * \return \a true if the two vectors are equal, \a false if not.
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline bool operator==( const DenseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs )
{
   // Early exit in case the vector sizes don't match
   if( (~lhs).size() != (~rhs).size() ) return false;

   // In order to compare the two vectors, the data values of the lower-order data
   // type are converted to the higher-order data type within the equal function.
   for( size_t i=0; i<(~lhs).size(); ++i )
      if( !equal( (~lhs)[i], (~rhs)[i] ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a dense vector and a sparse vector.
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side dense vector for the comparison.
 * \param rhs The right-hand side sparse vector for the comparison.
 * \return \a true if the two vectors are equal, \a false if not.
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline bool operator==( const DenseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs )
{
   // Early exit in case the vector sizes don't match
   if( (~lhs).size() != (~rhs).size() ) return false;

   typedef typename T2::ConstIterator  ConstIterator;

   size_t i( 0 );

   // In order to compare the two vectors, the data values of the lower-order data
   // type are converted to the higher-order data type within the equal function.
   for( ConstIterator element=(~rhs).begin(); element!=(~rhs).end(); ++element, ++i ) {
      for( ; i<element->index(); ++i ) {
         if( !isDefault( (~lhs)[i] ) ) return false;
      }
      if( !equal( element->value(), (~lhs)[i] ) ) return false;
   }
   for( ; i<(~lhs).size(); ++i ) {
      if( !isDefault( (~lhs)[i] ) ) return false;
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a sparse vector and a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side sparse vector for the comparison.
 * \param rhs The right-hand side dense vector for the comparison.
 * \return \a true if the two vectors are equal, \a false if not.
 */
template< typename T1  // Type of the left-hand side sparse vector
        , typename T2  // Type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline bool operator==( const SparseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs )
{
   return ( rhs == lhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a dense vector and a scalar value.
 * \ingroup dense_vector
 *
 * \param vec The left-hand side dense vector for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if all elements of the vector are equal to the scalar, \a false if not.
 *
 * If all values of the vector are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const DenseVector<T1,TF>& vec, T2 scalar )
{
   // In order to compare the vector and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type within the equal function.
   for( size_t i=0; i<(~vec).size(); ++i )
      if( !equal( (~vec)[i], scalar ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a scalar value and a dense vector.
 * \ingroup dense_vector
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param vec The right-hand side dense vector for the comparison.
 * \return \a true if all elements of the vector are equal to the scalar, \a false if not.
 *
 * If all values of the vector are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1  // Type of the left-hand side scalar
        , typename T2  // Type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const DenseVector<T2,TF>& vec )
{
   // In order to compare the vector and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type within the equal function.
   for( size_t i=0; i<(~vec).size(); ++i )
      if( !equal( (~vec)[i], scalar ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two dense vectors.
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side dense vector for the comparison.
 * \param rhs The right-hand side dense vector for the comparison.
 * \return \a true if the two vectors are not equal, \a false if they are equal.
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline bool operator!=( const DenseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a dense vector and a sparse vector.
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side dense vector for the comparison.
 * \param rhs The right-hand side sparse vector for the comparison.
 * \return \a true if the two vectors are not equal, \a false if they are equal.
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline bool operator!=( const DenseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a sparse vector and a dense vector.
 * \ingroup dense_vector
 *
 * \param lhs The left-hand side sparse vector for the comparison.
 * \param rhs The right-hand side dense vector for the comparison.
 * \return \a true if the two vectors are not equal, \a false if they are equal.
 */
template< typename T1  // Type of the left-hand side sparse vector
        , typename T2  // Type of the right-hand side dense vector
        , bool TF >    // Transposition flag
inline bool operator!=( const SparseVector<T1,TF>& lhs, const DenseVector<T2,TF>& rhs )
{
   return !( rhs == lhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a dense vector and a scalar value.
 * \ingroup dense_vector
 *
 * \param vec The left-hand side dense vector for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if at least one element of the vector is different from the scalar, \a false if not.
 *
 * If one value of the vector is inequal to the scalar value, the inequality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1  // Type of the left-hand side dense vector
        , typename T2  // Type of the right-hand side scalar
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const DenseVector<T1,TF>& vec, T2 scalar )
{
   return !( vec == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a scalar value and a dense vector.
 * \ingroup dense_vector
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param vec The right-hand side dense vector for the comparison.
 * \return \a true if at least one element of the vector is different from the scalar, \a false if not.
 *
 * If one value of the vector is inequal to the scalar value, the inequality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1  // Type of the left-hand side scalar
        , typename T2  // Type of the right-hand side vector
        , bool TF >    // Transposition flag
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const DenseVector<T2,TF>& vec )
{
   return !( vec == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for arbitrary sized dense vectors.
 * \ingroup dense_vector
 *
 * \param os Reference to the output stream.
 * \param dv Reference to a constant dense vector object.
 * \return Reference to the output stream.
 */
template< typename VT  // Type of the dense vector
        , bool TF >    // Transposition flag
inline std::ostream& operator<<( std::ostream& os, const DenseVector<VT,TF>& dv )
{
   for( size_t i=0; i<(~dv).size(); ++i )
      os << (~dv)[i] << "\n";
   return os;
}
//*************************************************************************************************

} // namespace pe

#endif
