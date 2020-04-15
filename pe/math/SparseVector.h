//=================================================================================================
/*!
 *  \file pe/math/SparseVector.h
 *  \brief Header file for the SparseVector CRTP base class
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

#ifndef _PE_MATH_SPARSEVECTOR_H_
#define _PE_MATH_SPARSEVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <ostream>
#include <pe/math/expressions/DenseVector.h>
#include <pe/math/expressions/DVecSVecMultExpr.h>
#include <pe/math/expressions/SparseVector.h>
#include <pe/math/expressions/SVecAbsExpr.h>
#include <pe/math/expressions/SVecDVecMultExpr.h>
#include <pe/math/expressions/SVecFabsExpr.h>
#include <pe/math/expressions/SVecNegExpr.h>
#include <pe/math/expressions/SVecScalarDivExpr.h>
#include <pe/math/expressions/SVecScalarMultExpr.h>
#include <pe/math/expressions/SVecSVecAddExpr.h>
#include <pe/math/expressions/SVecSVecMultExpr.h>
#include <pe/math/expressions/SVecSVecSubExpr.h>
#include <pe/math/expressions/SVecTransExpr.h>
#include <pe/math/expressions/TDVecSVecMultExpr.h>
#include <pe/math/expressions/TSVecSVecMultExpr.h>
#include <pe/math/expressions/TSVecDVecMultExpr.h>
#include <pe/math/shims/Equal.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name SparseVector global functions */
//@{
template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void assign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void assign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void addAssign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void addAssign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void subAssign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void subAssign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void multAssign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs );

template< typename VT1, bool TF1, typename VT2, bool TF2 >
inline void multAssign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a dense vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side dense vector to be assigned.
 * \return void
 *
 * This function implements the default assignment of a dense vector to a sparse vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void assign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a sparse vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side sparse vector to be assigned.
 * \return void
 *
 * This function implements the default assignment of a sparse vector to a sparse vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void assign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a dense vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side dense vector to be added.
 * \return void
 *
 * This function implements the default addition assignment of a dense vector to a sparse
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void addAssign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a sparse vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side sparse vector to be added.
 * \return void
 *
 * This function implements the default addition assignment of a sparse vector to a sparse
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void addAssign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a dense vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side dense vector to be subtracted.
 * \return void
 *
 * This function implements the default subtraction assignment of a dense vector to a sparse
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void subAssign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).subAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a sparse vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side sparse vector to be subtracted.
 * \return void
 *
 * This function implements the default subtraction assignment of a sparse vector to a sparse
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void subAssign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).subAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a dense vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side dense vector to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a dense vector to a sparse
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side dense vector
        , bool TF2 >    // Transposition flag of the right-hand side dense vector
inline void multAssign( SparseVector<VT1,TF1>& lhs, const DenseVector<VT2,TF2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).size() == (~rhs).size(), "Invalid vector sizes" );
   (~lhs).multAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a sparse vector to a sparse vector.
 * \ingroup sparse_vector
 *
 * \param lhs The target left-hand side sparse vector.
 * \param rhs The right-hand side sparse vector to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a sparse vector to a sparse
 * vector.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename VT1  // Type of the left-hand side sparse vector
        , bool TF1      // Transposition flag of the left-hand side sparse vector
        , typename VT2  // Type of the right-hand side sparse vector
        , bool TF2 >    // Transposition flag of the right-hand side sparse vector
inline void multAssign( SparseVector<VT1,TF1>& lhs, const SparseVector<VT2,TF2>& rhs )
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
/*!\name SparseVector operators */
//@{
template< typename T1, typename T2, bool TF >
inline bool operator==( const SparseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs );

template< typename T1, typename T2, bool TF >
inline bool operator!=( const SparseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs );

template< typename VT, bool TF >
inline std::ostream& operator<<( std::ostream& os, const SparseVector<VT,TF>& sv );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two sparse vectors.
 * \ingroup sparse_vector
 *
 * \param lhs The left-hand side sparse vector for the comparison.
 * \param rhs The right-hand side sparse vector for the comparison.
 * \return \a true if the two sparse vectors are equal, \a false if not.
 */
template< typename T1  // Type of the left-hand side sparse vector
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline bool operator==( const SparseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs )
{
   // Early exit in case the vector sizes don't match
   if( (~lhs).size() != (~rhs).size() ) return false;

   typedef typename T1::ElementType    LET;
   typedef typename T2::ElementType    RET;
   typedef typename T1::ConstIterator  LhsConstIterator;
   typedef typename T2::ConstIterator  RhsConstIterator;

   const LhsConstIterator lend( (~lhs).end() );
   const RhsConstIterator rend( (~rhs).end() );

   LhsConstIterator lelem( (~lhs).begin() );
   RhsConstIterator relem( (~rhs).begin() );

   while( lelem < lend && relem < rend )
   {
      if( isDefault( lelem->value() ) ) { ++lelem; continue; }
      if( isDefault( relem->value() ) ) { ++relem; continue; }

      if( lelem->index() != relem->index() || !equal( lelem->value(), relem->value() ) ) {
         return false;
      }
      else {
         ++lelem;
         ++relem;
      }
   }

   while( lelem < lend ) {
      if( !isDefault( lelem->value() ) )
         return false;
      ++lelem;
   }

   while( relem < rend ) {
      if( !isDefault( relem->value() ) )
         return false;
      ++relem;
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two sparse vectors.
 * \ingroup sparse_vector
 *
 * \param lhs The left-hand side sparse vector for the comparison.
 * \param rhs The right-hand side sparse vector for the comparison.
 * \return \a true if the two vectors are not equal, \a false if they are equal.
 */
template< typename T1  // Type of the left-hand side sparse vector
        , typename T2  // Type of the right-hand side sparse vector
        , bool TF >    // Transposition flag
inline bool operator!=( const SparseVector<T1,TF>& lhs, const SparseVector<T2,TF>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for sparse vectors.
 * \ingroup sparse_vector
 *
 * \param os Reference to the output stream.
 * \param sv Reference to a constant sparse vector object.
 * \return Reference to the output stream.
 */
template< typename VT  // Type of the sparse vector
        , bool TF >    // Transposition flag
inline std::ostream& operator<<( std::ostream& os, const SparseVector<VT,TF>& sv )
{
   for( size_t i=0; i<(~sv).size(); ++i )
      os << (~sv)[i] << "\n";
   return os;
}
//*************************************************************************************************

} // namespace pe

#endif
