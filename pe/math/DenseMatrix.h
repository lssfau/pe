//=================================================================================================
/*!
 *  \file pe/math/DenseMatrix.h
 *  \brief Header file for the DenseMatrix CRTP base class
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

#ifndef _PE_MATH_DENSEMATRIX_H_
#define _PE_MATH_DENSEMATRIX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iomanip>
#include <ostream>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/expressions/DMatAbsExpr.h>
#include <pe/math/expressions/DMatDMatAddExpr.h>
#include <pe/math/expressions/DMatDMatMultExpr.h>
#include <pe/math/expressions/DMatDMatSubExpr.h>
#include <pe/math/expressions/DMatDVecMultExpr.h>
#include <pe/math/expressions/DMatFabsExpr.h>
#include <pe/math/expressions/DMatNegExpr.h>
#include <pe/math/expressions/DMatScalarDivExpr.h>
#include <pe/math/expressions/DMatScalarMultExpr.h>
#include <pe/math/expressions/DMatSMatAddExpr.h>
#include <pe/math/expressions/DMatSMatMultExpr.h>
#include <pe/math/expressions/DMatSMatSubExpr.h>
#include <pe/math/expressions/DMatSVecMultExpr.h>
#include <pe/math/expressions/DMatTransExpr.h>
#include <pe/math/expressions/DVecDMatMultExpr.h>
#include <pe/math/expressions/DVecTDVecMultExpr.h>
#include <pe/math/expressions/SMatDMatMultExpr.h>
#include <pe/math/expressions/SMatDMatSubExpr.h>
#include <pe/math/expressions/SVecDMatMultExpr.h>
#include <pe/math/expressions/SparseMatrix.h>
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
/*!\name DenseMatrix global functions */
//@{
template< typename MT1, typename MT2 >
inline void assign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void assign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void addAssign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void addAssign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void subAssign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void subAssign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void multAssign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void multAssign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a dense matrix to a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side dense matrix to be assigned.
 * \return void
 *
 * This function implements the default assignment of a dense matrix to a dense matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side dense matrix
inline void assign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a sparse matrix to a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side sparse matrix to be assigned.
 * \return void
 *
 * This function implements the default assignment of a sparse matrix to a dense matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side sparse matrix
inline void assign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a dense matrix to a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side dense matrix to be added.
 * \return void
 *
 * This function implements the default addition assignment of a dense matrix to a dense
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side dense matrix
inline void addAssign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a sparse matrix to a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side sparse matrix to be added.
 * \return void
 *
 * This function implements the default addition assignment of a sparse matrix to a dense
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side sparse matrix
inline void addAssign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a dense matrix to dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side dense matrix to be subtracted.
 * \return void
 *
 * This function implements the default subtraction assignment of a dense matrix to a dense
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side dense matrix
inline void subAssign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).subAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a sparse matrix to a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side sparse matrix to be assigned.
 * \return void
 *
 * This function implements the default subtraction assignment of a sparse matrix to a dense
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side dense matrix
inline void subAssign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).subAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a dense matrix to a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side dense matrix to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a dense matrix to a dense
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side dense matrix
inline void multAssign( DenseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).rows(), "Invalid matrix sizes" );

   (~lhs).multAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a sparse matrix to a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The target left-hand side dense matrix.
 * \param rhs The right-hand side sparse matrix to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a sparse matrix to a dense
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1    // Type of the left-hand side dense matrix
        , typename MT2 >  // Type of the right-hand side sparse matrix
inline void multAssign( DenseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).rows(), "Invalid matrix sizes" );

   (~lhs).multAssign( rhs );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name DenseMatrix operators */
//@{
template< typename T1, typename T2 >
inline bool operator==( const DenseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline bool operator==( const DenseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline bool operator==( const SparseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const DenseMatrix<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( T1 scalar, const DenseMatrix<T2>& mat );

template< typename T1, typename T2 >
inline bool operator!=( const DenseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline bool operator!=( const DenseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline bool operator!=( const SparseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const DenseMatrix<T1>& mat, T2 scalar );

template< typename T1, typename T2 >
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( T1 scalar, const DenseMatrix<T2>& mat );

template< typename MT >
inline std::ostream& operator<<( std::ostream& os, const DenseMatrix<MT>& dm );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two dense matrices.
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side matrix for the comparison.
 * \param rhs The right-hand side matrix for the comparison.
 * \return \a true if the two matrices are equal, \a false if not.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side dense matrix
inline bool operator==( const DenseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs )
{
   // Early exit in case the matrix sizes don't match
   if( (~lhs).rows() != (~rhs).rows() || (~lhs).columns() != (~rhs).columns() )
      return false;

   // In order to compare the matrix and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type within the equal function.
   const size_t sqrsize( (~lhs).rows()*(~lhs).columns() );
   for( size_t i=0; i<sqrsize; ++i )
      if( !equal( (~lhs)[i], (~rhs)[i] ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a dense matrix and a sparse matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side dense matrix for the comparison.
 * \param rhs The right-hand side sparse matrix for the comparison.
 * \return \a true if the two matrices are equal, \a false if not.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side sparse matrix
inline bool operator==( const DenseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   // Early exit in case the matrix sizes don't match
   if( (~lhs).rows() != (~rhs).rows() || (~lhs).columns() != (~rhs).columns() )
      return false;

   typedef typename T2::ConstIterator  ConstIterator;

   size_t j( 0 );

   // In order to compare the two vectors, the data values of the lower-order data
   // type are converted to the higher-order data type within the equal function.
   for( size_t i=0; i<(~rhs).rows(); ++i ) {
      j = 0;
      for( ConstIterator element=(~rhs).begin(i); element!=(~rhs).end(i); ++element, ++j ) {
         for( ; j<element->index(); ++j ) {
            if( !isDefault( (~lhs)(i,j) ) ) return false;
         }
         if( !equal( element->value(), (~lhs)(i,j) ) ) return false;
      }
      for( ; j<(~lhs).columns(); ++j ) {
         if( !isDefault( (~lhs)(i,j) ) ) return false;
      }
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a sparse matrix and a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side sparse matrix for the comparison.
 * \param rhs The right-hand side dense matrix for the comparison.
 * \return \a true if the two matrices are equal, \a false if not.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side dense matrix
inline bool operator==( const SparseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs )
{
   return ( rhs == lhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a dense matrix and a scalar value.
 * \ingroup dense_matrix
 *
 * \param mat The left-hand side dense matrix for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if all elements of the matrix are equal to the scalar, \a false if not.
 *
 * If all values of the matrix are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side scalar
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator==( const DenseMatrix<T1>& mat, T2 scalar )
{
   // In order to compare the matrix and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type within the equal function.
   const size_t sqrsize( (~mat).rows()*(~mat).columns() );
   for( size_t i=0; i<sqrsize; ++i )
      if( !equal( (~mat)[i], scalar ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of a scalar value and a dense matrix.
 * \ingroup dense_matrix
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param mat The right-hand side dense matrix for the comparison.
 * \return \a true if all elements of the matrix are equal to the scalar, \a false if not.
 *
 * If all values of the matrix are equal to the scalar value, the equality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1    // Type of the left-hand side scalar
        , typename T2 >  // Type of the right-hand side dense matrix
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator==( T1 scalar, const DenseMatrix<T2>& mat )
{
   // In order to compare the matrix and the scalar value, the data values of the lower-order
   // data type are converted to the higher-order data type within the equal function.
   const size_t sqrsize( (~mat).rows()*(~mat).columns() );
   for( size_t i=0; i<sqrsize; ++i )
      if( !equal( (~mat)[i], scalar ) ) return false;
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two dense matrices.
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side dense matrix for the comparison.
 * \param rhs The right-hand side dense matrix for the comparison.
 * \return \a true if the two matrices are not equal, \a false if they are equal.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side dense matrix
inline bool operator!=( const DenseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a dense matrix and a sparse matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side dense matrix for the comparison.
 * \param rhs The right-hand side sparse matrix for the comparison.
 * \return \a true if the two matrices are not equal, \a false if they are equal.
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side sparse matrix
inline bool operator!=( const DenseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a sparse matrix and a dense matrix.
 * \ingroup dense_matrix
 *
 * \param lhs The left-hand side sparse matrix for the comparison.
 * \param rhs The right-hand side dense matrix for the comparison.
 * \return \a true if the two matrices are not equal, \a false if they are equal.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side dense matrix
inline bool operator!=( const SparseMatrix<T1>& lhs, const DenseMatrix<T2>& rhs )
{
   return !( rhs == lhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a dense matrix and a scalar value.
 * \ingroup dense_matrix
 *
 * \param mat The left-hand side dense matrix for the comparison.
 * \param scalar The right-hand side scalar value for the comparison.
 * \return \a true if at least one element of the matrix is different from the scalar, \a false if not.
 *
 * If one value of the matrix is inequal to the scalar value, the inequality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1    // Type of the left-hand side dense matrix
        , typename T2 >  // Type of the right-hand side scalar
inline typename EnableIf< IsNumeric<T2>, bool >::Type
   operator!=( const DenseMatrix<T1>& mat, T2 scalar )
{
   return !( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of a scalar value and a dense matrix.
 * \ingroup dense_matrix
 *
 * \param scalar The left-hand side scalar value for the comparison.
 * \param mat The right-hand side dense matrix for the comparison.
 * \return \a true if at least one element of the matrix is different from the scalar, \a false if not.
 *
 * If one value of the matrix is inequal to the scalar value, the inequality test returns \a true,
 * otherwise \a false. Note that this function can only be used with built-in, numerical data
 * types!
 */
template< typename T1    // Type of the left-hand side scalar
        , typename T2 >  // Type of the right-hand side dense matrix
inline typename EnableIf< IsNumeric<T1>, bool >::Type
   operator!=( T1 scalar, const DenseMatrix<T2>& mat )
{
   return !( mat == scalar );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for dense MxN matrices.
 * \ingroup dense_matrix
 *
 * \param os Reference to the output stream.
 * \param dm Reference to a constant dense matrix object.
 * \return Reference to the output stream.
 */
template< typename MT >  // Type of the dense matrix
inline std::ostream& operator<<( std::ostream& os, const DenseMatrix<MT>& dm )
{
   for( size_t i=0; i<(~dm).rows(); ++i ) {
      for( size_t j=0; j<(~dm).columns(); ++j ) {
         os << std::setw(14) << (~dm)(i,j);
      }
      os << "\n";
   }

   return os;
}
//*************************************************************************************************

} // namespace pe

#endif
