//=================================================================================================
/*!
 *  \file pe/math/SparseMatrix.h
 *  \brief Header file for the SparseMatrix CRTP base class
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

#ifndef _PE_MATH_SPARSEMATRIX_H_
#define _PE_MATH_SPARSEMATRIX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iomanip>
#include <ostream>
#include <pe/math/expressions/DenseMatrix.h>
#include <pe/math/expressions/DVecSMatMultExpr.h>
#include <pe/math/expressions/DVecTSVecMultExpr.h>
#include <pe/math/expressions/SMatAbsExpr.h>
#include <pe/math/expressions/SMatDVecMultExpr.h>
#include <pe/math/expressions/SMatFabsExpr.h>
#include <pe/math/expressions/SMatNegExpr.h>
#include <pe/math/expressions/SMatScalarDivExpr.h>
#include <pe/math/expressions/SMatScalarMultExpr.h>
#include <pe/math/expressions/SMatSMatAddExpr.h>
#include <pe/math/expressions/SMatSMatMultExpr.h>
#include <pe/math/expressions/SMatSMatSubExpr.h>
#include <pe/math/expressions/SMatSVecMultExpr.h>
#include <pe/math/expressions/SMatTransExpr.h>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/math/expressions/SVecSMatMultExpr.h>
#include <pe/math/expressions/SVecTDVecMultExpr.h>
#include <pe/math/expressions/SVecTSVecMultExpr.h>
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
/*!\name SparseMatrix global functions */
//@{
template< typename MT1, typename MT2 >
inline void assign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void assign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void addAssign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void addAssign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void subAssign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void subAssign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void multAssign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs );

template< typename MT1, typename MT2 >
inline void multAssign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a dense matrix to a sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side dense matrix to be assigned.
 * \return void
 *
 * This function implements the default assignment of a dense matrix to a sparse matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void assign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the assignment of a sparse matrix to a sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side sparse matrix to be assigned.
 * \return void
 *
 * This function implements the default assignment of a sparse matrix to a sparse matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void assign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).assign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a dense matrix to a sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side dense matrix to be added.
 * \return void
 *
 * This function implements the default addition assignment of a dense matrix to a sparse
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void addAssign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the addition assignment of a sparse matrix to a sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side sparse matrix to be added.
 * \return void
 *
 * This function implements the default addition assignment of a sparse matrix to a sparse
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void addAssign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).addAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a dense matrix to a sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side dense matrix to be subtracted.
 * \return void
 *
 * This function implements the default subtraction assignment of a dense matrix to a sparse
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void subAssign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).subAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the subtraction assignment of a sparse matrix to sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side sparse matrix to be added.
 * \return void
 *
 * This function implements the default subtraction assignment of a sparse matrix to a sparse
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void subAssign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).rows()    == (~rhs).rows()   , "Invalid number of rows"    );
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).columns(), "Invalid number of columns" );

   (~lhs).subAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a dense matrix to a sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side dense matrix to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a dense matrix to a sparse
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void multAssign( SparseMatrix<MT1>& lhs, const DenseMatrix<MT2>& rhs )
{
   pe_INTERNAL_ASSERT( (~lhs).columns() == (~rhs).rows(), "Invalid matrix sizes" );

   (~lhs).multAssign( rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the multiplication assignment of a sparse matrix to a sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param lhs The target left-hand side sparse matrix.
 * \param rhs The right-hand side sparse matrix to be multiplied.
 * \return void
 *
 * This function implements the default multiplication assignment of a sparse matrix to a sparse
 * matrix.\n
 * This function must \b NOT be called explicitly! It is used internally for the performance
 * optimized evaluation of expression templates. Calling this function explicitly might result
 * in erroneous results and/or in compilation errors. Instead of using this function use the
 * assignment operator.
 */
template< typename MT1, typename MT2 >
inline void multAssign( SparseMatrix<MT1>& lhs, const SparseMatrix<MT2>& rhs )
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
/*!\name SparseMatrixMxN operators */
//@{
template< typename T1, typename T2 >
inline bool operator==( const SparseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs );

template< typename T1, typename T2 >
inline bool operator!=( const SparseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs );

template< typename MT >
std::ostream& operator<<( std::ostream& os, const SparseMatrix<MT>& sm );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality operator for the comparison of two sparse matrices.
 * \ingroup sparse_matrix
 *
 * \param lhs The left-hand side sparse matrix for the comparison.
 * \param rhs The right-hand side sparse matrix for the comparison.
 * \return \a true if the two sparse matrices are equal, \a false if not.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side sparse matrix
inline bool operator==( const SparseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   // Early exit in case the matrix sizes don't match
   if( (~lhs).rows() != (~rhs).rows() || (~lhs).columns() != (~rhs).columns() )
      return false;

   typedef typename T1::ConstIterator  LhsConstIterator;
   typedef typename T2::ConstIterator  RhsConstIterator;
   typedef typename T1::ElementType    LET;
   typedef typename T2::ElementType    RET;

   for( size_t i=0; i<(~lhs).rows(); ++i )
   {
      const LhsConstIterator lend( (~lhs).end(i) );
      const RhsConstIterator rend( (~rhs).end(i) );

      LhsConstIterator lelem( (~lhs).begin(i) );
      RhsConstIterator relem( (~rhs).begin(i) );

      while( lelem < lend && relem < rend ) {
         if( lelem->index() < relem->index() && !isDefault( (lelem++)->value() ) )
            return false;
         else if( lelem->index() > relem->index() && !isDefault( (relem++)->value() ) )
            return false;
         else if( !equal( (lelem++)->value(), (relem++)->value() ) )
            return false;
      }

      while( lelem < lend ) {
         if( !isDefault( (lelem++)->value() ) )
            return false;
      }

      while( relem < rend ) {
         if( !isDefault( (relem++)->value() ) )
            return false;
      }
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality operator for the comparison of two sparse matrices.
 * \ingroup sparse_matrix
 *
 * \param lhs The left-hand side sparse matrix for the comparison.
 * \param rhs The right-hand side sparse matrix for the comparison.
 * \return \a true if the two sparse matrices are not equal, \a false if they are equal.
 */
template< typename T1    // Type of the left-hand side sparse matrix
        , typename T2 >  // Type of the right-hand side sparse matrix
inline bool operator!=( const SparseMatrix<T1>& lhs, const SparseMatrix<T2>& rhs )
{
   return !( lhs == rhs );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for sparse MxN matrices.
 * \ingroup sparse_matrix
 *
 * \param os Reference to the output stream.
 * \param sm Reference to a constant sparse matrix object.
 * \return Reference to the output stream.
 */
template< typename MT >  // Type of the sparse matrix
std::ostream& operator<<( std::ostream& os, const SparseMatrix<MT>& sm )
{
   for( size_t i=0; i<(~sm).rows(); ++i ) {
      for( size_t j=0; j<(~sm).columns(); ++j ) {
         os << std::setw(14) << (~sm)(i,j);
      }
      os << "\n";
   }

   return os;
}
//*************************************************************************************************

} // namespace pe

#endif
