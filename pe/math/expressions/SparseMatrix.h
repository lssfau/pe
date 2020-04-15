//=================================================================================================
/*!
 *  \file pe/math/expressions/SparseMatrix.h
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

#ifndef _PE_MATH_EXPRESSIONS_SPARSEMATRIX_H_
#define _PE_MATH_EXPRESSIONS_SPARSEMATRIX_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup sparse_matrix Sparse Matrices
 * \ingroup math
 */
/*!\defgroup sparse_matrix_expression Expressions
 * \ingroup sparse_matrix
 */
/*!\brief Base class for sparse matrices.
 * \ingroup sparse_matrix
 *
 * The SparseMatrix class is a base class for all sparse matrix classes. It provides an
 * abstraction from the actual type of the sparse matrix, but enables a conversion back
 * to this type via the 'Curiously Recurring Template Pattern' (CRTP).
 */
template< typename MT >  // Type of the sparse matrix
struct SparseMatrix
{
   //**Type definitions****************************************************************************
   typedef MT  MatrixType;  //!< Type of the sparse matrix.
   //**********************************************************************************************

   //**Non-const conversion operator***************************************************************
   /*!\brief Conversion operator for non-constant sparse matrices.
   //
   // \return Reference of the actual type of the sparse matrix.
   */
   inline MatrixType& operator~() {
      return *static_cast<MatrixType*>( this );
   }
   //**********************************************************************************************

   //**Const conversion operators******************************************************************
   /*!\brief Conversion operator for constant sparse matrices.
   */
   inline const MatrixType& operator~() const {
      return *static_cast<const MatrixType*>( this );
   }
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
