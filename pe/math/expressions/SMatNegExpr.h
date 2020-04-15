//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatNegExpr.h
 *  \brief Header file for the sparse matrix negation expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATNEGEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATNEGEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/expressions/SparseMatrix.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL UNARY ARITHMETIC OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Unary minus operator for the negation of a sparse matrix (\f$ A = -B \f$).
 * \ingroup sparse_matrix
 *
 * \param sm The sparse matrix to be negated.
 * \return The negation of the matrix.
 *
 * This operator represents the negation of a sparse matrix:

   \code
   pe::SMatN A, B;
   // ... Resizing and initialization
   B = -A;
   \endcode
 */
template< typename MT >  // Data type of the sparse matrix
inline const MT operator-( const SparseMatrix<MT>& sm )
{
   typedef typename MT::Iterator  Iterator;

   MT tmp( sm );

   for( size_t i=0; i<tmp.rows(); ++i ) {
      const Iterator endElem( tmp.end(i) );
      for( Iterator elem=tmp.begin(i); elem!=endElem; ++elem )
         elem->value() = -elem->value();
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
