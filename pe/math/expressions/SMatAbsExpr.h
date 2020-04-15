//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatAbsExpr.h
 *  \brief Header file for the sparse matrix absolute value expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATABSEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATABSEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a sparse matrix containing the absolute values of each single element of \a sm.
 * \ingroup sparse_matrix
 *
 * \param sm The integral sparse input matrix.
 * \return The absolute value of each single element of \a sm.
 *
 * The \a abs function calculates the absolute value of each element of the sparse input
 * matrix \a sm.
 */
template< typename MT >  // Type of the sparse matrix
inline const MT abs( const SparseMatrix<MT>& sm )
{
   using std::abs;

   typedef typename MT::Iterator  Iterator;

   MT tmp( sm );
   for( size_t i=0; i<tmp.rows(); ++i ) {
      for( Iterator element=tmp.begin(i); element!=tmp.end(i); ++element )
         *element = abs( *element );
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
