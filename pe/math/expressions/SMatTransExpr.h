//=================================================================================================
/*!
 *  \file pe/math/expressions/SMatTransExpr.h
 *  \brief Header file for the sparse matrix transpose expression
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

#ifndef _PE_MATH_EXPRESSIONS_SMATTRANSEXPR_H_
#define _PE_MATH_EXPRESSIONS_SMATTRANSEXPR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <pe/math/expressions/SparseMatrix.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculation of the transpose of the given sparse matrix.
 * \ingroup sparse_matrix
 *
 * \param sm The sparse matrix to be transposed.
 * \return The transpose of the matrix.
 *
 * This function returns the transpose of the given sparse matrix.
 */
template< typename MT >  // Type of the sparse matrix
inline const typename MT::ResultType trans( const SparseMatrix<MT>& sm )
{
   typedef typename MT::ConstIterator  ConstIterator;
   typedef typename MT::ResultType     ResultType;

   // Counting the number of elements per column
   std::vector<size_t> columnLengths( (~sm).columns(), 0 );
   for( size_t i=0; i<(~sm).rows(); ++i ) {
      const ConstIterator end( (~sm).end(i) );
      for( ConstIterator element=(~sm).begin(i); element!=end; ++element )
         ++columnLengths[element->index()];
   }

   // Setup of the transpose sparse matrix
   ResultType tmp( (~sm).columns(), (~sm).rows(), columnLengths );

   // Appending the elements to the rows of the transpose sparse matrix
   for( size_t i=0; i<(~sm).rows(); ++i ) {
      const ConstIterator end( (~sm).end(i) );
      for( ConstIterator element=(~sm).begin(i); element!=end; ++element )
         tmp.append( element->index(), i, element->value() );
   }

   return tmp;
}
//*************************************************************************************************

} // namespace pe

#endif
