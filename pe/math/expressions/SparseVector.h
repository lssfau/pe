//=================================================================================================
/*!
 *  \file pe/math/expressions/SparseVector.h
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

#ifndef _PE_MATH_EXPRESSIONS_SPARSEVECTOR_H_
#define _PE_MATH_EXPRESSIONS_SPARSEVECTOR_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup sparse_vector Sparse Vectors
 * \ingroup math
 */
/*!\defgroup sparse_vector_expression Expressions
 * \ingroup sparse_vector
 */
/*!\brief Base class for sparse vectors.
 * \ingroup sparse_vector
 *
 * The SparseVector class is a base class for all arbitrarily sized (N-dimensional) sparse
 * vectors. It provides an abstraction from the actual type of the sparse vector, but enables
 * a conversion back to this type via the 'Curiously Recurring Template Pattern' (CRTP).
 */
template< typename VT  // Type of the sparse vector
        , bool TF >    // Transposition flag
struct SparseVector
{
   //**Type definitions****************************************************************************
   typedef VT  VectorType;  //!< Type of the sparse vector.
   //**********************************************************************************************

   //**Non-const conversion operator***************************************************************
   /*!\brief Conversion operator for non-constant sparse vectors.
   //
   // \return Reference of the actual type of the sparse vector.
   */
   inline VectorType& operator~() {
      return *static_cast<VectorType*>( this );
   }
   //**********************************************************************************************

   //**Const conversion operators******************************************************************
   /*!\brief Conversion operator for constant sparse vectors.
   */
   inline const VectorType& operator~() const {
      return *static_cast<const VectorType*>( this );
   }
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
