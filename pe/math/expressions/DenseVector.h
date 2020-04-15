//=================================================================================================
/*!
 *  \file pe/math/expressions/DenseVector.h
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

#ifndef _PE_MATH_EXPRESSIONS_DENSEVECTOR_H_
#define _PE_MATH_EXPRESSIONS_DENSEVECTOR_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup dense_vector Dense Vectors
 * \ingroup math
 */
/*!\defgroup dense_vector_expression Expressions
 * \ingroup dense_vector
 */
/*!\brief Base class for N-dimensional dense vectors.
 * \ingroup dense_vector
 *
 * The DenseVector class is a base class for all arbitrarily sized (N-dimensional) dense
 * vectors. It provides an abstraction from the actual type of the dense vector, but enables
 * a conversion back to this type via the 'Curiously Recurring Template Pattern' (CRTP).
 */
template< typename VT  // Type of the dense vector
        , bool TF >    // Transposition flag
struct DenseVector
{
   //**Type definitions****************************************************************************
   typedef VT  VectorType;  //!< Type of the dense vector.
   //**********************************************************************************************

   //**Non-const conversion operator***************************************************************
   /*!\brief Conversion operator for non-constant dense vectors.
   //
   // \return Reference of the actual type of the dense vector.
   */
   inline VectorType& operator~() {
      return *static_cast<VectorType*>( this );
   }
   //**********************************************************************************************

   //**Const conversion operators******************************************************************
   /*!\brief Conversion operator for constant dense vectors.
   //
   // \return Const reference of the actual type of the dense vector.
   */
   inline const VectorType& operator~() const {
      return *static_cast<const VectorType*>( this );
   }
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
