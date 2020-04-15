//=================================================================================================
/*!
 *  \file pe/math/typetraits/IsMatrix.h
 *  \brief Header file for the IsMatrix type trait
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

#ifndef _PE_MATH_TYPETRAITS_ISMATRIX_H_
#define _PE_MATH_TYPETRAITS_ISMATRIX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/typetraits/IsDenseMatrix.h>
#include <pe/math/typetraits/IsSparseMatrix.h>
#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsMatrix type trait.
 * \ingroup math_type_traits
 */
template< typename T >
struct IsMatrixHelper
{
   //**********************************************************************************************
   enum { value = IsDenseMatrix<T>::value || IsSparseMatrix<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for matrix types.
 * \ingroup math_type_traits
 *
 * This type trait tests whether or not the given template parameter is a N-dimensional dense
 * or sparse matrix type. In case the type is a matrix type, the \a value member enumeration
 * is set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a yes is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsMatrix< MatrixMxN<double> >::value       // Evaluates to 1
   pe::IsMatrix< const Matrix3x3<float> >::Type   // Results in TrueType
   pe::IsMatrix< volatile SparseMatrixMxN<int> >  // Is derived from TrueType
   pe::IsMatrix< VectorN<double>::value           // Evaluates to 0
   pe::IsMatrix< const Vector3<float> >::Type     // Results in FalseType
   pe::IsMatrix< volatile SparseVectorN<int> >    // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsMatrix : public IsMatrixHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsMatrixHelper<T>::value };
   typedef typename IsMatrixHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
