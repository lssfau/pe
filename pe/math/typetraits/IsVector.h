//=================================================================================================
/*!
 *  \file pe/math/typetraits/IsVector.h
 *  \brief Header file for the IsVector type trait
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

#ifndef _PE_MATH_TYPETRAITS_ISVECTOR_H_
#define _PE_MATH_TYPETRAITS_ISVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/typetraits/IsDenseVector.h>
#include <pe/math/typetraits/IsSparseVector.h>
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
/*!\brief Auxiliary helper struct for the IsVector type trait.
 * \ingroup math_type_traits
 */
template< typename T >
struct IsVectorHelper
{
   //**********************************************************************************************
   enum { value = IsDenseVector<T>::value || IsSparseVector<T>::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for vector types.
 * \ingroup math_type_traits
 *
 * This type trait tests whether or not the given template parameter is a N-dimensional dense
 * or sparse vector type. In case the type is a vector type, the \a value member enumeration
 * is set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsVector< VectorN<double> >::value         // Evaluates to 1
   pe::IsVector< const Vector3<float> >::Type     // Results in TrueType
   pe::IsVector< volatile SparseVectorN<int> >    // Is derived from TrueType
   pe::IsVector< MatrixMxN<double> >::value       // Evaluates to 0
   pe::IsVector< const Matrix3x3<double> >::Type  // Results in FalseType
   pe::IsVector< volatile SparseMatrixMxN<int> >  // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsVector : public IsVectorHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsVectorHelper<T>::value };
   typedef typename IsVectorHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
