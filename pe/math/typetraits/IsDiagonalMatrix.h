//=================================================================================================
/*!
 *  \file pe/math/typetraits/IsDiagonalMatrix.h
 *  \brief Header file for the IsDiagonalMatrix type trait
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

#ifndef _PE_MATH_TYPETRAITS_ISDIAGONALMATRIX_H_
#define _PE_MATH_TYPETRAITS_ISDIAGONALMATRIX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_base_of.hpp>
#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename > class DiagonalMatrix;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsDiagonalMatrix type trait.
 * \ingroup math_type_traits
 */
template< typename T >
struct IsDiagonalMatrixHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of< DiagonalMatrix<T>, T >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for diagonal matrix types.
 * \ingroup math_type_traits
 *
 * This type trait tests whether or not the given template parameter is a diagonal, N-dimensional
 * matrix type. In case the type is a diagonal matrix type, the \a value member enumeration is
 * set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsDiagonalMatrix< DiagonalMatrixN<double> >::value      // Evaluates to 1
   pe::IsDiagonalMatrix< const DiagonalMatrixN<float> >::Type  // Results in TrueType
   pe::IsDiagonalMatrix< volatile DiagonalMatrixN<int> >       // Is derived from TrueType
   pe::IsDiagonalMatrix< VectorN<double> >::value              // Evaluates to 0
   pe::IsDiagonalMatrix< const MatrixMxN<double> >::Type       // Results in FalseType
   pe::IsDiagonalMatrix< SparseVectorN<double> >               // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsDiagonalMatrix : public IsDiagonalMatrixHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsDiagonalMatrixHelper<T>::value };
   typedef typename IsDiagonalMatrixHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
