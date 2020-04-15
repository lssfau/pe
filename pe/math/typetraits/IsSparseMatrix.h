//=================================================================================================
/*!
 *  \file pe/math/typetraits/IsSparseMatrix.h
 *  \brief Header file for the IsSparseMatrix type trait
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

#ifndef _PE_MATH_TYPETRAITS_ISSPARSEMATRIX_H_
#define _PE_MATH_TYPETRAITS_ISSPARSEMATRIX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/remove_cv.hpp>
#include <pe/util/FalseType.h>
#include <pe/util/SelectType.h>
#include <pe/util/TrueType.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename > struct SparseMatrix;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsSparseMatrix type trait.
 * \ingroup math_type_traits
 */
template< typename T >
struct IsSparseMatrixHelper
{
private:
   //**********************************************************************************************
   typedef typename boost::remove_cv<T>::type  T2;
   //**********************************************************************************************

public:
   //**********************************************************************************************
   enum { value = boost::is_base_of< SparseMatrix<T2>, T2 >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for sparse matrix types.
 * \ingroup math_type_traits
 *
 * This type trait tests whether or not the given template parameter is a sparse, N-dimensional
 * matrix type. In case the type is a sparse matrix type, the \a value member enumeration is
 * set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsSparseMatrix< SparseMatrixMxN<double> >::value      // Evaluates to 1
   pe::IsSparseMatrix< const SparseMatrixMxN<float> >::Type  // Results in TrueType
   pe::IsSparseMatrix< volatile SparseMatrixMxN<int> >       // Is derived from TrueType
   pe::IsSparseMatrix< VectorN<double> >::value              // Evaluates to 0
   pe::IsSparseMatrix< const MatrixMxN<double> >::Type       // Results in FalseType
   pe::IsSparseMatrix< SparseVectorN<double> >               // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsSparseMatrix : public IsSparseMatrixHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsSparseMatrixHelper<T>::value };
   typedef typename IsSparseMatrixHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
