//=================================================================================================
/*!
 *  \file pe/math/typetraits/IsDenseVector.h
 *  \brief Header file for the IsDenseVector type trait
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

#ifndef _PE_MATH_TYPETRAITS_ISDENSEVECTOR_H_
#define _PE_MATH_TYPETRAITS_ISDENSEVECTOR_H_


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

template< typename, bool > struct DenseVector;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsDenseVector type trait.
 * \ingroup math_type_traits
 */
template< typename T >
struct IsDenseVectorHelper
{
private:
   //**********************************************************************************************
   typedef typename boost::remove_cv<T>::type  T2;
   //**********************************************************************************************

public:
   //**********************************************************************************************
   enum { value = boost::is_base_of< DenseVector<T2,false>, T2 >::value ||
                  boost::is_base_of< DenseVector<T2,true >, T2 >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for dense vector types.
 * \ingroup math_type_traits
 *
 * This type trait tests whether or not the given template parameter is a dense, N-dimensional
 * vector type. In case the type is a dense vector type, the \a value member enumeration is
 * set to 1, the nested type definition \a Type is \a TrueType, and the class derives from
 * \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives
 * from \a FalseType.

   \code
   pe::IsDenseVector< VectorN<double> >::value         // Evaluates to 1
   pe::IsDenseVector< const Vector3<float> >::Type     // Results in TrueType
   pe::IsDenseVector< volatile Vector6<int> >          // Is derived from TrueType
   pe::IsDenseVector< SparseVectorN<double> >::value   // Evaluates to 0
   pe::IsDenseVector< SparseMatrixMxN<double> >::Type  // Results in FalseType
   pe::IsDenseVector< MatrixMxN<double> >              // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsDenseVector : public IsDenseVectorHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsDenseVectorHelper<T>::value };
   typedef typename IsDenseVectorHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
