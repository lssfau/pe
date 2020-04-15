//=================================================================================================
/*!
 *  \file pe/math/typetraits/IsTransposeVector.h
 *  \brief Header file for the IsTransposeVector type trait
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

#ifndef _PE_MATH_TYPETRAITS_ISTRANSPOSEVECTOR_H_
#define _PE_MATH_TYPETRAITS_ISTRANSPOSEVECTOR_H_


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
template< typename, bool > struct SparseVector;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsTransposeVector type trait.
 * \ingroup math_type_traits
 */
template< typename T >
struct IsTransposeVectorHelper
{
private:
   //**********************************************************************************************
   typedef typename boost::remove_cv<T>::type  T2;
   //**********************************************************************************************

public:
   //**********************************************************************************************
   enum { value = boost::is_base_of< DenseVector <T2,true>, T2 >::value ||
                  boost::is_base_of< SparseVector<T2,true>, T2 >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for transpose vector types.
 * \ingroup math_type_traits
 *
 * This type trait tests whether or not the given template argument is a transpose dense or
 * sparse vector type (i.e., a vector whose transposition flag is set to 1). In case the type
 * is a transpose vector type, the \a value member enumeration is set to 1, the nested type
 * definition \a Type is \a TrueType, and the class derives from \a TrueType. Otherwise
 * \a value is set to 0, \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   pe::IsTransposeVector< VectorN<double,true> >::value        // Evaluates to 1
   pe::IsTransposeVector< const Vector3<float,true> >::Type    // Results in TrueType
   pe::IsTransposeVector< volatile SparseVectorN<int,true> >   // Is derived from TrueType
   pe::IsTransposeVector< VectorN<double,false> >::value       // Evaluates to 0
   pe::IsTransposeVector< const Vector3<float,false> >::Type   // Results in FalseType
   pe::IsTransposeVector< volatile SparseVectorN<int,false> >  // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsTransposeVector : public IsTransposeVectorHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsTransposeVectorHelper<T>::value };
   typedef typename IsTransposeVectorHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
