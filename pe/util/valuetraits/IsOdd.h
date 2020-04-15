//=================================================================================================
/*!
 *  \file pe/util/valuetraits/IsOdd.h
 *  \brief Header file for the IsEven value trait
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

#ifndef _PE_UTIL_VALUETRAITS_ISODD_H_
#define _PE_UTIL_VALUETRAITS_ISODD_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

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
/*!\brief Compile time check whether a compile time constant expression is odd.
 * \ingroup value_traits
 *
 * This value trait tests whether the given integral value \a N is an odd value. In case the
 * value is odd, the \a value member enumeration is set to 1, the nested type definition
 * \a Type is \a TrueType, and the class derives from \a TrueType. Otherwise \a value is set
 * to 0, \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   pe::IsOdd<1>::value   // Evaluates to 1
   pe::IsOdd<3>::Type    // Results in TrueType
   pe::IsOdd<5>          // Is derived from TrueType
   pe::IsOdd<2>::value   // Evaluates to 0
   pe::IsOdd<4>::Type    // Results in FalseType
   pe::IsOdd<6>          // Is derived from FalseType
   \endcode
 */
template< size_t N >
struct IsOdd : public SelectType<N%2,TrueType,FalseType>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = ( N%2 )?( 1 ):( 0 ) };
   typedef typename SelectType<N%2,TrueType,FalseType>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
