//=================================================================================================
/*!
 *  \file pe/core/typetraits/IsBallJoint.h
 *  \brief Header file for the IsBallJoint type trait
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

#ifndef _PE_CORE_TYPETRAITS_ISBALLJOINT_H_
#define _PE_CORE_TYPETRAITS_ISBALLJOINT_H_


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

class BallJoint;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsBallJoint type trait.
 * \ingroup core_type_traits
 */
template< typename T >
struct IsBallJointHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of< BallJoint, T >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for ball joint types.
 * \ingroup core_type_traits
 *
 * This type trait tests whether or not the given template parameter is a ball joint type
 * (i.e., related to class BallJoint). In case the type is a ball joint type, the \a value
 * member enumeration is set to 1, the nested type definition \a Type is \a TrueType, and the
 * class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType,
 * and the class derives from \a FalseType.

   \code
   pe::IsBallJoint< BallJoint >::value         // Evaluates to 1
   pe::IsBallJoint< const BallJoint >::Type    // Results in TrueType
   pe::IsBallJoint< volatile BallJoint >       // Is derived from TrueType
   pe::IsBallJoint< FixedJoint >::value        // Evaluates to 0
   pe::IsBallJoint< const SliderJoint >::Type  // Results in FalseType
   pe::IsBallJoint< volatile HingeJoint >      // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsBallJoint : public IsBallJointHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsBallJointHelper<T>::value };
   typedef typename IsBallJointHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
