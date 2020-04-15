//=================================================================================================
/*!
 *  \file pe/core/typetraits/IsSliderJoint.h
 *  \brief Header file for the IsSliderJoint type trait
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

#ifndef _PE_CORE_TYPETRAITS_ISSLIDERJOINT_H_
#define _PE_CORE_TYPETRAITS_ISSLIDERJOINT_H_


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

class SliderJoint;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsSliderJoint type trait.
 * \ingroup core_type_traits
 */
template< typename T >
struct IsSliderJointHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of< SliderJoint, T >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for slider joint types.
 * \ingroup core_type_traits
 *
 * This type trait tests whether or not the given template parameter is a slider joint type
 * (i.e., is inheritance related to class SliderJoint). In case the type is a (possibly
 * cv-qualified) slider joint type, the \a value member enumeration is set to 1, the nested
 * type definition \a Type is \a TrueType, and the class derives from \a TrueType. Otherwise
 * \a value is set to 0, \a Type is \a FalseType, and the class derives from \a FalseType.

   \code
   pe::IsSliderJoint< SliderJoint >::value       // Evaluates to 1
   pe::IsSliderJoint< const SliderJoint >::Type  // Results in TrueType
   pe::IsSliderJoint< volatile SliderJoint >     // Is derived from TrueType
   pe::IsSliderJoint< FixedJoint >::value        // Evaluates to 0
   pe::IsSliderJoint< const BallJoint >::Type    // Results in FalseType
   pe::IsSliderJoint< volatile HingeJoint >      // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsSliderJoint : public IsSliderJointHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsSliderJointHelper<T>::value };
   typedef typename IsSliderJointHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
