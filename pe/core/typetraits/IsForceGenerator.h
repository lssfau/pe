//=================================================================================================
/*!
 *  \file pe/core/typetraits/IsForceGenerator.h
 *  \brief Header file for the IsForceGenerator type trait
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

#ifndef _PE_CORE_TYPETRAITS_ISFORCEGENERATOR_H_
#define _PE_CORE_TYPETRAITS_ISFORCEGENERATOR_H_


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

class ForceGenerator;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsForceGenerator type trait.
 * \ingroup core_type_traits
 */
template< typename T >
struct IsForceGeneratorHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of< ForceGenerator, T >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for force generator types.
 * \ingroup core_type_traits
 *
 * This type trait tests whether or not the given template parameter is a force generator type
 * (i.e., is inheritance related to class ForceGenerator, as for instance Spring or Gravity).
 * In case the type is a (possibly cv-qualified) force generator type, the \a value member
 * enumeration is set to 1, the nested type definition \a Type is \a TrueType, and the class
 * derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the
 * class derives from \a FalseType.

   \code
   pe::IsForceGenerator< ForceGenerator >::value  // Evaluates to 1
   pe::IsForceGenerator< const Gravity >::Type    // Results in TrueType
   pe::IsForceGenerator< volatile Spring >        // Is derived from TrueType
   pe::IsForceGenerator< RigidBody >::value       // Evaluates to 0
   pe::IsForceGenerator< const Joint >::Type      // Results in FalseType
   pe::IsForceGenerator< volatile Link >          // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsForceGenerator : public IsForceGeneratorHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsForceGeneratorHelper<T>::value };
   typedef typename IsForceGeneratorHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
