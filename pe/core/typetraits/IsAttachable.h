//=================================================================================================
/*!
 *  \file pe/core/typetraits/IsAttachable.h
 *  \brief Header file for the IsAttachable type trait
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

#ifndef _PE_CORE_TYPETRAITS_ISATTACHABLE_H_
#define _PE_CORE_TYPETRAITS_ISATTACHABLE_H_


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

class Attachable;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Auxiliary helper struct for the IsAttachable type trait.
 * \ingroup core_type_traits
 */
template< typename T >
struct IsAttachableHelper
{
   //**********************************************************************************************
   enum { value = boost::is_base_of< Attachable, T >::value };
   typedef typename SelectType<value,TrueType,FalseType>::Type  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compile time check for attachable types.
 * \ingroup core_type_traits
 *
 * This type trait tests whether or not the given template parameter is an attachable type
 * (i.e., a type related to class Attachable, as for instance force generators or joints).
 * In case the type is an attachable type, the \a value member enumeration is set to 1, the
 * nested type definition \a Type is \a TrueType, and the class derives from \a TrueType.
 * Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives from
 * \a FalseType.

   \code
   pe::IsAttachable< Attachable >::value           // Evaluates to 1
   pe::IsAttachable< const ForceGenerator >::Type  // Results in TrueType
   pe::IsAttachable< volatile Joint >              // Is derived from TrueType
   pe::IsAttachable< RigidBody >::value            // Evaluates to 0
   pe::IsAttachable< const Contact >::Type         // Results in FalseType
   pe::IsAttachable< volatile Link >               // Is derived from FalseType
   \endcode
 */
template< typename T >
struct IsAttachable : public IsAttachableHelper<T>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsAttachableHelper<T>::value };
   typedef typename IsAttachableHelper<T>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace pe

#endif
