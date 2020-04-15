//=================================================================================================
/*!
 *  \file pe/core/detection/fine/typetraits/IsSelectedDetector.h
 *  \brief Header file for the IsSelectedDetector type trait
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

#ifndef _PE_CORE_DETECTION_FINE_TYPETRAITS_ISSELECTEDDETECTOR_H_
#define _PE_CORE_DETECTION_FINE_TYPETRAITS_ISSELECTEDDETECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/detection/fine/typetraits/IsSameDetector.h>
#include <pe/util/FalseType.h>
#include <pe/util/TrueType.h>
#include <pe/system/Collisions.h>


namespace pe {

namespace detection {

namespace fine {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time check for fine collision detector types.
 * \ingroup fine_collision_detection_type_traits
 *
 * This class tests if the given data type \a A is the compile time selected fine collision
 * detector type pe::pe_FINE_COLLISION_DETECTOR. If \a A is the same detector type, then the
 * \a value member enumeration is set to 1, the nested type definition \a Type is \a TrueType,
 * and the class derives from \a TrueType. Otherwise \a value is set to 0, \a Type is
 * \a FalseType, and the class derives from \a FalseType.\n
 * The following examples demonstrates the use of the type trait with the assumption that
 * pe::pe_FINE_COLLISION_DETECTOR is set to MaxContacts:

   \code
   pe::IsSelectedDetector< MaxContacts >::value  // Evaluates to 1
   pe::IsSelectedDetector< MaxContacts >::Type   // Results in TrueType
   pe::IsSelectedDetector< MaxContacts >         // Is derived from TrueType
   \endcode
 */
template< typename T >
struct IsSelectedDetector : public IsSameDetector<T,pe_FINE_COLLISION_DETECTOR>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsSameDetector<T,pe_FINE_COLLISION_DETECTOR>::value };
   typedef typename IsSameDetector<T,pe_FINE_COLLISION_DETECTOR>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace fine

} // namespace detection

} // namespace pe

#endif
