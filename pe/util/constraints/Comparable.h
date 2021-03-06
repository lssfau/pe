//=================================================================================================
/*!
 *  \file pe/util/constraints/Comparable.h
 *  \brief Constraint on the pointer relationship
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

#ifndef _PE_UTIL_CONSTRAINTS_COMPARABLE_H_
#define _PE_UTIL_CONSTRAINTS_COMPARABLE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/ConstraintTest.h>
#include <pe/util/Suffix.h>
#include <pe/util/typetraits/IsConvertible.h>


namespace pe {

//=================================================================================================
//
//  POINTER_MUST_BE_COMPARABLE CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_POINTER_MUST_BE_COMPARABLE_FAILED;
template<> struct CONSTRAINT_POINTER_MUST_BE_COMPARABLE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the pointer relationship.
 * \ingroup constraints
 *
 * In case \a P1 is not comparable with \a P2, a compilation error is created.
 */
#define pe_CONSTRAINT_POINTER_MUST_BE_COMPARABLE(P1,P2) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_POINTER_MUST_BE_COMPARABLE_FAILED< pe::IsConvertible<P1,P2>::value || \
                                                           pe::IsConvertible<P2,P1>::value >::value > \
      pe_JOIN( CONSTRAINT_POINTER_MUST_BE_COMPARABLE_TYPEDEF, __LINE__ )
//*************************************************************************************************

} // namespace pe

#endif
