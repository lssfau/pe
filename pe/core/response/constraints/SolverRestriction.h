//=================================================================================================
/*!
 *  \file pe/core/response/constraints/SolverRestriction.h
 *  \brief Constraint on the data type
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

#ifndef _PE_CORE_RESPONSE_CONSTRAINTS_SOLVERRESTRICTION_H_
#define _PE_CORE_RESPONSE_CONSTRAINTS_SOLVERRESTRICTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/ConstraintTest.h>
#include <pe/util/Suffix.h>
#include <pe/util/TypeList.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  COMPLEMENTARITY_SOLVER_RESTRICTION CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compile time constraint.
 * \ingroup collision_response_constraints
 *
 * Helper template class for the compile time constraint enforcement. Based on the compile time
 * constant expression used for the template instantiation, either the undefined basic template
 * or the specialization is selected. If the undefined basic template is selected, a compilation
 * error is created.
 */
template< bool > struct CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED;
template<> struct CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the data type.
 * \ingroup collision_response_constraints
 *
 * In case the given complementarity solver \a T is not contained in the type list \a TYPELIST,
 * a compilation error is created.
 */
#define pe_CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION(T,TYPELIST) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::response::CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED< pe::Contains<TYPELIST,T>::value >::value > \
      pe_JOIN( CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_TYPEDEF, __LINE__ )
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
