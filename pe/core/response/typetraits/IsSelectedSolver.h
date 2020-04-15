//=================================================================================================
/*!
 *  \file pe/core/response/typetraits/IsSelectedSolver.h
 *  \brief Header file for the IsSelectedSolver type trait
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

#ifndef _PE_CORE_RESPONSE_TYPETRAITS_ISSELECTEDSOLVER_H_
#define _PE_CORE_RESPONSE_TYPETRAITS_ISSELECTEDSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/response/typetraits/IsSameSolver.h>
#include <pe/util/FalseType.h>
#include <pe/util/TrueType.h>
#include <pe/system/Collisions.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Compile time check for constraint solver types.
 * \ingroup collision_response_type_traits
 *
 * This class tests if the given data type \a A is the compile time selected constraint solver
 * type pe::pe_CONSTRAINT_SOLVER. If \a A is the same data type, then the \a value member
 * enumeration is set to 1, the nested type definition \a Type is \a TrueType, and the class
 * derives from \a TrueType. Otherwise \a value is set to 0, \a Type is \a FalseType, and the
 * class derives from \a FalseType.\n
 * The following examples demonstrates the use of the type trait with the assumption that
 * pe::pe_CONSTRAINT_SOLVER is set to BoxFrictionSolver:

   \code
   pe::IsSelectedSolver< BoxFrictionSolver >::value  // Evaluates to 1
   pe::IsSelectedSolver< BoxFrictionSolver >::Type   // Results in TrueType
   pe::IsSelectedSolver< BoxFrictionSolver >         // Is derived from TrueType
   pe::IsSelectedSolver< FFDSolver >::value          // Evaluates to 0
   pe::IsSelectedSolver< ConeFrictionSolver >::Type  // Results in FalseType
   pe::IsSelectedSolver< PolyhedralFrictionSolver>   // Is derived from FalseType
   \endcode
 */
template< template<typename,typename,typename> class T >
struct IsSelectedSolver : public IsSameSolver<T,pe_CONSTRAINT_SOLVER>::Type
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = IsSameSolver<T,pe_CONSTRAINT_SOLVER>::value };
   typedef typename IsSameSolver<T,pe_CONSTRAINT_SOLVER>::Type  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
