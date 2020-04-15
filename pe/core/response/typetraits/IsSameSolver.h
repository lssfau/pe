//=================================================================================================
/*!
 *  \file pe/core/response/typetraits/IsSameSolver.h
 *  \brief Header file for the IsSameSolver type trait
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

#ifndef _PE_CORE_RESPONSE_TYPETRAITS_ISSAMESOLVER_H_
#define _PE_CORE_RESPONSE_TYPETRAITS_ISSAMESOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/FalseType.h>
#include <pe/util/TrueType.h>


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
 * This class tests if the two data types \a A and \a B are the same constraint solver type.
 * If \a A and \a B are the same data type, then the \a value member enumeration is set to 1,
 * the nested type definition \a Type is \a TrueType, and the class derives from \a TrueType.
 * Otherwise \a value is set to 0, \a Type is \a FalseType, and the class derives from
 * \a FalseType.

   \code
   pe::IsSameSolver< FFDSolver, FFDSolver >::value                 // Evaluates to 1
   pe::IsSameSolver< BoxFrictionSolver, BoxFrictionSolver >::Type  // Results in TrueType
   pe::IsSameSolver< ConeFrictionSolver, ConeFrictionSolver >      // Is derived from TrueType
   pe::IsSameSolver< BoxFrictionSolver, FFDSolver >::value         // Evaluates to 0
   pe::IsSameSolver< FFDSolver, ConeFrictionSolver >::Type         // Results in FalseType
   pe::IsSameSolver< BoxFrictionSolver, ConeFrictionSolver >       // Is derived from FalseType
   \endcode
 */
template< template<typename,typename,typename> class A
        , template<typename,typename,typename> class B >
struct IsSameSolver : public FalseType
{
public:
   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   enum { value = 0 };
   typedef FalseType  Type;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
//! Specialization of the IsSameSolver type trait for two equal constraint solver types.
template< template<typename,typename,typename> class T >
struct IsSameSolver<T,T> : public TrueType
{
public:
   //**********************************************************************************************
   enum { value = 1 };
   typedef TrueType  Type;
   //**********************************************************************************************
};
/*! \endcond */
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
