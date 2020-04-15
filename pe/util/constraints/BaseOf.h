//=================================================================================================
/*!
 *  \file pe/util/constraints/BaseOf.h
 *  \brief Constraint on the inheritance relationship of a data type
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

#ifndef _PE_UTIL_CONSTRAINTS_BASEOF_H_
#define _PE_UTIL_CONSTRAINTS_BASEOF_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/ConstraintTest.h>
#include <pe/util/Suffix.h>
#include <pe/util/typetraits/IsBaseOf.h>


namespace pe {

//=================================================================================================
//
//  MUST_BE_BASE_OF CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_BE_BASE_OF_FAILED;
template<> struct CONSTRAINT_MUST_BE_BASE_OF_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the inheritance relationship.
 * \ingroup constraints
 *
 * In case \a B is not a base class of \a D, a compilation error is created.
 */
#define pe_CONSTRAINT_MUST_BE_BASE_OF(B,D) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_BASE_OF_FAILED< pe::IsBaseOf<B,D>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_BASE_OF_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_NOT_BE_BASE_OF CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_NOT_BE_BASE_OF_FAILED;
template<> struct CONSTRAINT_MUST_NOT_BE_BASE_OF_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the inheritance relationship.
 * \ingroup constraints
 *
 * In case \a B is a base class of \a D or in case \a B is the same type as \a D, a
 * compilation error is created.
 */
#define pe_CONSTRAINT_MUST_NOT_BE_BASE_OF(B,D) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_NOT_BE_BASE_OF_FAILED< !pe::IsBaseOf<B,D>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_NOT_BE_BASE_OF_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_BE_STRICTLY_BASE_OF CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_BE_STRICTLY_BASE_OF_FAILED;
template<> struct CONSTRAINT_MUST_BE_STRICTLY_BASE_OF_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the inheritance relationship.
 * \ingroup constraints
 *
 * In case \a B is not a base class of \a D, a compilation error is created. In contrast to
 * the pe_CONSTRAINT_MUST_BE_BASE_OF constraint, a compilation error is also created in case
 * \a B and \a D are the same type.
 */
#define pe_CONSTRAINT_MUST_BE_STRICTLY_BASE_OF(B,D) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_STRICTLY_BASE_OF_FAILED< pe::IsBaseOf<B,D>::value && \
                                                        !pe::IsBaseOf<D,B>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_STRICTLY_BASE_OF_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_NOT_BE_STRICTLY_BASE_OF CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_NOT_BE_STRICTLY_BASE_OF_FAILED;
template<> struct CONSTRAINT_MUST_NOT_BE_STRICTLY_BASE_OF_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constraint on the inheritance relationship.
 * \ingroup constraints
 *
 * In case \a B is a base class of \a D, a compilation error is created. In contrast to the
 * pe_CONSTRAINT_MUST_NOT_BE_BASE_OF constraint, no compilation error is created in case \a B
 * and \a D are the same type.
 */
#define pe_CONSTRAINT_MUST_NOT_BE_STRICTLY_BASE_OF(B,D) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_NOT_BE_STRICTLY_BASE_OF_FAILED< !pe::IsBaseOf<B,D>::value || \
                                                              pe::IsBaseOf<D,B>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_NOTBE_STRICTLY_BASE_OF_TYPEDEF, __LINE__ )
//*************************************************************************************************

} // namespace pe

#endif
