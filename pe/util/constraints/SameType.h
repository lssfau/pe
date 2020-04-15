//=================================================================================================
/*!
 *  \file pe/util/constraints/SameType.h
 *  \brief Data type constraint
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

#ifndef _PE_UTIL_CONSTRAINTS_SAMETYPE_H_
#define _PE_UTIL_CONSTRAINTS_SAMETYPE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/ConstraintTest.h>
#include <pe/util/Suffix.h>
#include <pe/util/typetraits/IsSame.h>


namespace pe {

//=================================================================================================
//
//  MUST_BE_SAME_TYPE CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_BE_SAME_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_BE_SAME_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Data type constraint.
 * \ingroup constraints
 *
 * In case the two types \a A and \a B are not the same (ignoring all cv-qualifiers of both data
 * types), a compilation error is created. The following example illustrates the behavior of this
 * constraint:

   \code
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( double, double );        // No compilation error
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( double, const double );  // No compilation error (only cv-qualifiers differ)
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( double, float );         // Compilation error, different data types!
   \endcode

 * In case the cv-qualifiers should not be ignored (e.g. 'double' and 'const double' should be
 * considered to be unequal), use the pe::pe_CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE constraint.
 */
#define pe_CONSTRAINT_MUST_BE_SAME_TYPE(A,B) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_SAME_TYPE_FAILED< pe::IsSame<A,B>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_SAME_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_NOT_BE_SAME_TYPE CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_NOT_BE_SAME_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_NOT_BE_SAME_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Data type constraint.
 * \ingroup constraints
 *
 * In case the two types \a A and \a B are the same (ignoring all cv-qualifiers of both data
 * types), a compilation error is created. The following example illustrates the behavior of
 * this constraint:

   \code
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( double, float );         // No compilation error, different data types
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( double, const double );  // Compilation error (only cv-qualifiers differ)
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE( double, double );        // Compilation error, same data type!
   \endcode

 * In case the cv-qualifiers should not be ignored (e.g. 'double' and 'const double' should
 * be considered to be unequal), use the pe::pe_CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE
 * constraint.
 */
#define pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE(A,B) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_NOT_BE_SAME_TYPE_FAILED< !pe::IsSame<A,B>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_NOT_BE_SAME_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_BE_STRICTLY_SAME_TYPE CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Data type constraint.
 * \ingroup constraints
 *
 * In case the two types \a A and \a B are not the same, a compilation error is created. Note
 * that this constraint even considers two types as unequal if the cv-qualifiers differ, e.g.

   \code
   pe_CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE( double, double );        // No compilation error
   pe_CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE( double, const double );  // Compilation error, different cv-qualifiers!
   pe_CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE( double, float );         // Compilation error, different data types!
   \endcode

 * In case the cv-qualifiers should be ignored (e.g. 'double' and 'const double' should be
 * considered to be equal), use the pe::pe_CONSTRAINT_MUST_BE_SAME_TYPE constraint.
 */
#define pe_CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE(A,B) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE_FAILED< pe::IsStrictlySame<A,B>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_BE_STRICTLY_SAME_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************




//=================================================================================================
//
//  MUST_NOT_BE_STRICTLY_SAME_TYPE CONSTRAINT
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
template< bool > struct CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE_FAILED;
template<> struct CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE_FAILED<true> { enum { value = 1 }; };
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Data type constraint.
 * \ingroup constraints
 *
 * In case the two types \a A and \a B are the same, a compilation error is created. Note that
 * this constraint even considers two types as unequal if the cv-qualifiers differ, e.g.

   \code
   pe_CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE( double, float );         // No compilation error, different data types
   pe_CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE( double, const double );  // No compilation error, different cv-qualifiers!
   pe_CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE( double, double );        // Compilation error, same data type!
   \endcode

 * In case the cv-qualifiers should be ignored (e.g. 'double' and 'const double' should be
 * considered to be equal), use the pe::pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE constraint.
 */
#define pe_CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE(A,B) \
   typedef \
      pe::CONSTRAINT_TEST< \
         pe::CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE_FAILED< !pe::IsStrictlySame<A,B>::value >::value > \
      pe_JOIN( CONSTRAINT_MUST_NOT_BE_STRICTLY_SAME_TYPE_TYPEDEF, __LINE__ )
//*************************************************************************************************

} // namespace pe

#endif
