//=================================================================================================
/*!
 *  \file pe/math/shims/IsNaN.h
 *  \brief Header file for the isnan shim
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

#ifndef _PE_MATH_SHIMS_ISNAN_H_
#define _PE_MATH_SHIMS_ISNAN_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/FloatingPoint.h>


//*************************************************************************************************
// Macro undefinition
//*************************************************************************************************

#ifdef isnan
#  undef isnan
#endif


namespace pe {

//=================================================================================================
//
//  ISNAN SHIM
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Platform independent implementation of the C99 \a isnan function.
 * \ingroup math_shims
 *
 * \param a Floating point value to be checked.
 * \return Non-zero value if \a a is a not a number (NaN).
 *
 * This function provides a platform independent check for NaN values since some compilers
 * don't support the \a isnan function (although is is part of the latest C standard library).
 *
 * \b Note: Since NaN values are only defined for floating point types, this \a isnan can
 * only be used for floating point types. The attempt to use this function for an integral
 * data type results in a compile time error.
 */
template< typename T >
inline bool isnan( T a )
{
   pe_CONSTRAINT_MUST_BE_FLOATING_POINT_TYPE( T );
   return a != a;
}
//*************************************************************************************************

} // namespace pe

#endif
