//=================================================================================================
/*!
 *  \file pe/util/Byte.h
 *  \brief Header file for the byte type
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

#ifndef _PE_UTIL_BYTE_H_
#define _PE_UTIL_BYTE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/constraints/Integral.h>
#include <pe/util/constraints/Size.h>


namespace pe {

//=================================================================================================
//
//  TYPE DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Byte data type of the physics engine.
 * \ingroup util
 *
 * The \a byte data type is guaranteed to be an integral data type of size 1.
 */
typedef unsigned char  byte;
//*************************************************************************************************




//=================================================================================================
//
//  COMPILE TIME CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
namespace check {

// Constraint on the byte data type
pe_CONSTRAINT_MUST_BE_INTEGRAL_TYPE( byte );
pe_CONSTRAINT_MUST_HAVE_SIZE( byte, 1 );

} // namespace check
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
