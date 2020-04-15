//=================================================================================================
/*!
 *  \file pe/math/shims/Reset.h
 *  \brief Header file for the reset shim
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

#ifndef _PE_MATH_SHIMS_RESET_H_
#define _PE_MATH_SHIMS_RESET_H_


namespace pe {

//=================================================================================================
//
//  RESET SHIM
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resetting the given value/object to the default value.
 * \ingroup math_shims
 *
 * \param resettable The value/object to be resetted.
 * \return void
 *
 * The reset shim represents an abstract interface for the resetting of a value/object of
 * any given data type to its default value. Values of built-in data type are reset to zero.
 */
template< typename Type >
inline void reset( Type& resettable )
{
   resettable = Type(0);
}
//*************************************************************************************************

} // namespace pe

#endif
