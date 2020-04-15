//=================================================================================================
/*!
 *  \file pe/util/FalseType.h
 *  \brief Header file for the FalseType type/value trait base class
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

#ifndef _PE_UTIL_FALSETYPE_H_
#define _PE_UTIL_FALSETYPE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/type_traits/integral_constant.hpp>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Type/value traits base class.
 * \ingroup util
 *
 * The FalseType class is used as base class for type traits and value traits that evaluate to
 * \a false.
 */
typedef boost::false_type  FalseType;
//*************************************************************************************************

} // namespace pe

#endif
