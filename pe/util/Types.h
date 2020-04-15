//=================================================================================================
/*!
 *  \file pe/util/Types.h
 *  \brief Header file for basic type definitions
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

#ifndef _PE_UTIL_TYPES_H_
#define _PE_UTIL_TYPES_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cstddef>
#include <boost/cstdint.hpp>


namespace pe {

//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\class pe::size_t
 * \brief Size type of the physics engine.
 * \ingroup util
 */
using std::size_t;
//*************************************************************************************************


//*************************************************************************************************
/*!\class pe::ptrdiff_t
 * \brief Pointer difference type of the physics engine.
 * \ingroup util
 */
using std::ptrdiff_t;
//*************************************************************************************************


//*************************************************************************************************
/*!\class pe::int32_t
 * \brief 32-bit signed integer type of the physics engine.
 * \ingroup util
 */
using boost::int32_t;
//*************************************************************************************************


//*************************************************************************************************
/*!\class pe::uint32_t
 * \brief 32-bit unsigned integer type of the physics engine.
 * \ingroup util
 */
using boost::uint32_t;
//*************************************************************************************************


//*************************************************************************************************
/*!\class pe::int64_t
 * \brief 64-bit signed integer type of the physics engine.
 * \ingroup util
 */
#ifndef BOOST_NO_INT64_T
using boost::int64_t;
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\class pe::uint64_t
 * \brief 64-bit unsigned integer type of the physics engine.
 * \ingroup util
 */
#ifndef BOOST_NO_INT64_T
using boost::uint64_t;
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The largest available signed integer data type.
 * \ingroup util
 */
#ifndef BOOST_NO_INT64_T
typedef int64_t  large_t;
#else
typedef int32_t  large_t;
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief The largest available unsigned integer data type.
 * \ingroup util
 */
#ifndef BOOST_NO_INT64_T
typedef uint64_t  ularge_t;
#else
typedef uint32_t  ularge_t;
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unsigned integer data type for integral IDs.
 * \ingroup util
 */
typedef ularge_t  id_t;
//*************************************************************************************************

} // namespace pe

#endif
