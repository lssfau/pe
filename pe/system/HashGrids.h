//=================================================================================================
/*!
 *  \file pe/system/HashGrids.h
 *  \brief System settings for the hierarchical hash grids coarse collision detection algorithm
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

#ifndef _PE_SYSTEM_HASHGRIDS_H_
#define _PE_SYSTEM_HASHGRIDS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/Precision.h>
#include <pe/util/StaticAssert.h>
#include <pe/util/Types.h>
#include <pe/util/valuetraits/IsPowerOf.h>


namespace pe {

namespace detection {

namespace coarse {

namespace hhg {

//=================================================================================================
//
//  HIERARCHICAL HASH GRID COARSE COLLISION DETECTION SETTINGS
//
//=================================================================================================

#include <pe/config/HashGrids.h>




//=================================================================================================
//
//  COMPILE TIME CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
namespace {

pe_STATIC_ASSERT( xCellCount >= 4 && ( IsPowerOf<2,xCellCount>::value ) );
pe_STATIC_ASSERT( yCellCount >= 4 && ( IsPowerOf<2,yCellCount>::value ) );
pe_STATIC_ASSERT( zCellCount >= 4 && ( IsPowerOf<2,zCellCount>::value ) );
pe_STATIC_ASSERT( minimalGridDensity > 0 );

}
/*! \endcond */
//*************************************************************************************************

} // namespace hhg

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
