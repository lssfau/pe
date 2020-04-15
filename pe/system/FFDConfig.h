//=================================================================================================
/*!
 *  \file pe/system/FFDConfig.h
 *  \brief System settings for the fast frictional dynamics solver
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

#ifndef _PE_SYSTEM_FFDCONFIG_H_
#define _PE_SYSTEM_FFDCONFIG_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/Precision.h>
#include <pe/util/StaticAssert.h>
#include <pe/util/Types.h>


namespace pe {

namespace response {

namespace ffd {

//=================================================================================================
//
//  FAST FRICTIONAL DYNAMICS SOLVER SETTINGS
//
//=================================================================================================

#include <pe/config/FFDConfig.h>




//=================================================================================================
//
//  COMPILE TIME CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
namespace {

pe_STATIC_ASSERT( frictionSamples > 0 );

}
/*! \endcond */
//*************************************************************************************************

} // namespace ffd

} // namespace response

} // namespace pe

#endif
