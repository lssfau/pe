//=================================================================================================
/*!
 *  \file pe/system/LCPConfig.h
 *  \brief System settings for for the complementarity solvers
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

#ifndef _PE_SYSTEM_LCPCONFIG_H_
#define _PE_SYSTEM_LCPCONFIG_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/solvers/Types.h>
#include <pe/system/Precision.h>
#include <pe/util/StaticAssert.h>
#include <pe/util/Types.h>


namespace pe {

namespace response {

namespace lcp {

//=================================================================================================
//
//  COMPLEMENTARITY SOLVER SETTINGS
//
//=================================================================================================

#include <pe/config/LCPConfig.h>




//=================================================================================================
//
//  COMPILE TIME CONSTRAINT
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
namespace {

pe_STATIC_ASSERT( maxIterations > 0 );
pe_STATIC_ASSERT( facets >= 4 && facets % 2 == 0 );

}
/*! \endcond */
//*************************************************************************************************

} // namespace lcp

} // namespace response

} // namespace pe

#endif
