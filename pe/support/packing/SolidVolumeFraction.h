//=================================================================================================
/*!
 *  \file pe/support/packing/Octree.h
 *  \brief Header file for the solid volume fraction function prototypes.
 *
 *  Copyright (C) 2016 Tobias Preclik
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

#ifndef _PE_SUPPORT_PACKING_SOLIDVOLUMEFRACTION_H_
#define _PE_SUPPORT_PACKING_SOLIDVOLUMEFRACTION_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/support/packing/SolidVolumeBoundingBox.h>
#include <pe/core/domaindecomp/RectilinearGrid.h>


namespace pe {

SolidVolumeBoundingBox determineSolidVolumeBoundingBox();

real determineLocalSolidVolumeFraction( SolidVolumeBoundingBox solidVolumeBoundingBox, real maxSamplingDistance );

#if HAVE_MPI
real determineSolidVolumeFraction( SolidVolumeBoundingBox solidVolumeBoundingBox, const RectilinearGrid& grid, real maxSamplingDistance );
#endif

} // namespace pe

#endif
