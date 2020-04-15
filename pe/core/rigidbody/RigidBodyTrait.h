//=================================================================================================
/*!
 *  \file pe/core/RigidBodyTrait.h
 *  \brief Header file for the RigidBodyTrait class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODYTRAIT_H_


//*************************************************************************************************
// Forward Declarations
//*************************************************************************************************

namespace pe {
template<typename> class ProcessTrait;
} // namespace pe


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/rigidbodytrait/Default.h>
#include <pe/core/rigidbody/rigidbodytrait/FFDSolver.h>
#include <pe/core/rigidbody/rigidbodytrait/DEMSolver.h>
#include <pe/core/rigidbody/rigidbodytrait/DEMSolverObsolete.h>
#include <pe/core/rigidbody/rigidbodytrait/HardContactSemiImplicitTimesteppingSolvers.h>


#endif
