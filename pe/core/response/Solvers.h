//=================================================================================================
/*!
 *  \file pe/core/response/Solvers.h
 *  \brief Header file for the collision response algorithms
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

#ifndef _PE_CORE_RESPONSE_SOLVERS_H_
#define _PE_CORE_RESPONSE_SOLVERS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/response/BoxFrictionSolver.h>
#include <pe/core/response/ConeFrictionSolver.h>
#include <pe/core/response/DEMSolver.h>
#include <pe/core/response/DEMSolverObsolete.h>
#include <pe/core/response/FFDSolver.h>
#include <pe/core/response/FrictionlessSolver.h>
#include <pe/core/response/HardContactSemiImplicitTimesteppingSolvers.h>
#if HAVE_OPENCL
#include <pe/core/response/OpenCLSolver.h>
#endif
#include <pe/core/response/PolyhedralFrictionSolver.h>

#endif
