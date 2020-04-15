//=================================================================================================
/*!
 *  \file pe/config/Collisions.h
 *  \brief Configuration of the collision treatment algorithms
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


//*************************************************************************************************
/*!\brief Selection of the coarse collision detection algorithm.
 * \ingroup config
 *
 * This macro selects the coarse collision detection algorithm. The purpose of this algorithm
 * is used to efficiently sort out all pairs of rigid bodies that are potentially colliding
 * and sending them to the fine collision detection. The following algorithms can be chosen:
 *   - ::pe::detection::coarse::ExhaustiveSearch
 *   - ::pe::detection::coarse::HashGrids
 *   - ::pe::detection::coarse::SweepAndPrune
 */
#ifndef pe_COARSE_COLLISION_DETECTOR
#define pe_COARSE_COLLISION_DETECTOR  ::pe::detection::coarse::HashGrids
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Selection of the fine collision detection algorithm.
 * \ingroup config
 *
 * This macro selects the fine collision detection algorithm. The purpose of this algorithm is
 * the generation of contact points between colliding rigid bodies. The following algorithms
 * are available for the fine collision detection:
 *   - ::pe::detection::fine::MaxContacts
 */
#ifndef pe_FINE_COLLISION_DETECTOR
#define pe_FINE_COLLISION_DETECTOR  ::pe::detection::fine::MaxContacts
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Selection of the batch generation algorithm.
 * \ingroup config
 *
 * This macro selects the batch generation algorithm. The purpose of this algorithm is the
 * setup of distinct groups/batches of contacts that can be treated in parallel and independent
 * of each other. The following algorithms are available:
 *   - ::pe::batches::SingleBatch
 *   - ::pe::batches::UnionFind
 */
#ifndef pe_BATCH_GENERATOR
#define pe_BATCH_GENERATOR  ::pe::batches::UnionFind
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Selection of the collision response algorithm.
 * \ingroup config
 *
 * This macro selects the collision response algorithm. The collision response is responsible
 * to calculate contact forces/impulses to handle collisions between rigid bodies. The following
 * solvers are available:
 *   - ::pe::response::FrictionlessSolver
 *   - ::pe::response::BoxFrictionSolver
 *   - ::pe::response::ConeFrictionSolver
 *   - ::pe::response::PolyhedralFrictionSolver
 *   - ::pe::response::DEMSolver
 *   - ::pe::response::DEMSolverObsolete
 *   - ::pe::response::FFDSolver
 *   - ::pe::response::HardContactSemiImplicitTimesteppingSolvers
 *   - ::pe::response::OpenCLSolver
 */
#ifndef pe_CONSTRAINT_SOLVER
#define pe_CONSTRAINT_SOLVER  ::pe::response::HardContactSemiImplicitTimesteppingSolvers
#endif
//*************************************************************************************************
