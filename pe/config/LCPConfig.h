//=================================================================================================
/*!
 *  \file pe/config/LCPConfig.h
 *  \brief Configuration file for the complementarity solvers
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
/*!\brief Selection of the complementarity solver.
 * \ingroup config
 *
 * This type definition selects the (initial) complementarity solver for LCP-based collision
 * response algorithms. The following solvers are available:
 *   - ::pe::solver::Lemke
 *   - ::pe::solver::PGS
 *   - ::pe::solver::CPG
 */
#ifndef pe_COMPLEMENTARITY_SOLVER
typedef ::pe::solvers::PGS  ComplementaritySolver;
#else
typedef pe_COMPLEMENTARITY_SOLVER ComplementaritySolver;
#endif
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Maximum number of iterations of the complementarity solver.
 * \ingroup config
 *
 * This value specifies the maximum number of iteration steps the complementarity solver
 * is performing for a single LCP problem. The choice of the number of iteration steps is a
 * consideration between the accuracy of the solution and the time to solution: for a large
 * number of iteration steps, the solution will be more accurate, whereas a small number
 * will be considerable faster to calculate. For instance, an important property of the PGS
 * solver is that the convergence speed of the approximated solution is strongly decreasing
 * with a larger number of time steps. In order to noticeably improve the solution, the
 * maximum number of time steps has to be considerably increased.
 *
 * Possible settings for the \a maxIterations variable: \f$ [1..\infty) \f$
 */
const size_t maxIterations = 25000;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Residuum threshold for the LCP solution of the complementarity solver.
 * \ingroup config
 *
 * This value specifies the threshold for the residuum calculation during a solution of a
 * single LCP problem. The calculation stops as soon as the approximated solution meets the
 * specified threshold value. The choice of the threshold value is a consideration between
 * the accuracy of the solution and the time to solution: for a small threshold value (e.g.
 * \f$ 1e-9 \f$), the solution will be more accurate, whereas a large threshold value (e.g.
 * \f$ 1e-5 \f$) will be considerable faster. For instance, an important property of the PGS
 * solver is that the convergance speed of the approximated solution is strongly decreasing
 * with a larger number of time steps. The time to solution for a LCP calculation using a
 * smaller threshold value will be noticeably larger.
 */
const real threshold = static_cast<real>( 5e-7 );
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Number of facets of the approximated friction cone.
 * \ingroup config
 *
 * The Lemke LCP solvers uses a linear approximation of the friction cone. For this approximation
 * the number of facets can be specified. The more facets are used, the more accurate the solver
 * can solve the frictional contact problem, but the slower the solver will be. The number of
 * facets can be set to any positive, even number greater-or-equal to 4.
 */
const size_t facets = 4;
//*************************************************************************************************
