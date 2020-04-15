//=================================================================================================
/*!
 *  \file pe/math/solvers/GaussianElimination.h
 *  \brief Header file for the GaussianElimination class
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

#ifndef _PE_MATH_SOLVERS_GAUSSIANELIMINATION_H_
#define _PE_MATH_SOLVERS_GAUSSIANELIMINATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/math/MatrixMxN.h>
#include <pe/math/problems/LSE.h>
#include <pe/math/solvers/Solver.h>
#include <pe/math/SparseMatrixMxN.h>
#include <pe/math/VectorN.h>


namespace pe {

namespace solvers {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of the Gaussian elimination direct linear system solver.
 * \ingroup lse_solvers
 *
 * TODO: description
 * TODO: Problem formulation: \f$ A \cdot x + b = 0 \f$ !!
 */
class GaussianElimination : public Solver
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit GaussianElimination();
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   inline bool solve( LSE& lse );
          bool solve( const SMatN& A, const VecN& b, VecN& x );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   MatN A_;  //!< TODO
   VecN b_;  //!< TODO
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * \param lse TODO
 * \return TODO
 * \exception std::invalid_argument Invalid matrix size.
 * \exception std::invalid_argument Invalid right-hand side vector size.
 *
 * TODO: description
 * TODO: Problem formulation: \f$ A \cdot x + b = 0 \f$ !!
 */
inline bool GaussianElimination::solve( LSE& lse )
{
   return solve( lse.A_, lse.b_, lse.x_ );
}
//*************************************************************************************************

} // namespace solvers

} // namespace pe

#endif
