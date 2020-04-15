//=================================================================================================
/*!
 *  \file pe/math/solvers/CG.h
 *  \brief Header file for the conjugate gradient solver
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

#ifndef _PE_MATH_SOLVERS_CG_H_
#define _PE_MATH_SOLVERS_CG_H_


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
/*!\brief A conjugate gradient solver.
 * \ingroup lse_solvers
 *
 * TODO: description
 * TODO: Problem formulation: \f$ A \cdot x + b = 0 \f$ !!
 */
class CG : public Solver
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit CG();
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   bool solve( LSE& lse );
   bool solve( const SMatN& A, const VecN& b, VecN& x );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   VecN r_;  //!< TODO
   VecN d_;  //!< TODO
   VecN h_;  //!< TODO
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
 *
 * TODO: description
 * TODO: Problem formulation: \f$ A \cdot x + b = 0 \f$ !!
 */
inline bool CG::solve( LSE& lse ) {
   return solve( lse.A_, lse.b_, lse.x_ );
}
//*************************************************************************************************

} // namespace solvers

} // namespace pe

#endif
