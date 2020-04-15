//=================================================================================================
/*!
 *  \file pe/math/solvers/PGS.h
 *  \brief Implementation of the projected Gauss-Seidel algorithm
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

#ifndef _PE_MATH_SOLVERS_PGS_H_
#define _PE_MATH_SOLVERS_PGS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/math/problems/BoxLCP.h>
#include <pe/math/problems/ContactLCP.h>
#include <pe/math/problems/LCP.h>
#include <pe/math/solvers/Solver.h>
#include <pe/math/SparseMatrixMxN.h>
#include <pe/math/VectorN.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/Types.h>


namespace pe {

namespace solvers {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A projected Gauss-Seidel Solver for (box) LCPs.
 * \ingroup complementarity_solvers
 *
 * TODO: description of the PGS solver
 * TODO: capabilities of the PGS solver (which LCP problems, etc)
 * TODO: known issues of the PGS solver
 */
class PGS : public Solver
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit PGS();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename CP > bool solve( CP& cp );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename CP > inline real sweep( CP& cp ) const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   VecN diagonal_;  //!< Vector for the diagonal entries of the LCP matrix.
                    /*!< For performance reasons, the vector contains the inverse of the diagonal
                         elements. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Solves the provided complementarity problem.
 *
 * \param cp The complementarity problem to solve.
 * \return Returns \a true if the solution is sufficiently accurate, otherwise it returns \a false.
 *
 * TODO
 */
template< typename CP >  // Type of the complementarity problem
bool PGS::solve( CP& cp )
{
   const size_t n( cp.size() );
   const SMatN& A( cp.A_ );
   bool converged( false );

   // Allocating the helper data
   diagonal_.resize( n, false );

   // Locating the diagonal entries in system matrix and precomputing the inverse values
   for( size_t i=0; i<n; ++i ) {
      const real tmp( A(i,i) );
      pe_INTERNAL_ASSERT( tmp != real(0), "Invalid diagonal element in the LCP matrix" );
      diagonal_[i] = real(1) / tmp;
   }

   // Projecting the initial solution to a feasible region
   for( size_t i=0; i<n; ++i ) {
      cp.project( i );
   }

   // Computing the initial residual
   lastPrecision_ = cp.residual();
   if( lastPrecision_ < threshold_ )
      converged = true;

   // The main iteration loop
   size_t it( 0 );

   for( ; !converged && it<maxIterations_; ++it ) {
      lastPrecision_ = sweep( cp );
      if( lastPrecision_ < threshold_ )
         converged = true;
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( converged )
         log << "      Solved the complementarity problem in " << it << " PGS iterations.";
      else
         log << pe_YELLOW << "      WARNING: Did not solve the complementarity problem within accuracy. (" << lastPrecision_ << ")" << pe_OLDCOLOR;
   }

   lastIterations_ = it;

   return converged;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param cp The complementarity problem to solve.
 * \return TODO
 *
 * TODO
 */
template< typename CP >  // Type of the complementarity problem
inline real PGS::sweep( CP& cp ) const
{
   real maxResidual( 0 ), xold( 0 );
   const size_t n( cp.size() );

   const SMatN& A( cp.A_ );
   const VecN&  b( cp.b_ );
   VecN& x( cp.x_ );

   for( size_t i=0; i<n; ++i )
   {
      const real residual( - b[i] - ( A * x )[i] );

      // Updating and projecting the unknown
      xold = x[i];
      x[i] += diagonal_[i] * residual;
      cp.project( i );
      maxResidual = max( maxResidual, std::fabs( xold - x[i] ) );
   }

   return maxResidual;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param cp The complementarity problem to solve.
 * \return TODO
 *
 * TODO: specialization for contact LCPs
 * TODO: more infos
 */
template<>
inline real PGS::sweep( ContactLCP& cp ) const
{
   const size_t N( cp.size() / 3 );
   real rmax( 0 ), residual, flimit, aux;
   size_t j;

   const SMatN& A( cp.A_ );
   const VecN&  b( cp.b_ );
   VecN& x( cp.x_ );

   for( size_t i=0; i<N; ++i )
   {
      j = i * 3;
      residual = -b[j] - ( A * x )[j];
      aux = max( 0, x[j] + diagonal_[j] * residual );
      rmax = max( rmax, std::fabs( x[j] - aux ) );
      x[j] = aux;

      flimit = cp.cof_[i] * x[j];

      ++j;
      residual = -b[j] - ( A * x )[j];
      aux = max( -flimit, min( flimit, x[j] + diagonal_[j] * residual ) );
      rmax = max( rmax, std::fabs( x[j] - aux ) );
      x[j] = aux;

      ++j;
      residual = -b[j] - ( A * x )[j];
      aux = max( -flimit, min( flimit, x[j] + diagonal_[j] * residual ) );
      rmax = max( rmax, std::fabs( x[j] - aux ) );
      x[j] = aux;
   }

   return rmax;
}
//*************************************************************************************************




//=================================================================================================
//
//  EXPLICIT TEMPLATE INSTANTIATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
#if !defined(_MSC_VER)
extern template bool PGS::solve<LCP>( LCP& );
extern template bool PGS::solve<BoxLCP>( BoxLCP& );
extern template bool PGS::solve<ContactLCP>( ContactLCP& );
#endif
/*! \endcond */
//*************************************************************************************************

} // namespace solvers

} // namespace pe

#endif
