//*************************************************************************************************
/*!
 *  \file src/math/solvers/GaussianElimination.cpp
 *  \brief Source file for the GaussianElimination class
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
//*************************************************************************************************


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <pe/math/Accuracy.h>
#include <pe/math/shims/IsDefault.h>
#include <pe/math/shims/Reset.h>
#include <pe/math/solvers/GaussianElimination.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/Types.h>


namespace pe {

namespace solvers {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for the GaussianElimination class.
 */
GaussianElimination::GaussianElimination()
   : A_()  // TODO
   , b_()  // TODO
{}
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief TODO
 *
 * \param A TODO
 * \param b TODO
 * \param x TODO
 * \return TODO
 * \exception std::invalid_argument Invalid matrix size.
 * \exception std::invalid_argument Invalid right-hand side vector size.
 *
 * TODO: description
 * TODO: problem formulation \f$ A \cdot x + b = 0 \f$ !!
 */
bool GaussianElimination::solve( const SMatN& A, const VecN& b, VecN& x )
{
   if( A.rows() != A.columns() )
      throw std::invalid_argument( "Invalid matrix size" );

   if( A.rows() != b.size() )
      throw std::invalid_argument( "Invalid right-hand side vector size" );

   const size_t n( b.size() );

   // Allocating helper data
   A_ =  A;
   b_ = -b;
   x.resize( n, false );

   size_t pi, pj;
   lastPrecision_ = real(0);

   // Initializing the pivot vector
   VectorN<size_t> p( n );
   for( size_t j=0; j<n; ++j ) {
      p[j] = j;
   }

   // Performing the Gaussian elimination
   for( size_t j=0; j<n; ++j )
   {
      size_t max( j );
      real max_val( std::fabs( A_(p[max],j) ) );

      // Partial search for pivot
      for( size_t i=j+1; i<n; ++i ) {
         if( std::fabs( A_(p[i],j) ) > max_val ) {
            max = i;
            max_val = std::fabs( A_(p[max],j) );
         }
      }

      // Swapping rows such the pivot lies on the diagonal
      std::swap( p[max], p[j] );
      pj = p[j];

      if( !isDefault( A_(pj,j) ) )
      {
         // Eliminating the column below the diagonal
         for( size_t i=j+1; i<n; ++i )
         {
            pi = p[i];
            const real f = A_(pi,j) / A_(pj,j);

            reset( A_(pi,j) );

            for( size_t k=j+1; k<n; ++k ) {
               A_(pi,k) -= A_(pj,k) * f;
            }

            b_[pi] -= b_[pj] * f;
         }
      }
      else {
         // Asserting that the column is zero below the diagonal
         for( size_t i=j+1; i<n; ++i ) {
            pe_INTERNAL_ASSERT( isDefault( A_(p[i],j) ), "Fatal error in Gaussian elimination" );
         }
      }
   }

   // Performing the backward substitution
   for( size_t i=n-1; i<n; --i )
   {
      pi = p[i];
      real rhs = b_[pi];

      for( size_t j=i+1; j<n; ++j ) {
         rhs -= x[j] * A_(pi,j);
      }

      if( std::fabs( A_(pi,i) ) > accuracy ) {
         x[i] = rhs / A_(pi,i);
      }
      else {
         // This will introduce errors in the solution
         reset( x[i] );
         lastPrecision_ = max( lastPrecision_, std::fabs( rhs ) );
      }
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( lastPrecision_ < threshold_ )
         log << "      Solved the linear system using Gaussian elimination.";
      else
         log << pe_YELLOW << "      WARNING: Did not solve the linear system within accuracy. (" << lastPrecision_ << ")" << pe_OLDCOLOR;
   }

   lastIterations_ = 1;

   return lastPrecision_ < threshold_;
}
//*************************************************************************************************

} // namespace solvers

} // namespace pe
