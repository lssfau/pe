//=================================================================================================
/*!
 *  \file pe/math/problems/LSE.h
 *  \brief A data structure for linear systems of equations
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

#ifndef _PE_MATH_PROBLEMS_LSE_H_
#define _PE_MATH_PROBLEMS_LSE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/math/Functions.h>
#include <pe/math/Infinity.h>
#include <pe/math/SparseMatrixMxN.h>
#include <pe/math/VectorN.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief A linear system of equations (LSE) data structure.
 * \ingroup math
 *
 * The LSE class represent a linear system of equations of the form \f$ A \cdot x + b = 0 \f$.
 */
struct LSE
{
public:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline size_t size    ()               const;
   inline void   project ( size_t index );
   inline real   lbound  ( size_t index ) const;
   inline real   ubound  ( size_t index ) const;
   inline real   residual( size_t index ) const;
   inline real   residual()               const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   SMatN A_;  //!< The system matrix \f$ A \f$.
   VecN  b_;  //!< The right-hand side vector \f$ b \f$.
   VecN  x_;  //!< The vector of unknowns \f$ x \f$.
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
/*!\brief Returns the size of the linear system of equations.
 *
 * \return The actual size of the LSE.
 */
inline size_t LSE::size() const
{
   return x_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Projects the unknown at the given index on the solution range.
 *
 * \param index Access index. The index has to be in the range \f$ [0..size) \f$.
 * \return void
 */
inline void LSE::project( size_t /*index*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the lower bound of the unknown at the given index.
 *
 * \param index Access index. The index has to be in the range \f$ [0..size) \f$.
 * \return void
 */
inline real LSE::lbound( size_t /*index*/ ) const
{
   return -inf;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the upper bound of the unknown at the given index.
 *
 * \param index Access index. The index has to be in the range \f$ [0..size) \f$.
 * \return void
 */
inline real LSE::ubound( size_t /*index*/ ) const
{
   return inf;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the residual of the unknown at the given index.
 *
 * \param index Access index. The index has to be in the range \f$ [0..size) \f$.
 * \return The residual at index \a index.
 */
inline real LSE::residual( size_t index ) const
{
   return ( A_ * x_ )[index] + b_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the maximum norm of the residual of the linear system of equations.
 *
 * \return The maximum norm of the global residual of the LSE.
 */
inline real LSE::residual() const
{
   real rmax( 0 );

   for( size_t i=0; i<size(); ++i ) {
      rmax = max( rmax, std::fabs( residual( i ) ) );
   }

   return rmax;
}
//*************************************************************************************************

} // namespace pe

#endif
