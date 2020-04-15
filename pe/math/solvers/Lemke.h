//=================================================================================================
/*!
 *  \file pe/math/solvers/Lemke.h
 *  \brief The Lemke pivoting algorithm for solving LCPs.
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

#ifndef _PE_MATH_SOLVERS_LEMKE_H_
#define _PE_MATH_SOLVERS_LEMKE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/math/MatrixMxN.h>
#include <pe/math/problems/LCP.h>
#include <pe/math/solvers/Solver.h>
#include <pe/math/VectorN.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

namespace solvers {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The Lemke pivoting algorithm for solving LCPs.
 * \ingroup complementarity_solvers
 *
 * TODO
 */
class Lemke : public Solver
{
public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Lemke();
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   bool solve( LCP& lcp );
   bool solve( LCP& lcp, const VecN& d );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void printTableau( std::ostream& os ) const;
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   bool isComponentwiseNonnegative( const VecN& v ) const;
   bool isComponentwisePositive   ( const VecN& v ) const;
   bool isLexicographicallyLess   ( size_t i1, real f1, size_t i2, real f2 ) const;
   bool isLexicographicallyGreater( size_t i1, real f1, size_t i2, real f2 ) const;
   void pivot                     ( size_t block, size_t drive );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   VectorN<ptrdiff_t> basics_;     //!< TODO
   VectorN<ptrdiff_t> nonbasics_;  //!< TODO
   MatrixMxN<real>    M_;          //!< TODO
   MatrixMxN<real>    Q_;          //!< TODO
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************

} // namespace solvers

} // namespace pe

#endif
