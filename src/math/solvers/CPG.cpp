//*************************************************************************************************
/*!
 *  \file src/math/solvers/CPG.cpp
 *  \brief Implementation of the conjugate projected gradient algorithm
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

#include <pe/math/solvers/CPG.h>


namespace pe {

namespace solvers {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for the CPG class.
 */
CPG::CPG()
   : r_()         // TODO
   , w_()         // TODO
   , p_()         // TODO
   , activity_()  // TODO
{}
//*************************************************************************************************




//=================================================================================================
//
//  EXPLICIT TEMPLATE INSTANTIATIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
#if !defined(_MSC_VER)
template bool CPG::solve<LCP>( LCP& );
template bool CPG::solve<BoxLCP>( BoxLCP& );
template bool CPG::solve<ContactLCP>( ContactLCP& );
#endif
/*! \endcond */
//*************************************************************************************************

} // namespace solvers

} // namespace pe
