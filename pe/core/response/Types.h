//=================================================================================================
/*!
 *  \file pe/core/response/Types.h
 *  \brief Header file for the constraint solver types
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_RESPONSE_TYPES_H_
#define _PE_CORE_RESPONSE_TYPES_H_


namespace pe {

namespace response {

//=================================================================================================
//
//  ::pe::response NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template<typename,typename,typename> class BoxFrictionSolver;
template<typename,typename,typename> class ConeFrictionSolver;
template<typename,typename,typename> class DEMSolver;
template<typename,typename,typename> class DEMSolverObsolete;
template<typename,typename,typename> class FFDSolver;
template<typename,typename,typename> class FrictionlessSolver;
template<typename,typename,typename> class HardContactSemiImplicitTimesteppingSolvers;
template<typename,typename,typename> class OpenCLSolver;
template<typename,typename,typename> class PolyhedralFrictionSolver;

} // namespace response

} // namespace pe

#endif
