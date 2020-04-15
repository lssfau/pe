//=================================================================================================
/*!
 *  \file pe/util/constraints/ConstraintTest.h
 *  \brief Constraint wrapper class
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

#ifndef _PE_UTIL_CONSTRAINTS_CONSTRAINTTEST_H_
#define _PE_UTIL_CONSTRAINTS_CONSTRAINTTEST_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Constraint wrapper class.
 * \ingroup constraints
 *
 * This class is used as a wrapper for the instantiation of the specific constraint class
 * templates. It serves the purpose to force the instantiation of either the defined
 * specialization or the undefined basic template during the compilation. In case the
 * compile time condition is met, the type pe::CONSTRAINT_TEST<1> is defined.
 */
template< int > struct CONSTRAINT_TEST {};
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
