//=================================================================================================
/*!
 *  \file pe/math/Expression.h
 *  \brief Header file for the Expression base class
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

#ifndef _PE_MATH_EXPRESSION_H_
#define _PE_MATH_EXPRESSION_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for all expression templates.
 * \ingroup math
 *
 * The Expression class is the base class for all expression templates. All classes, that
 * represent a mathematical operation and that are used within the expression template
 * environment of the physics engine have to derive from this class in order to qualify
 * as expression template. Only in case a class is derived from the Expression base class,
 * the IsExpression type trait recognizes the class as valid expression template.
 */
struct Expression
{};
//*************************************************************************************************

} // namespace pe

#endif
