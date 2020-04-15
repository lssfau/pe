//=================================================================================================
/*!
 *  \file pe/config/Precision.h
 *  \brief Configuration of the floating point precision of the physics engine
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


//*************************************************************************************************
/*!\brief Floating point data type of the physics engine.
 * \ingroup config
 *
 * This type definition offers the possibility to switch the floating point precision of
 * the physics engine between float, double and long double.
 *
 * Valid types for the \a real floating point type: <a>float, double, long double</a>
 */
#ifndef pe_PRECISION
typedef double real;
#else
typedef pe_PRECISION real;
#endif
//*************************************************************************************************
