//=================================================================================================
/*!
 *  \file pe/util/Unused.h
 *  \brief Header file for a macro silencing unused warnings.
 *
 *  Copyright (C) 2011 Tobias Preclik
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

#ifndef _PE_UTIL_UNUSED_H_
#define _PE_UTIL_UNUSED_H_


//=================================================================================================
//
//  UNUSED MACRO
//
//=================================================================================================

/*! This macro silences unused variables/parameter warnings in a portable way. */
#define UNUSED(expr) do { (void)(expr); } while (0)

#endif
