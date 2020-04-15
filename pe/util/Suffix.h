//=================================================================================================
/*!
 *  \file pe/util/Suffix.h
 *  \brief Header file for compile time constraints
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

#ifndef _PE_UTIL_SUFFIX_H_
#define _PE_UTIL_SUFFIX_H_


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Helper macro for macro concatenation.
 * \ingroup util
 *
 * The following code was borrowed from the Boost C++ framework (www.boost.org). This piece of
 * macro magic joins the two arguments together, even when one of the arguments is itself a
 * macro (see 16.3.1 in C++ standard).  The key is that macro expansion of macro arguments does
 * not occur in pe_DO_JOIN2 but does in pe_DO_JOIN.
 */
#define pe_JOIN( X, Y ) pe_DO_JOIN( X, Y )
#define pe_DO_JOIN( X, Y ) pe_DO_JOIN2(X,Y)
#define pe_DO_JOIN2( X, Y ) X##Y
/*! \endcond */
//*************************************************************************************************

#endif
