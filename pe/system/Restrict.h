//=================================================================================================
/*!
 *  \file pe/system/Restrict.h
 *  \brief System settings for the restrict keyword
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

#ifndef _PE_SYSTEM_RESTRICT_H_
#define _PE_SYSTEM_RESTRICT_H_


//=================================================================================================
//
//  RESTRICT SETTINGS
//
//=================================================================================================

#include <pe/config/Restrict.h>




//=================================================================================================
//
//  RESTRICT KEYWORD
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Platform dependent setup of the restrict keyword.
 * \ingroup config
 */
#if pe_USE_RESTRICT

// Intel compiler
#  if defined(__INTEL_COMPILER) || defined(__ICL) || defined(__ICC) || defined(__ECC)
#    define pe_RESTRICT __restrict

// GNU compiler
#  elif defined(__GNUC__)
#    define pe_RESTRICT __restrict

// Microsoft visual studio
#  elif defined(_MSC_VER)
#    define pe_RESTRICT

// All other compilers
#  else
#    define pe_RESTRICT

#  endif
#else
#  define pe_RESTRICT
#endif
/*! \endcond */
//*************************************************************************************************

#endif
