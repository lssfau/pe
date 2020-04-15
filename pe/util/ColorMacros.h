//=================================================================================================
/*!
 *  \file pe/util/ColorMacros.h
 *  \brief Header file for color macros
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

#ifndef _PE_UTIL_COLORMACROS_H_
#define _PE_UTIL_COLORMACROS_H_


//=================================================================================================
//
//  COLOR MACRO SWITCH
//
//=================================================================================================

//! pe color output mode.
/*! This mode triggers the color output macros. */
#define pe_COLOR_OUTPUT 0




//=================================================================================================
//
//  COLOR MACRO DEFINITIONS
//
//=================================================================================================

#if pe_COLOR_OUTPUT

//! Switches the text color to black in case the pe_COLOR_OUTPUT macro is set.
#define pe_BLACK         "\033[0;30m"

//! Switches the text color to red in case the pe_COLOR_OUTPUT macro is set.
#define pe_RED           "\033[0;31m"

//! Switches the text color to green in case the pe_COLOR_OUTPUT macro is set.
#define pe_GREEN         "\033[0;32m"

//! Switches the text color to brown in case the pe_COLOR_OUTPUT macro is set.
#define pe_BROWN         "\033[0;33m"

//! Switches the text color to blue in case the pe_COLOR_OUTPUT macro is set.
#define pe_BLUE          "\033[0;34m"

//! Switches the text color to magenta in case the pe_COLOR_OUTPUT macro is set.
#define pe_MAGENTA       "\033[0;35m"

//! Switches the text color to cyan in case the pe_COLOR_OUTPUT macro is set.
#define pe_CYAN          "\033[0;36m"

//! Switches the text color to white in case the pe_COLOR_OUTPUT macro is set.
#define pe_WHITE         "\033[0;37m"

//! Switches the text color to a light black in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTBLACK    "\033[1;30m"

//! Switches the text color to a light red in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTRED      "\033[1;31m"

//! Switches the text color to a light green in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTGREEN    "\033[1;32m"

//! Switches the text color to yellow in case the pe_COLOR_OUTPUT macro is set.
#define pe_YELLOW        "\033[1;33m"

//! Switches the text color to a light blue in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTBLUE     "\033[1;34m"

//! Switches the text color to a light magenta in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTMAGENTA  "\033[1;35m"

//! Switches the text color to a light cyan in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTCYAN     "\033[1;36m"

//! Switches the text color to a light white in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTWHITE    "\033[1;37m"

//! Switches the text color back to the default color.
#define pe_OLDCOLOR      "\033[0m"

#else

//! Switches the text color to black in case the pe_COLOR_OUTPUT macro is set.
#define pe_BLACK         ""

//! Switches the text color to red in case the pe_COLOR_OUTPUT macro is set.
#define pe_RED           ""

//! Switches the text color to green in case the pe_COLOR_OUTPUT macro is set.
#define pe_GREEN         ""

//! Switches the text color to brown in case the pe_COLOR_OUTPUT macro is set.
#define pe_BROWN         ""

//! Switches the text color to blue in case the pe_COLOR_OUTPUT macro is set.
#define pe_BLUE          ""

//! Switches the text color to magenta in case the pe_COLOR_OUTPUT macro is set.
#define pe_MAGENTA       ""

//! Switches the text color to cyan in case the pe_COLOR_OUTPUT macro is set.
#define pe_CYAN          ""

//! Switches the text color to white in case the pe_COLOR_OUTPUT macro is set.
#define pe_WHITE         ""

//! Switches the text color to a light black in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTBLACK    ""

//! Switches the text color to a light red in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTRED      ""

//! Switches the text color to a light green in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTGREEN    ""

//! Switches the text color to yellow in case the pe_COLOR_OUTPUT macro is set.
#define pe_YELLOW        ""

//! Switches the text color to a light blue in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTBLUE     ""

//! Switches the text color to a light magenta in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTMAGENTA  ""

//! Switches the text color to a light cyan in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTCYAN     ""

//! Switches the text color to a light white in case the pe_COLOR_OUTPUT macro is set.
#define pe_LIGHTWHITE    ""

//! Switches the text color back to the default color.
#define pe_OLDCOLOR      ""

#endif

#endif
