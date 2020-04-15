//=================================================================================================
/*!
 *  \file pe/config/Assertion.h
 *  \brief Configuration of the run time assertion macros
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
/*!\brief Compilation switch for internal assertions.
 * \ingroup config
 *
 * This compilation switch triggers internal assertions, which are used to verify the program
 * itself. The internal assertions can also be deactivated by defining \a NDEBUG during the
 * compilation.
 *
 * Possible settings for the internal assertion switch:
 *  - Deactivated: \b 0
 *  - Activated  : \b 1
 */
#define pe_INTERNAL_ASSERTION 1
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Compilation switch for user assertions.
 * \ingroup config
 *
 * This compilation switch triggers user assertions, which are used to check user specified
 * function parameters and values. The user assertions can also be deactivated by defining
 * \a NDEBUG during the compilation.
 *
 * Possible settings for the user assertion switch:
 *  - Deactivated: \b 0
 *  - Activated  : \b 1
 */
#define pe_USER_ASSERTION 1
//*************************************************************************************************
