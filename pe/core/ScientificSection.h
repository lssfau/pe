//=================================================================================================
/*!
 *  \file pe/core/ScientificSection.h
 *  \brief Scientific section for code exclusively executed in scientific mode
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

#ifndef _PE_CORE_SCIENTIFICSECTION_H_
#define _PE_CORE_SCIENTIFICSECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/GamesMode.h>


//=================================================================================================
//
//  SCIENTIFIC SECTION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Scientific section for code exclusively executed in scientific mode.
 * \ingroup core
 *
 * The \a pe_SCIENTIFIC_SECTION macro starts a scientific section that contains code that
 * is only executed in case the \a pe_GAMES_MODE is switched off. This macro has the opposite
 * effect of the \a pe_GAMES_SECTION macro, that contains code that is only executed in case
 * the games mode is switched on. The following code demonstrates the use of these two macros:

   \code
   pe_GAMES_SECTION {
      // Code that is exclusively executed while in games mode
   }

   pe_SIENTIFIC_SECTION {
      // Code that is exclusively executed while not in games mode
   }

   // Code that is executed both in games mode and scientific mode
   \endcode
 */
#if pe_GAMES_MODE
#  define pe_SCIENTIFIC_SECTION if( false )
#else
#  define pe_SCIENTIFIC_SECTION if( true )
#endif
//*************************************************************************************************

#endif
