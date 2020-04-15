//=================================================================================================
/*!
 *  \file pe/core/GamesSection.h
 *  \brief Games section for code exclusively executed in games mode
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

#ifndef _PE_CORE_GAMESSECTION_H_
#define _PE_CORE_GAMESSECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/GamesMode.h>


//=================================================================================================
//
//  GAMES SECTION MACRO
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Games section for code exclusively executed in games mode.
 * \ingroup core
 *
 * The \a pe_GAMES_SECTION macro starts a games section that contains code that is only
 * executed in case the \a pe_GAMES_MODE is switched on. This macro has the opposite effect
 * of the \a pe_SCIENTIFIC_SECTION macro, that contains code that is only executed in case
 * the games mode is switched off. The following code demonstrates the use of these two macros:

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
#  define pe_GAMES_SECTION if( true )
#else
#  define pe_GAMES_SECTION if( false )
#endif
//*************************************************************************************************

#endif
