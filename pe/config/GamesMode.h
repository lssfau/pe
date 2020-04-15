//=================================================================================================
/*!
 *  \file pe/config/GamesMode.h
 *  \brief Configuration of the games mode
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
/*!\brief Compilation switch for the games mode.
 * \ingroup config
 *
 * This compilation switch triggers the games mode of the \b pe physics engine. The games mode
 * is optimized for computational efficiency. Some features of the \b pe physics engine are
 * not relevant for computer games and are simply switched off in order to speed up the overall
 * performance. However, some functions change their behavior during game mode. If this is the
 * case, the function documentation explains these changes.\n
 * During games mode the following features are switched off:
 *
 *  - links between rigid bodies contained in a union for a calculation of contact forces
 *    and torques
 *
 * Possible settings for the games mode switch:
 *  - Deactivated: \b 0
 *  - Activated  : \b 1
 */
#define pe_GAMES_MODE 0
//*************************************************************************************************
