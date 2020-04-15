//=================================================================================================
/*!
 *  \file pe/config/Logging.h
 *  \brief Configuration of the logging functionality
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
/*!\brief Setting of the logging level.
 * \ingroup config
 *
 * This value specifies the logging level of the \b pe logging functionality. Depending on this
 * setting, more or less informations will be written to the log file(s). The following logging
 * levels can be selected:
 *
 *  - \a inactive: Completely deactivates the logging functionality, i.e., no log file(s) will be
 *                 written. Since this setting can immensely complicate error correction, it is
 *                 not recommended to use this setting!
 *  - \a error   : Only (severe) errors are written to the log file(s).
 *  - \a warning : Extends the \a error setting by warning messages.
 *  - \a info    : Extends the \a warning setting by additional informative messages (default).
 *  - \a progress: Extends the \a info setting by progress informations.
 *  - \a debug   : Extends the \a progress setting by debug information.
 *  - \a detail  : Extends the \a debug setting by very fine grained detail information.
 */
const LogLevel loglevel = info;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adding an additional spacing line between two log messages.
 * \ingroup config
 *
 * This setting gives the opportunity to add an additional spacing line between two log messages
 * to improve readability of log files. If set to \a true, each log message will be appended with
 * an additional empty line. If set to \a false, no line will be appended.
 */
const bool spacing = false;
//*************************************************************************************************
