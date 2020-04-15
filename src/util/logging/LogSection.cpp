//=================================================================================================
/*!
 *  \file src/util/logging/LogSection.cpp
 *  \brief Source file for the LogSection class
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
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <string>
#include <boost/format.hpp>
#include <pe/util/Assert.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/logging/LogSection.h>
#include <pe/util/SystemClock.h>


namespace pe {

namespace logging {

//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the LogSection class.
 *
 * \param level The level of the log section.
 */
LogSection::LogSection( LogLevel level )
   : level_  ( level )  // The logging level of the log section
   , message_()         // Intermediate buffer for log messages
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the LogSection class.
 */
LogSection::~LogSection()
{
   commit();
}
//*************************************************************************************************




//=================================================================================================
//
//  LOGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Commits the current log message to the log file.
 *
 * \return void
 *
 * This function commits the current log message to the log file. The function is automatically
 * called at the end of a log section, but can also be used manually in order to intermediately
 * commit log messages, for instance in the case of nested log sections.
 */
void LogSection::commit()
{
   // Early exit in case the log message is empty
   if( message_.str().empty() ) return;

   std::string line;
   std::ostringstream oss;

   // Writing the log level information
   switch( level_ )
   {
      case inactive: oss << "[INACTIVE]"; break;
      case error   : oss << "[ERROR   ]"; break;
      case warning : oss << "[WARNING ]"; break;
      case info    : oss << "[INFO    ]"; break;
      case progress: oss << "[PROGRESS]"; break;
      case debug   : oss << "[DEBUG   ]"; break;
      case detail  : oss << "[DETAIL  ]"; break;
      default: pe_INTERNAL_ASSERT( false, "Unknown logging level" ); break;
   }

   // Writing the elapsed time
   const time_t elapsed( theSystemClock()->elapsed() );
   const time_t hours  ( elapsed / time_t( 3600 ) );
   const time_t minutes( ( elapsed % time_t( 3600 ) ) / time_t( 60 ) );
   const time_t seconds( elapsed % time_t( 60 ) );
   oss << boost::format( "[%03d:%02d:%02d] " ) % hours % minutes % seconds;

   // Committing the log message
   std::getline( message_, line );
   oss << line << "\n";

   while( std::getline( message_, line ) ) {
      if( !line.empty() )
         oss << "                      " << line << "\n";
   }

   // Adding an additional spacing line
   if( spacing )
      oss << "\n";

   // Logging the formated log message
   boost::shared_ptr<Logger> logger( Logger::instance() );
   logger->log( oss.str() );

   // Resetting the message buffer
   message_.str( "" );
   message_.clear();
}
//*************************************************************************************************

} // namespace logging

} // namespace pe
