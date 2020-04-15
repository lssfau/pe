//=================================================================================================
/*!
 *  \file src/core/ExclusiveSection.cpp
 *  \brief Exclusive section for a single process in a parallel environment
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
#include <pe/core/ExclusiveSection.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

bool ExclusiveSection::active_( false );




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the ExclusiveSection class.
 *
 * \param rank The rank of the process starting the exclusive section.
 * \exception std::runtime_error Nested exclusive sections detected.
 * \exception std::logic_error Invalid exclusive section inside global section.
 * \exception std::invalid_argument Invalid MPI rank for exclusive section.
 */
ExclusiveSection::ExclusiveSection( int rank )
{
   if( active_ )
      throw std::runtime_error( "Nested exclusive sections detected" );

   if( GlobalSection::isActive() )
      throw std::logic_error( "Invalid exclusive section inside global section" );

   if( rank < 0 || rank >= MPISettings::size() )
      throw std::invalid_argument( "Invalid MPI rank for exclusive section" );

   active_ = ( MPISettings::rank() == rank );
}
//*************************************************************************************************

} // namespace pe
