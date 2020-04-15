//=================================================================================================
/*!
 *  \file src/core/GlobalSection.cpp
 *  \brief Global section for the setup of global rigid bodies
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


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

bool GlobalSection::active_( false );




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the GlobalSection class.
 *
 * \param activate Activation flag for the global section.
 * \exception std::runtime_error Nested global sections detected.
 * \exception std::logic_error Invalid global section inside exclusive section.
 */
GlobalSection::GlobalSection( bool activate )
{
   if( active_ )
      throw std::runtime_error( "Nested global sections detected" );

   if( ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid global section inside exclusive section" );

   active_ = activate;
}
//*************************************************************************************************

} // namespace pe
