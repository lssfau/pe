//=================================================================================================
/*!
 *  \file src/core/attachable/ForceGenerator.cpp
 *  \brief Interface for force generators
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

#include <algorithm>
#include <ostream>
#include <pe/core/attachable/ForceGenerator.h>
#include <pe/core/MPI.h>
#include <pe/util/Assert.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/logging/DebugSection.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

ForceGenerator::Generators ForceGenerator::generators_;




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the ForceGenerator class.
 *
 * \param type The type of the force generator.
 * \param sid The unique system-specific ID of the force generator.
 * \param visible Specifies if the force generator is visible or not.
 */
ForceGenerator::ForceGenerator( AttachableType type, id_t sid, bool visible )
   : Attachable( type, sid )  // Initialization of the attachable base class
   , visible_  ( visible   )  // Visibility flag
{
   generators_.pushBack( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the ForceGenerator class.
 */
ForceGenerator::~ForceGenerator()
{
   Generators::Iterator pos( std::find( generators_.begin(), generators_.end(), this ) );
   pe_INTERNAL_ASSERT( pos != generators_.end(), "Force generator is not registered" );
   generators_.erase( pos );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\fn void ForceGenerator::setVisible( bool visible )
 * \brief Setting the force generator visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the force generator visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the force generator visible/invisible in all active visualizations.
 */
//*************************************************************************************************




//=================================================================================================
//
//  FORCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Triggers all active force generators.
 *
 * \return void
 */
void ForceGenerator::applyForces()
{
   if( generators_.isEmpty() ) return;

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Activating the force generators";
   }

   for( Generators::Iterator g=generators_.begin(); g!=generators_.end(); ++g ) {
      g->applyForce();
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for force generators.
 * \ingroup force_generator
 *
 * \param os Reference to the output stream.
 * \param fg Reference to a constant force generator object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const ForceGenerator& fg )
{
   os << "--" << pe_BROWN << "FORCE GENERATOR PARAMETERS" << pe_OLDCOLOR
      << "----------------------------------------------------\n";
   fg.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for force generator handles.
 * \ingroup force_generator
 *
 * \param os Reference to the output stream.
 * \param fg Constant force generator handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstForceGeneratorID fg )
{
   os << "--" << pe_BROWN << "FORCE GENERATOR PARAMETERS" << pe_OLDCOLOR
      << "----------------------------------------------------\n";
   fg->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
