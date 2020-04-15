//=================================================================================================
/*!
 *  \file src/core/domaindecomp/Domain.cpp
 *  \brief Source file for the domain of the rigid body simulation world
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#include <pe/core/Configuration.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/RigidBody.h>


namespace pe {

//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================


//*************************************************************************************************
/*!\brief Tests if a rigid body is required in the local domain of the simulation world.
 *
 * \param o The rigid body to test.
 * \return \a true if the body is required by the local domain, \a false if not.
 *
 * This function tests whether the given rigid body is required by the local domain of the
 * simulation world. In case the rigid body itself or any directly attached rigid body is at
 * least partially contained, the function returns \a true, otherwise it returns \a false.
 * Note that for a non-parallel simulation, this function always returns \a true.
 */
bool Domain::requires( ConstBodyID o ) const
{
#if HAVE_MPI
   // Testing if the given body is contained in the local domain
   if( intersectsWith( o ) ) return true;

   // Testing if any of the attached bodies is contained in the local domain
   typedef RigidBody::ConstAttachedBodyIterator  ConstAttachedBodyIterator;
   const ConstAttachedBodyIterator bbegin( o->beginAttachedBodies() );
   const ConstAttachedBodyIterator bend  ( o->endAttachedBodies()   );

   for( ConstAttachedBodyIterator attachedBody=bbegin; attachedBody!=bend; ++attachedBody ) {
      if( intersectsWith( *attachedBody ) ) return true;
   }

   return false;
#else
   UNUSED( o );
   return true;
#endif
}
//*************************************************************************************************

} // namespace pe
