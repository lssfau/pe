//=================================================================================================
/*!
 *  \file pe/core/CollisionSystem.h
 *  \brief Header file for the collision system of the simulation world
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

#ifndef _PE_CORE_COLLISIONSYSTEM_H_
#define _PE_CORE_COLLISIONSYSTEM_H_


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Collision system of the rigid body simulation world.
 * \ingroup core
 *
 * The CollisionSystem class represents the collision system of the rigid body simulation world,
 * which is a combination of all collision related algorithms. The collision system provides all
 * functionality for the coarse collision detection (see pe::pe_COARSE_COLLISION_DETECTOR), the
 * fine collision detection (see pe::pe_FINE_COLLISION_DETECTOR), the batch generation process
 * (see pe::pe_BATCH_GENERATOR), and the collision response (see pe::pe_CONSTRAINT_SOLVER). The
 * CollisionSystem class is closely coupled to the World class and is implemented in terms of
 * the world's body storage and contact storage.\n
 * The CollisionSystem needs to be instantiated with a single template template argument of the
 * following structure:

   \code
   template< template<typename> class CD                      // Type of the coarse collision detection algorithm
           , typename FD                                      // Type of the fine collision detection algorithm
           , template<typename> class BG                      // Type of the batch generation algorithm
           , template<typename,typename,typename> class CR >  // Type of the collision response algorithm
   struct Configuration;
   \endcode

 * The given template template argument must be take four template arguments, where the first
 * argument resembles the currently selected coarse collision detection algorithm, the second
 * represents the selected fine collision detection algorithm, the third is the selected
 * batch generation algorithm, and the fourth represents the selected collision response
 * algorithm. The attempt to instantiate the CollisionSystem with any other type of template
 * argument results in a compile time error!
 */
template< typename C >  // Type of the configuration
class CollisionSystem;
//*************************************************************************************************

} // namespace pe




//*************************************************************************************************
// Includes
//*************************************************************************************************

// WARNING: Include specializations before default implementation so that instantiations of the
// CollisionSystem can be detected easily.
#include <pe/core/collisionsystem/DEMSolverObsolete.h>
#include <pe/core/collisionsystem/DEMSolver.h>
#include <pe/core/collisionsystem/FFDSolver.h>
#include <pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h>
#include <pe/core/collisionsystem/OpenCLSolver.h>
#include <pe/core/collisionsystem/Default.h>




namespace pe {

//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Type of the selected coarse collision detector.
 * \ingroup core
 *
 * The type of the coarse collision detection algorithm is selected by the setting of the
 * pe::pe_COARSE_COLLISION_DETECTOR macro.
 */
typedef CollisionSystem<Config>::CoarseDetector  CoarseDetector;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Type of the selected batch generation algorithm.
 * \ingroup core
 *
 * The type of the batch generation algorithm is selected by the setting of the
 * pe_BATCH_GENERATOR macro.
 */
typedef CollisionSystem<Config>::BatchGenerator  BatchGenerator;
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Type of the selected contact solver.
 * \ingroup core
 *
 * The type of the contact solver is selected by the setting of the pe_CONSTRAINT_SOLVER macro.
 */
typedef CollisionSystem<Config>::ContactSolver  ContactSolver;
//*************************************************************************************************




//=================================================================================================
//
//  COLLISION SYSTEM SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns a handle to the \b pe collision system.
 * \ingroup core
 *
 * \return Handle to the collision system
 *
 * Via this function it is possible to gain access to the collision system of the \b pe
 * physics engine. This function returns a handle to the collision system that can be used
 * to configure the collision system, to acquire the current settings, or to directly access
 * the active coarse collision detector, the batch generation system, and the contact solver.
 */
inline CollisionSystemID theCollisionSystem()
{
   return CollisionSystem<Config>::instance();
}
//*************************************************************************************************

} // namespace pe

#endif
