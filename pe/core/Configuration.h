//=================================================================================================
/*!
 *  \file pe/core/Configuration.h
 *  \brief Header file for the configuration
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2013-2014 Tobias Scharpff
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

#ifndef _PE_CORE_CONFIGURATION_H_
#define _PE_CORE_CONFIGURATION_H_

//*************************************************************************************************
// Forward Declarations
//*************************************************************************************************

namespace pe {

class Attachable;
class BallJoint;
class Box;
class Capsule;
class Contact;
class Cylinder;
class FixedJoint;
class Gravity;
class HingeJoint;
class Joint;
class Plane;
class Process;
class RigidBody;
class SliderJoint;
class Sphere;
class Spring;
class TriangleMesh;
class Union;

} // namespace pe




//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/system/Collisions.h>
#include <pe/core/configuration/Default.h>
#include <pe/core/configuration/DEMSolver.h>
#include <pe/core/configuration/DEMSolverObsolete.h>
#include <pe/core/configuration/FFDSolver.h>
#include <pe/core/configuration/HardContactSemiImplicitTimesteppingSolvers.h>




//*************************************************************************************************
// Type Definitions
//*************************************************************************************************

namespace pe {

//*************************************************************************************************
/*!\brief The actual configuration of the rigid body physics engine.
 * \ingroup core
 *
 * The actual configuration is the concrete instance of the Configuration class for the
 * current configuration of the physics engine.
 */
typedef Configuration< pe_COARSE_COLLISION_DETECTOR  // Type of the coarse collision detection algorithm
                     , pe_FINE_COLLISION_DETECTOR    // Type of the fine collision detection algorithm
                     , pe_BATCH_GENERATOR            // Type of the batch generation algorithm
                     , pe_CONSTRAINT_SOLVER          // Type of the collision response algorithm
                     >::Config  Config;              // Type of the active configuration
//*************************************************************************************************

} // namespace pe

#endif
