//=================================================================================================
/*!
 *  \file pe/core.h
 *  \brief Header file for the inclusion of the core module of the rigid body physics engine
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

#ifndef _PE_CORE_MODULE_H_
#define _PE_CORE_MODULE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/Algorithm.h>
#include <pe/core/attachable/AttachableCast.h>
#include <pe/core/attachable/Gravity.h>
#include <pe/core/BodyReader.h>
#include <pe/core/BodyBinaryReader.h>
#include <pe/core/BodyBinaryWriter.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/Configuration.h>
#include <pe/core/Constraints.h>
#include <pe/core/contact/Contact.h>
#include <pe/core/ContactType.h>
#include <pe/core/contact/ContactVector.h>
#include <pe/core/domaindecomp/DomainDecomposition.h>
#include <pe/core/domaindecomp/HalfSpace.h>
#include <pe/core/domaindecomp/Intersection.h>
#include <pe/core/domaindecomp/Merging.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/RectilinearGrid.h>
#include <pe/core/Distance.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/GamesSection.h>
#include <pe/core/GeomTools.h>
#include <pe/core/GeomType.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/io/BodySimpleAsciiWriter.h>
#include <pe/core/Link.h>
#include <pe/core/LinkVector.h>
#include <pe/core/Marshalling.h>
#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISystem.h>
#include <pe/core/MPISystemID.h>
#include <pe/core/MPITag.h>
#include <pe/core/Overlap.h>
#include <pe/core/RecvBuffer.h>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/rigidbody/BodyVector.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/BoxVector.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/CapsuleVector.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/CylinderVector.h>
#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/rigidbody/GeomVector.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/PlaneVector.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/SphereVector.h>
#include <pe/core/rigidbody/SuperBody.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/core/rigidbody/UnionVector.h>
#include <pe/core/SendBuffer.h>
#include <pe/core/Serialization.h>
#include <pe/core/SerialSection.h>
#include <pe/core/ScientificSection.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/TimeStep.h>
#include <pe/core/TypeConvertingRecvBuffer.h>
#include <pe/core/TypeConvertingSendBuffer.h>
#include <pe/core/TypeTraits.h>
#include <pe/core/Types.h>
#include <pe/core/UniqueID.h>
#include <pe/core/Visualization.h>
#include <pe/core/World.h>
#include <pe/core/WorldID.h>

#endif
