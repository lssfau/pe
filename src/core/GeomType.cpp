//=================================================================================================
/*!
 *  \file src/core/GeomType.cpp
 *  \brief Source file for the rigid body geometry types
 *
 *  Copyright (C) 2011 Tobias Preclik
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

#include <iostream>
#include <pe/core/GeomType.h>
#include <pe/core/MPI.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/util/Assert.h>


namespace pe {

//*************************************************************************************************
/*!\brief Global output operator for GeomType.
 *
 * \param os Reference to the output stream.
 * \param type The GeomType to be put into the stream.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, GeomType type )
{
   switch( type )
   {
      case sphereType:       os << "sphere";   break;
      case boxType:          os << "box";      break;
      case capsuleType:      os << "capsule";  break;
      case cylinderType:     os << "cylinder"; break;
      case planeType:        os << "plane";    break;
      case triangleMeshType: os << "mesh";     break;
      case unionType:        os << "union";    break;
      default: pe_INTERNAL_ASSERT( false, "Unknown geometry type" ); break;
   }

   return os;
}
//*************************************************************************************************

//!\cond pe_INTERNAL

//*************************************************************************************************
/*!\brief Returns the geometry type of a sphere.
 * \return The geometry type of a sphere.
 */
template<>
GeomType geomType<Sphere>() {
   return sphereType;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the geometry type of a box.
 * \return The geometry type of a box.
 */
template<>
GeomType geomType<Box>() {
   return boxType;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the geometry type of a capsule.
 * \return The geometry type of a capsule.
 */
template<>
GeomType geomType<Capsule>() {
   return capsuleType;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the geometry type of a cylinder.
 * \return The geometry type of a cylinder.
 */
template<>
GeomType geomType<Cylinder>() {
   return cylinderType;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the geometry type of a plane.
 * \return The geometry type of a plane.
 */
template<>
GeomType geomType<Plane>() {
   return planeType;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the geometry type of a mesh.
 * \return The geometry type of a mesh.
 */
template<>
GeomType geomType<TriangleMesh>() {
   return triangleMeshType;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the geometry type of a union.
 * \return The geometry type of a union.
 */
template<>
GeomType geomType<Union>() {
   return unionType;
}
//*************************************************************************************************

//!\endcond

} // namespace pe
