//=================================================================================================
/*!
 *  \file pe/core/GeomType.h
 *  \brief Header file for the rigid body geometry types
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

#ifndef _PE_CORE_GEOMTYPE_H_
#define _PE_CORE_GEOMTYPE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/util/Byte.h>


namespace pe {

//=================================================================================================
//
//  RIGID BODY GEOMETRY TYPES
//
//=================================================================================================

//*************************************************************************************************
//! Geometry types of the rigid bodies.
enum GeomType {
   sphereType       = 1,  //!< Code for Sphere geometries.
   boxType          = 2,  //!< Code for Box geometries.
   capsuleType      = 3,  //!< Code for Capsule geometries.
   cylinderType     = 4,  //!< Code for Cylinder geometries.
   planeType        = 5,  //!< Code for Plane geometries.
   triangleMeshType = 6,  //!< Code for TriangleMesh geometries.
   unionType        = 7   //!< Code for Union geometries.
};
//*************************************************************************************************




//=================================================================================================
//
//  GEOMETRY TYPE UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Geometry type utility functions */
//@{
std::ostream& operator<<( std::ostream& os, GeomType type );

/*!\brief Returns the geometry type of the template parameter.
 * \return The geometry type of the template parameter.
 */
template<typename T>
GeomType geomType();
//@}
//*************************************************************************************************




//=================================================================================================
//
//  GEOMETRY TYPE MARSHALLING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Geometry type marshalling functions */
//@{
template< typename Buffer > inline void marshal( Buffer& buffer, const GeomType& type );
template< typename Buffer > inline void unmarshal( Buffer& buffer, GeomType& type );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const GeomType& type ) {
   buffer << static_cast<byte>( type );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, GeomType& type ) {
   byte tmp;
   buffer >> tmp;
   type = static_cast<GeomType>( tmp );
}
//*************************************************************************************************

} // namespace pe

#endif
