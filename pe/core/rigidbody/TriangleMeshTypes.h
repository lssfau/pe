//=================================================================================================
/*!
 *  \file pe/core/rigidbody/TriangleMeshTypes.h
 *  \brief Header file holding the definitions for the types used with triangle meshes
 *
 *  Copyright (C) 2013-2014 Tobias Scharpff
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


#ifndef TRIANGLEMESHTYPES_H_
#define TRIANGLEMESHTYPES_H_

//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>
#include <pe/math/Vector3.h>
#include <pe/math/Vector2.h>


namespace pe {

//=================================================================================================
//
//  TYPE DEFINITIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Vector of triangle vertices.
 * \ingroup core
 *
 * \todo Review documentation. Check ingroup might be triangleMesh
 */
typedef std::vector<Vec3> Vertices;

//*************************************************************************************************
/*!\brief Vector of triangle normals.
 * \ingroup core
 *
 * \todo Review documentation. Check ingroup might be triangleMesh
 */
typedef std::vector<Vec3> Normals;


//*************************************************************************************************
/*!\brief Vector of a 3-tupel of indices.
 * \ingroup core
 *
 * The indices are used to reference the position of a certain information within another vector
 * to an triangle vertex.
 *
 * \todo Review documentation. Check ingroup might be triangleMesh
 */
typedef std::vector< Vector3<size_t> > IndicesLists;

//*************************************************************************************************
/*!\brief A index vector.
 * \ingroup core
 *
 * The index is used to reference the position of a certain information within another vector
 * to an triangle vertex.
 *
 * \todo Review documentation. Check ingroup might be triangleMesh
 */
typedef std::vector< size_t > IndexList;


//*************************************************************************************************
/*!\brief Vector of texture coordinates.
 * \ingroup core
 *
 * \todo Review documentation. Check ingroup might be triangleMesh
 */
typedef std::vector< Vec2 > TextureCoordinates;

} // namespace pe


#endif /* TRIANGLEMESHTYPES_H_ */
