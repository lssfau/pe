//=================================================================================================
/*!
 *  \file pe/core/rigidbody/TriangleMeshBase.h
 *  \brief Base class for the triangle mesh geometry
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

#ifndef TRIANGLEMESHBASE_H_
#define TRIANGLEMESHBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>
#include <pe/core/Types.h>
#include <pe/system/Precision.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for the triangle mesh geometry.
 * \ingroup triangleMesh
 *
 * The TriangleMeshBase class represents the base class for the triangle mesh geometry.
 * It provides the basic functionality of a triangle mesh. For a full description of the
 * triangle mesh geometry, see the TriangleMesh class description.
 */
class TriangleMeshBase : public pe::GeomPrimitive
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit TriangleMeshBase( id_t sid, id_t uid, const Vec3& gpos,
                              const Vertices& vertices, const IndicesLists& faceIndices,
                              MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~TriangleMeshBase() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vertices&      getWFVertices();
   inline const Vertices&      getBFVertices() const;
   inline const IndicesLists&  getFaceIndices() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
          virtual Vec3 support( const Vec3& d ) const;
   inline virtual Vec3 supportContactThreshold( const Vec3& d ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Volume, mass and density functions**********************************************************
   /*!\name Volume, mass and density functions */
   //@{
   static inline real calcVolume( const Vertices& vertices, const IndicesLists& faceIndices );
   static inline real calcMass( const Vertices& vertices, const IndicesLists& faceIndices, real density );
   static inline real calcDensity( const Vertices& vertices, const IndicesLists& faceIndices, real mass );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual void calcBoundingBox();  // Calculation of the axis-aligned bounding box
   void calcInertia();      // Calculation of the moment of inertia
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   const Vertices     verticesOriginal_; //!< List of all vertices on the surface of the triangle mesh, which has its center of mess at (0,0,0).
   const IndicesLists faceIndices_;      //!< List of indices which assign three elements of \a verticesOriginal_ / \a verticesCurrent_ to one triangle

   Vertices           verticesCurrent_;  //!< List of all current vertices positions of the triangle mesh
   Vec3               lastGpos_;         //!< Position of the triangle mesh at the point the cache was updates the last time
   Quat               lastQ_;            //!< Rotation of the triangle mesh at the point the cache was updates the last time
   //@}
   //**********************************************************************************************

   //**Caching utility functions*******************************************************************
   /*!\name Caching utility functions */
   //@{
   void updateCache();
   //@}
   //**********************************************************************************************

};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the list of all vertices which are on the surface of the triangle mesh,
 * at the very moment in time, in world frame coordinates.
 *
 * \return The list of all vertices which are on the surface of the triangle mesh in world frame coordinates.
 *
 * \todo Review documentation.
 */
inline const Vertices& TriangleMeshBase::getWFVertices()
{
   updateCache();
   return verticesCurrent_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the list of all vertices which are on the surface of the triangle mesh,
 * in body frame coordinates.
 *
 * \return The list of all vertices which are on the surface of the triangle mesh in body frame coordinates.
 *
 * \todo Review documentation.
 */
inline const Vertices& TriangleMeshBase::getBFVertices() const
{
   return verticesOriginal_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the list of indices which assign three elements of vertices_ to one triangle.
 *
 * \return The list of indices which assign three elements of vertices_ to one triangle.
 *
 * \todo Review documentation.
 */
inline const IndicesLists& TriangleMeshBase::getFaceIndices() const
{
   return faceIndices_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates
 * \return The support point in world-frame coordinates in direction a\ d extended by a vector in
 *         direction \a d of length \a pe::contactThreshold.
 *
 * \todo Review documentation.
 */
inline Vec3 TriangleMeshBase::supportContactThreshold( const Vec3& d ) const
{
   return support(d) + d*contactThreshold;
}
//*************************************************************************************************




//=================================================================================================
//
//  VOLUME, MASS AND DENSITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculates the volume of a triangle mesh for given vertex list and corresponding index list.
 *
 * \param vertices The list of vertices forming the triangle mesh.
 * \param faceIndices List of indices which assign three elements of vertices to one triangle.
 * \return The volume of the triangle mesh.
 */
inline real TriangleMeshBase::calcVolume(  const Vertices& vertices, const IndicesLists& faceIndices )
{
   static const real sixth = real(1.0) / real(6.0);
   //Calculate centre of mass and volume
   //http://stackoverflow.com/questions/2083771/a-method-to-calculate-the-centre-of-mass-from-a-stl-stereo-lithography-file
   real totalVolume ( 0.0 );
   real currentVolume ( 0.0 );

   for (size_t i = 0; i < faceIndices.size(); ++i) {
      const Vec3& a = vertices[faceIndices[i][0]];
      const Vec3& b = vertices[faceIndices[i][1]];
      const Vec3& c = vertices[faceIndices[i][2]];

      //http://mathworld.wolfram.com/Tetrahedron.html
      currentVolume = (trans(a) * ( b % c )); //*sixth out of loop
      totalVolume += currentVolume;
   }

   totalVolume *= sixth;

   return totalVolume;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the mass of a triangle mesh for given vertex list, corresponding index list and density.
 *
 * \param vertices The list of vertices forming the triangle mesh.
 * \param faceIndices List of indices which assign three elements of vertices to one triangle.
 * \param density The density of the triangle mesh.
 * \return The total mass of the triangle mesh.
 */
inline real TriangleMeshBase::calcMass( const Vertices& vertices, const IndicesLists& faceIndices, real density )
{
   return calcVolume(vertices, faceIndices) * density;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the density of a triangle mesh for given vertex list, corresponding index list and mass.
 *
 * \param vertices The list of vertices forming the triangle mesh.
 * \param faceIndices List of indices which assign three elements of vertices to one triangle.
 * \param mass The total mass of the triangle mesh.
 * \return The density of the triangle mesh.
 */
inline real TriangleMeshBase::calcDensity( const Vertices& vertices, const IndicesLists& faceIndices, real mass)
{
   return mass / calcVolume(vertices, faceIndices);
}
//*************************************************************************************************

} /* namespace pe */

#endif /* TRIANGLEMESHBASE_H_ */
