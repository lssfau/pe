//=================================================================================================
/*!
 *  \file src/core/rigidbody/TriangleMeshBase.cpp
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/TriangleMeshBase.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>
#include <pe/core/Materials.h>
#include <pe/core/Thresholds.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the TriangleMeshBase class.
 *
 * \param sid Unique system-specific ID for the triangle mesh.
 * \param uid User-specific ID for the triangle mesh.
 * \param gpos Global geometric center of the triangle mesh.
 * \param vertices List of all vertices on the surface of the triangle mesh, which has its center of mess at (0,0,0).
 * \param faceIndices List of indices which assign three elements of vertices to one triangle
 * \param material The material of the triangle mesh.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 *
 * \todo Review documentation.
 */
TriangleMeshBase::TriangleMeshBase( id_t sid, id_t uid, const Vec3& gpos,
                                    const Vertices& vertices, const IndicesLists& faceIndices,
                                    MaterialID material, bool visible )
   : GeomPrimitive( triangleMeshType, true, visible, sid, uid, material )  // Initializing the base object
   , verticesOriginal_( vertices )
   , faceIndices_ ( faceIndices )
   , verticesCurrent_ ( vertices )
   , lastGpos_( -gpos ) // make sure that it is not valid at this point
   , lastQ_ ()          // should be ok as lastGpos_ is invalid anyway
{
   // Setting the center of the triangle mesh which is also the center of mass
   gpos_ = gpos;

   // Calculating the triangle mesh mass
   mass_ = calcMass( verticesOriginal_, faceIndices_, Material::getDensity( material ) );
   invMass_ = real(1) / mass_;

   // Calculating the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding triangle mesh
   TriangleMeshBase::calcBoundingBox();
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the TriangleMeshBase class.
 */
TriangleMeshBase::~TriangleMeshBase()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \return Returns the support point in world-frame coordinates in direction a\ d.
 *
 * \note This is a brute force 0(n) implementation.
 * \todo Review documentation.
 */
Vec3 TriangleMeshBase::support( const Vec3& d ) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised");

   //transform the search direction form world-frame coordinates to body-frame
   //triangle mesh cache doesn't have to be up to date, so this might save time
   Vec3T  dT         = trans(vectorFromWFtoBF(d)); //the transposed search direction in BF

   double bestScalar = -1.0; //scalar product must be positive or the point is not further in the search direction
   size_t bestIndex  = verticesOriginal_.size();

   for(size_t v=0; v < verticesOriginal_.size(); ++v) {
      double scalar = dT * verticesOriginal_[v];
      if(scalar > bestScalar) {
         bestScalar = scalar;
         bestIndex = v;
      }
   }

   pe_INTERNAL_ASSERT( bestIndex < verticesOriginal_.size(), "No support point could be estimated" );

   return pointFromBFtoWF(verticesOriginal_[bestIndex]);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the axis-aligned bounding box of the triangle mesh.
 *
 * \return void
 *
 * This function updates the axis-aligned bounding box of the triangle mesh primitive according to the
 * current position and orientation of the triangle mesh. Note that the bounding box is increased in
 * all dimensions by pe::contactThreshold to guarantee that rigid bodies in close proximity
 * of the box are also considered during the collision detection process.
 */
void TriangleMeshBase::calcBoundingBox()
{
   updateCache();

   aabb_[0] = gpos_[0];
   aabb_[1] = gpos_[1];
   aabb_[2] = gpos_[2]; //std::numeric_limits<real>::max();
   aabb_[3] = gpos_[0];
   aabb_[4] = gpos_[1];
   aabb_[5] = gpos_[2]; //std::numeric_limits<real>::min(); //is actually a positive value close to 0

   for(Vertices::const_iterator v=verticesCurrent_.begin(); v != verticesCurrent_.end(); ++v) {
      aabb_[0] = (*v)[0] < aabb_[0] ? (*v)[0] : aabb_[0];
      aabb_[1] = (*v)[1] < aabb_[1] ? (*v)[1] : aabb_[1];
      aabb_[2] = (*v)[2] < aabb_[2] ? (*v)[2] : aabb_[2];
      aabb_[3] = (*v)[0] > aabb_[3] ? (*v)[0] : aabb_[3];
      aabb_[4] = (*v)[1] > aabb_[4] ? (*v)[1] : aabb_[4];
      aabb_[5] = (*v)[2] > aabb_[5] ? (*v)[2] : aabb_[5];
   }

   aabb_[0] -= contactThreshold;
   aabb_[1] -= contactThreshold;
   aabb_[2] -= contactThreshold;
   aabb_[3] += contactThreshold;
   aabb_[4] += contactThreshold;
   aabb_[5] += contactThreshold;

   pe_INTERNAL_ASSERT( aabb_.isValid()        , "Invalid bounding box detected" );
   pe_INTERNAL_ASSERT( aabb_.contains( gpos_ ), "Invalid bounding box detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the moment of inertia in reference to the body frame of the triangle mesh.
 *
 * \return void
 */
void TriangleMeshBase::calcInertia()
{
   updateCache();

   //www.melax.com/volint
   real volume ( 0 );  // technically this variable accumulates the volume times 6
   Vec3 diag(0,0,0);   // accumulate matrix main diagonal integrals [x*x, y*y, z*z]
   Vec3 offd(0,0,0);   // accumulate matrix off-diagonal  integrals [y*z, x*z, x*y]

   for (size_t i = 0; i < faceIndices_.size(); ++i) {
      const Vec3& a = verticesCurrent_[faceIndices_[i][0]];
      const Vec3& b = verticesCurrent_[faceIndices_[i][1]];
      const Vec3& c = verticesCurrent_[faceIndices_[i][2]];

      Mat3 A(a-gpos_, b-gpos_, c-gpos_);
      //original code works with row inizialised matrix
      //http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=954&sid=a27635305168c5f31c15ad954723e7b1#p954
      A.transpose();
      //float3x3 A(vertices[tris[i][0]]-com,vertices[tris[i][1]]-com,vertices[tris[i][2]]-com);  // matrix trick for volume calc by taking determinant

      // matrix trick for volume calc by taking determinant
      real d = (trans(a) * ( b % c )); // vol of tiny parallelapiped= d * dr * ds * dt (the 3 partials of my tetral triple integral equasion)
      volume += d; // add vol of current tetra (note it could be negative - that's ok we need that sometimes)

      for(size_t j=0;j<3;j++)
      {
         int j1=(j+1)%3;
         int j2=(j+2)%3;
         diag[j] += (A(0,j)*A(1,j) + A(1,j)*A(2,j) + A(2,j)*A(0,j) +
                  A(0,j)*A(0,j) + A(1,j)*A(1,j) + A(2,j)*A(2,j)  ) *d; // divide by 60.0f later;
         offd[j] += (A(0,j1)*A(1,j2)  + A(1,j1)*A(2,j2)  + A(2,j1)*A(0,j2)  +
                  A(0,j1)*A(2,j2)  + A(1,j1)*A(0,j2)  + A(2,j1)*A(1,j2)  +
                  A(0,j1)*A(0,j2)*2+ A(1,j1)*A(1,j2)*2+ A(2,j1)*A(2,j2)*2 ) *d; // divide by 120.0f later
      }

   }
   diag /= volume*(real( 60.0) / real(6.0));  // divide by total volume (vol/6) since density=1/volume
   offd /= volume*(real(120.0) / real(6.0));

   //x =^ [0]; y =^ [1]; z =^ [2]
   I_[0] = diag[1]+diag[2]; I_[1] = -offd[2];        I_[2] = -offd[1];
   I_[3] = -offd[2];        I_[4] = diag[0]+diag[2]; I_[5] = -offd[0];
   I_[6] = -offd[1];        I_[7] = -offd[0];        I_[8] = diag[0]+diag[1];

   //scale with total mass as the calculation is for mass 1
   I_ *= mass_;

   Iinv_ = I_.getInverse();
}
//*************************************************************************************************


//=================================================================================================
//
//  CACHING UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Updates verticesCurrent_ data.
 *
 * \return void
 */
void TriangleMeshBase::updateCache() {
   if(lastGpos_ == gpos_ && lastQ_ == q_) {
      return; //cache is up to date
   }

   for(size_t v=0; v < verticesCurrent_.size(); ++v) {
      //rotation
      verticesCurrent_[v] = q_.rotate(verticesOriginal_[v]);
      //translation
      verticesCurrent_[v] += gpos_;
   }

   //Backup the current state
   lastGpos_ = gpos_;
   lastQ_    = q_;
}
//**********************************************************************************************

} /* namespace pe */
