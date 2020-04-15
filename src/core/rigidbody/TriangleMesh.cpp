//=================================================================================================
/*!
 *  \file src/core/rigidbody/TriangleMesh.cpp
 *  \brief Source file for the Triangle class
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************
//#include <pe/math/Vector3.h>
//#include <pe/math/VectorN.h>
//#include <pe/math/Quaternion.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <cstring>
#include <sstream>
#include <map>
#include <utility>
#include <algorithm>
#include <limits>

#include <pe/core/Materials.h>
#include <pe/core/MPI.h>
#include <pe/core/Visualization.h>
#include <pe/math/Quaternion.h>
#include <pe/system/VerboseMode.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/ConvexHull.h>
#include <pe/util/Limits.h>

#include <pe/core/BodyManager.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/GlobalSection.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/core/GeomTools.h>

#include <pe/util/Logging.h>

#include <pe/core/STLMeshLoader.h>
#include <pe/core/OBJMeshLoader.h>

#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/core/rigidbody/TriangleMeshTypes.h>

#include <pe/povray.h>
#include <pe/povray/Texture.h>
#if HAVE_IRRLICHT
#  include <pe/irrlicht.h>
#  include <irrlicht/irrlicht.h>
#endif


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor for the TriangleMesh class
 *
 * \param sid Unique system-specific ID for the triangle mesh.
 * \param uid User-specific ID for the triangle mesh.
 * \param gpos Global geometric center of the triangle mesh.
 * \param vertices The list of vertices that build the surface
 * \param faceIndices The list of indices which assign three elements of vertices to one triangle
 * \param faceNormals A list of face normals if given
 * \param vertexNormals The list of vertex normals if given (for rendering only)
 * \param normalIndices The list of indices which assign three elements of vertexNormals to one triangle (for rendering only)
 * \param texturCoordinates The list of textur coordinates if given (for rendering only)
 * \param texturIndices The list of indices which assign three elements of texturCoordinates to one triangle (for rendering only)
 * \param material The material of the triangle mesh.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 * \param convex Flag that can be set if it is sure that the triangle mesh is convex
 *
 * \todo Review documentation.
 */
TriangleMesh::TriangleMesh( id_t sid, id_t uid, const Vec3& gpos,
         const Vertices& vertices, const IndicesLists& faceIndices,
         const Normals& faceNormals,
         const Normals& vertexNormals, const IndicesLists normalIndices,
         const TextureCoordinates& texturCoordinates, const IndicesLists& texturIndices,
         MaterialID material, bool visible, bool convex )
   : Parent( sid, uid, gpos, vertices, faceIndices, material, visible ) // Initializing the base object
   , faceNormals_( faceNormals )
   , vertexNormals_( vertexNormals )
   , normalIndices_( normalIndices )
   , textureCoordinates_( texturCoordinates )
   , textureIndices_( texturIndices )
   , convex_(convex)
   , smallMeshLimit_(512) //good value at least for containsRelPoint()
   , renderSmooth_(false)
#if HAVE_IRRLICHT
   , cachedIrrlichtBuffer_(NULL)
#endif
{
   aabbBottomFrontLeftIndices_[0] = 0;
   aabbBottomFrontLeftIndices_[1] = 0;
   aabbBottomFrontLeftIndices_[2] = 0;
   aabbTopBackRightIndices_[0] = 0;
   aabbTopBackRightIndices_[1] = 0;
   aabbTopBackRightIndices_[2] = 0;

   initHalfEdge();
   updatePOVcacheString();
#if HAVE_IRRLICHT
   updateIrrlichtCacheBuffer();
#endif

   // Registering the triangle mesh for visualization
   Visualization::add( this );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the TriangleMesh class
 *
 * \param sid Unique system-specific ID for the triangle mesh.
 * \param uid User-specific ID for the triangle mesh.
 * \param gpos Global geometric center of the triangle mesh.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the triangle mesh's body frame in the global world frame.
 * \param vertices The list of vertices that build the surface
 * \param faceIndices List of indices which assign three elements of vertices_ to one triangle
 * \param faceNormals A list of face normals if given
 * \param vertexNormals The list of vertex normals if given (for rendering only)
 * \param normalIndices The list of indices which assign three elements of vertexNormals to one triangle (for rendering only)
 * \param textureCoordinates The list of textur coordinates if given (for rendering only)
 * \param textureIndices The list of indices which assign three elements of texturCoordinates to one triangle (for rendering only)
 * \param material The material of the triangle mesh.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 * \param fixed \a true to fix the triangle mesh, \a false to unfix it.
 * \param convex Flag that can be set if it is sure that the triangle mesh is convex
 *
 * \todo Review documentation.
 */
TriangleMesh::TriangleMesh( id_t sid, id_t uid, const Vec3& gpos,
                       const Vec3& rpos, const Quat& q,
                       const Vertices& vertices, const IndicesLists& faceIndices,
                       const Normals& faceNormals,
                       const Normals& vertexNormals, const IndicesLists normalIndices,
                       const TextureCoordinates& texturCoordinates, const IndicesLists& texturIndices,
                       MaterialID material, bool visible, bool fixed,  bool convex )
   : Parent( sid, uid, gpos, vertices, faceIndices, material, visible ) // Initializing the base object
   , faceNormals_( faceNormals )
   , vertexNormals_( vertexNormals )
   , normalIndices_( normalIndices )
   , textureCoordinates_( texturCoordinates )
   , textureIndices_( texturIndices )
   , convex_(convex)
   , smallMeshLimit_(512) //good value at least for containsRelPoint()
   , renderSmooth_(false)
#if HAVE_IRRLICHT
   , cachedIrrlichtBuffer_(NULL)
#endif
{
   aabbBottomFrontLeftIndices_[0] = 0;
   aabbBottomFrontLeftIndices_[1] = 0;
   aabbBottomFrontLeftIndices_[2] = 0;
   aabbTopBackRightIndices_[0] = 0;
   aabbTopBackRightIndices_[1] = 0;
   aabbTopBackRightIndices_[2] = 0;

   // Initializing the instantiated triangle mesh
   remote_ = true;                   // Setting the remote flag
   rpos_   = rpos;                   // Setting the relative position
   q_      = q;                      // Setting the orientation
   R_      = q_.toRotationMatrix();  // Setting the rotation matrix

   if( fixed ) {
      fixed_   = true;               // Setting the fixed flag
      invMass_ = real(0);            // Setting the inverse total mass
      Iinv_    = real(0);            // Setting the inverse moment of inertia
   }

   initHalfEdge();
   updatePOVcacheString();
#if HAVE_IRRLICHT
   updateIrrlichtCacheBuffer();
#endif

   // Registering the triangle mesh for visualization
   Visualization::add( this );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the TriangleMesh class.
 */
TriangleMesh::~TriangleMesh()
{
   // Deregistering the triangle mesh from visualization
   Visualization::remove( this );


#if HAVE_IRRLICHT
   //destroy irrlicht cache buffer
   if(cachedIrrlichtBuffer_ != NULL) {
      cachedIrrlichtBuffer_->drop();
   }
#endif
}
//*************************************************************************************************




//=================================================================================================
//
//  INITIALIZATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/* \todo Document.
 */
/* deprecated code, might be still useful later on
void TriangleMesh::initSTL( const char* const file,
                            Vertices& vertices, IndicesLists& faceIndices,
                            Normals& faceNormals)
{
   // Extracting the vertices and normals from the given STL file
   STLMeshLoader stl;
   stl.load( file, vertices, faceNormals );

   if(vertices.size() % 3 != 0) {
      std::stringstream error;
      error << "TriangleMesh::initSTL: Corrupted input file, number of vertices is not a multiple of 3.\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::invalid_argument(error.str());
   }

   if(vertices.size() / 3 != faceNormals.size()) {
      std::stringstream error;
      error << "TriangleMesh::initSTL: Corrupted input file, number of vertices is not appropriate to the number of faces.\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::invalid_argument(error.str());
   }

   //Initialising the faceIndices array
   faceIndices.reserve(faceNormals.size());
   size_t index = 0;
   for(size_t f = 0; f < faceNormals.size(); ++f, index+=3) {
      Vector3<size_t> indices(index, index+1, index+2);
      faceIndices.push_back(indices);
   }

   return;

}
*/
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Loads all triangle mesh data from a given OBJ-file and stores all date in the provided containers
 *
 * \param file Location of the OBJ-file that should be parsed.
 * \param vertices Container for the vertex data.
 * \param faceIndices Container for the list of indices which assign three elements of \a vertices to one triangle.
 * \param faceNormals Container for the face normal data.
 * \param vertexNormals Container for the vertex normal data.
 * \param normalIndices Container for the list of indices which assign three elements of \a vertexNormals to one triangle.
 * \param textureCoordinates Container for the texture coordinate data.
 * \param textureIndices Container for the list of indices which assign three elements of \a textureCoordinates to one triangle.
 * \param clockwise \a true if the triangles described in \a file are ordered clockwise, otherwise \a false, which is more common.
 * \param lefthanded \a true if the coordinates used in the OBJ-file are from a left-handed coordinate system, \a false if the coordinate system is right-handed (pe default)
 * \return void
 *
 * \todo Review documentation.
 */
void TriangleMesh::initOBJ(const std::string file,
                  Vertices& vertices, IndicesLists& faceIndices,
                  Normals& faceNormals,
                  Normals& vertexNormals, IndicesLists normalIndices,
                  TextureCoordinates& textureCoordinates, IndicesLists& textureIndices,
                  bool clockwise, bool lefthanded)
{
   OBJMeshLoader parser;

   parser.load(file, vertices, faceIndices, vertexNormals, normalIndices, textureCoordinates, textureIndices, clockwise, lefthanded);

   if(faceIndices.size()<4){
      std::stringstream error;
      error << "TriangleMesh::initOBJ: TriangleMashes with less than 4 (" << faceIndices.size() << ") faces not allowed.\n";
      pe_LOG_INFO_SECTION(log) {
         log << error.str();
      }
      throw std::invalid_argument(error.str());
   }

   //triangles.reserve(faceIndices_.size());
   faceNormals.reserve(faceIndices.size());

   for(size_t f = 0; f < faceIndices.size(); ++f) {
      Vec3& a = vertices[faceIndices[f][0]];
      Vec3& b = vertices[faceIndices[f][1]];
      Vec3& c = vertices[faceIndices[f][2]];

      //calculate face normal
      Vec3 ab = b-a;
      Vec3 ac = c-a;
      Vec3 n  = ab % ac;
      n.normalize();
      faceNormals.push_back(n);

      //check if face normal points in the same direction as the vertices normals
      bool inverted = false;
      if(normalIndices.size() > 0) {
         real cosVec1 = trans(n) * vertexNormals[normalIndices[f][0]];
         real cosVec2 = trans(n) * vertexNormals[normalIndices[f][1]];
         real cosVec3 = trans(n) * vertexNormals[normalIndices[f][2]];

         if(cosVec1 < 0 && cosVec2 < 0 && cosVec3 < 0 && !inverted) {
            std::stringstream error;
            error << "TriangleMesh::initOBJ: Vertex order seams to be inverted.\n";
            pe_LOG_INFO_SECTION(log) {
               log << error.str();
            }
            inverted = true;
         }
         else if (cosVec1 < 0 || cosVec2 < 0 || cosVec3 < 0) {
            std::stringstream error;
            error << "TriangleMesh::initOBJ: At least one vertex normal does not match face normal direction.\n";
            pe_LOG_INFO_SECTION(log) {
               log << error.str();
            }
         }
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Initializes the pseudo half edge data structure
 *
 * The virtual neighbors have only one strict criteria. If one of the virtual neighbors is a local
 * minimum than, the other is part of a plane with another surface normal.
 * If there is no local minimum the chosen virtual neighbor might be any point of the triangle mesh!
 *
 * \todo Review documentation.
 */
void TriangleMesh::initHalfEdge() {
   using namespace std;

   //Mapping each vertex to an edge
   //This reference may be set multiple times. As it doesn't matter which edge
   //is assigned to the vertex as long as it contains the vertex. There is no such thing as the right edge.
   vertexEdge_.resize(verticesOriginal_.size(), faceIndices_.size() * 3);
   //faceIndices_.size()*3 is not a valid index for a edge

   //Each triangle consist of three counterclockwise ordered "half edges"
   edgeEdge_.resize(faceIndices_.size() * 3, faceIndices_.size() * 3);
   //faceIndices_.size()*3 is not a valid index for a edge

   //Mapping vertex->virtualNeighbor
   vertexVNeighbor_.resize(verticesOriginal_.size(), verticesOriginal_.size());
   //verticesOriginal_.size() is not a valid index for a virtual neighbor


   typedef map< pair< size_t, size_t >, size_t > EdgeMap;
   EdgeMap existingEdges;

   //iterate over all faces aka triangles
   //first run: set ups the existingEdges map
   for(size_t t=0; t < faceIndices_.size(); ++t) {
      size_t v0 = faceIndices_[t][0];
      size_t v1 = faceIndices_[t][1];
      size_t v2 = faceIndices_[t][2];

      //adding new edge->pair-edge references to the map
      existingEdges[make_pair(v1, v0)] = 3*t + 0;
      existingEdges[make_pair(v2, v1)] = 3*t + 1;
      existingEdges[make_pair(v0, v2)] = 3*t + 2;
   }

   //iterate over all faces aka triangles
   //second run: set edge->edge mapping
   //set vertex->edge
   for(size_t t=0; t < faceIndices_.size(); ++t) {
      size_t v0 = faceIndices_[t][0];
      size_t v1 = faceIndices_[t][1];
      size_t v2 = faceIndices_[t][2];

      //set the vertex->edge reference
      vertexEdge_[v0] = 3*t + 0; //id of the edge v0->v1
      vertexEdge_[v1] = 3*t + 1; //id of the edge v1->v2
      vertexEdge_[v2] = 3*t + 2; //id of the edge v2->v0

      EdgeMap::const_iterator end = existingEdges.end();
      EdgeMap::const_iterator pos;

      //read the edge->pair-edge references
      //if there is a hole in the mesh we might have a problem
      if((pos = existingEdges.find(make_pair(v0, v1))) != end) {
         edgeEdge_[3*t + 0] = pos->second;
      }
      if((pos = existingEdges.find(make_pair(v1, v2))) != end) {
         edgeEdge_[3*t + 1] = pos->second;
      }
      if((pos = existingEdges.find(make_pair(v2, v0))) != end) {
         edgeEdge_[3*t + 2] = pos->second;
      }
   }

   // Determine for each vertex v a virtual neighbor vertex, which is not in the same plane as one
   // of the faces associated with the vertex v - this guarantees that the hill climbing algorithm
   // makes progress even if a vertex v is surrounded by coplanar faces only.
   for( size_t v = 0; v < vertexVNeighbor_.size(); ++v ) {
      size_t neighborIndex = 0;

      pe_INTERNAL_ASSERT( vertexEdge_[v] < 3*faceIndices_.size(), "invalid vertex edge entry" );

      size_t faceIndex = vertexEdge_[v] / 3;

      // Start with a random point and then try all other if necessary
      size_t rndIndex  = rand<size_t>( 0, verticesOriginal_.size() - 1 );

      // Test all vertices starting from a random one if it is a suitable virtual neighbor vertex
      // of v. In the worst case (almost all vertices are part of a plane) this loop takes linear
      // time. If the object is numerically degenerate then it fails to find a virtual neighbor
      // vertex with the criterions specified above.
      for( size_t n = 0; n < verticesOriginal_.size(); ++n ) {
         neighborIndex = (n + rndIndex) % verticesOriginal_.size();

         // The virtual edge must not be loops
         if( neighborIndex == v )
            continue;

         // Points on the same face are already real neighbors
         if(    faceIndices_[faceIndex][0] == neighborIndex
             || faceIndices_[faceIndex][1] == neighborIndex
             || faceIndices_[faceIndex][2] == neighborIndex ) {
            continue;
         }

         if( vertexEdge_[neighborIndex] == faceIndices_.size()*3 ) {
            stringstream error;
            error << "TriangleMesh::initHalfEdge: TriangleMesh(sid="<< this->getID()<<") vertex with index="<<
                     neighborIndex << " is not part of the surface\n";
            pe_LOG_INFO_SECTION(log) {
               log << error.str();
            }
            continue;
         }

         // Do not accept candidate point neighborIndex if it is located in the same plane as the triangle associated with vertex v
         if( std::abs( trans( verticesOriginal_[neighborIndex] - verticesOriginal_[v] )
                          * ( ( verticesOriginal_[faceIndices_[faceIndex][1]] - verticesOriginal_[faceIndices_[faceIndex][0]] )
                              % ( verticesOriginal_[faceIndices_[faceIndex][2]] - verticesOriginal_[faceIndices_[faceIndex][0]] ) ) )
                        < Limits<real>::fpuAccuracy() )
            continue;

         break;
      }

      // Make last candidate point the virtual neighbor vertex - in the worst case this point
      // is the point itself or on the same plane but at least this will not destroy things.
      vertexVNeighbor_[v] = neighborIndex;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the triangle mesh visible/invisible in all active visualizations.
 *
 * \param visible \a true to make the triangle mesh visible, \a false to make it invisible.
 * \return void
 *
 * This function makes the triangle mesh visible/invisible in all active visualizations.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 * process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 * all rigid bodies after local changes, the World::synchronize() function should be used to
 * update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 * neglected and overwritten by the settings of the rigid body on its local process!
 */
void TriangleMesh::setVisible( bool visible )
{
   if( visible ^ visible_ ) {
      visible_ = visible;
      Visualization::changeVisibility( this );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Enables/Disables smooth rendering for the triangle mesh in all active visualizations.
 *
 * \param smooth \a true to make the rendering of the triangle mesh smooth, \a false non smooth rendering.
 * \return void
 *
 * This function enable/disables smooth rendering for the triangle mesh in all active visualizations.
 * Smooth rendering is only possible if vertex normals are given.
 * \note be aware of the problem described at http://wiki.povray.org/content/Knowledgebase:The_Shadow_Line_Artifact
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 * process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 * all rigid bodies after local changes, the World::synchronize() function should be used to
 * update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 * neglected and overwritten by the settings of the rigid body on its local process!
 *
 * \todo Review documentation.
 */
void TriangleMesh::setRenderSmooth( bool smooth) 
{
   if( smooth ^ renderSmooth_ ) {
      renderSmooth_ = smooth;
      updatePOVcacheString();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the triangle mesh.
 *
 * \param px The x-component of the global position.
 * \param py The y-component of the global position.
 * \param pz The z-component of the global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global triangle mesh inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a triangle mesh contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::setPosition( real px, real py, real pz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global triangle mesh inside an exclusive section" );

   gpos_ = Vec3( px, py, pz );

   calcBoundingBox();    // Updating the axis-aligned bounding box of the triangle mesh
   wake();               // Waking the triangle mesh from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global position of the triangle mesh.
 *
 * \param gpos The global position.
 * \return void
 * \exception std::logic_error Invalid translation of a global triangle mesh inside an exclusive section.
 *
 * \b Note:
 * - Setting the position of a triangle mesh contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this may cause an invalidation of links contained in the
 *   union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the position change must be applied on all processes. It is not allowed
 *   to change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::setPosition( const Vec3& gpos )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global triangle mesh inside an exclusive section" );

   gpos_ = gpos;

   TriangleMesh::calcBoundingBox();    // Updating the axis-aligned bounding box of the triangle mesh
   wake();               // Waking the triangle mesh from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the triangle mesh.
 *
 * \param r The value for the real part.
 * \param i The value for the first imaginary part.
 * \param j The value for the second imaginary part.
 * \param k The value for the third imaginary part.
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a triangle mesh contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this changes the union and may cause an invalidation of
 *   links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not allowed
 *   to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::setOrientation( real r, real i, real j, real k )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   q_ = Quat( r, i, j, k );     // Setting the orientation of the triangle mesh
   R_ = q_.toRotationMatrix();  // Updating the rotation of the triangle mesh

   TriangleMesh::calcBoundingBox();  // Updating the axis-aligned bounding box of the triangle mesh
   wake();             // Waking the triangle mesh from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global orientation of the triangle mesh.
 *
 * \param q The global orientation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * \b Note:
 * - Setting the orientation of a triangle mesh contained in a union changes the mass distribution and
 *   geometry of the union. Therefore this changes the union and may cause an invalidation of
 *   links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the orientation change must be applied on all processes. It is not allowed
 *   to change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::setOrientation( const Quat& q )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   q_ = q;                      // Setting the orientation of the triangle mesh
   R_ = q_.toRotationMatrix();  // Updating the rotation of the triangle mesh

   TriangleMesh::calcBoundingBox();  // Updating the axis-aligned bounding box of the triangle mesh
   wake();             // Waking the triangle mesh from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the center of mass of the triangle mesh by the displacement vector
 * \brief (\a dx,\a dy,\a dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 * \exception std::logic_error Invalid translation of a global triangle mesh inside an exclusive section.
 *
 * \b Note:
 * - Translating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void TriangleMesh::translate( real dx, real dy, real dz )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global triangle mesh inside an exclusive section" );

   gpos_[0] += dx;
   gpos_[1] += dy;
   gpos_[2] += dz;

   calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the rigid body from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the center of mass of the triangle mesh by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 * \exception std::logic_error Invalid translation of a global triangle mesh inside an exclusive section.
 *
 * \b Note:
 * - Translating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this may cause an invalidation of links contained in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside a
 *   pe_GLOBAL_SECTION) the translation must be applied on all processes. It is not allowed to
 *   change the position from within a pe_EXCLUSIVE_SECTION. The attempt to do this results in
 *   a \a std::logic_error.
 */
void TriangleMesh::translate( const Vec3& dp )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid translation of a global triangle mesh inside an exclusive section" );

   gpos_ += dp;

   calcBoundingBox();    // Updating the axis-aligned bounding box
   wake();               // Waking the triangle mesh from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around the global rotation axis (x,y,z) by the rotation angle \a angle.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * Changing the orientation/rotation of the triangle mesh. The triangle mesh is rotated around its center of mass
 * around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotate( real x, real y, real z, real angle )
{
   rotate( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around the specified global rotation axis by the rotation
 * \brief angle \a angle.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * Changing the orientation/rotation of the triangle mesh. The triangle mesh is rotated around its center of mass
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotate( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   q_ = Quat( axis, angle ) * q_;  // Updating the orientation of the triangle mesh
   R_ = q_.toRotationMatrix();     // Updating the rotation of the triangle mesh

   calcBoundingBox();  // Updating the axis-aligned bounding box of the triangle mesh
   wake();             // Waking the triangle mesh from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh by the Euler angles \a xangle, \a yangle and \a zangle.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * Changing the orientation/rotation of the triangle mesh. The triangle mesh is rotated around its center of mass
 * by the Euler angles \a xangle, \a yangle and \a zangle (all in radian measure). The rotations
 * are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotate( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   // Updating the orientation of the triangle mesh
   q_.rotateX( xangle );  // Rotation around the x-axis
   q_.rotateY( yangle );  // Rotation around the y-axis
   q_.rotateZ( zangle );  // Rotation around the z-axis

   R_ = q_.toRotationMatrix();  // Updating the rotation of the triangle mesh

   calcBoundingBox();  // Updating the axis-aligned bounding triangle mesh of the triangle mesh
   wake();             // Waking the triangle mesh from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh by the Euler angles \a euler.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * Changing the orientation/rotation of the triangle mesh. The triangle mesh is rotated around its center of mass
 * by the Euler angles \a euler (all components in radian measure). The rotations are applied
 * in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotate( const Vec3& euler )
{
   rotate( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh by the quaternion \a dq.
 *
 * \param dq The quaternion for the rotation.
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * Changing the orientation/rotation of the triangle mesh. The triangle mesh is rotated around its center of mass
 * by the quaternion \a dq. \n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotate( const Quat& dq )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   q_ = dq * q_;                // Updating the orientation of the triangle mesh
   R_ = q_.toRotationMatrix();  // Updating the rotation of the triangle mesh

   calcBoundingBox();  // Updating the axis-aligned bounding box of the triangle mesh
   wake();             // Waking the triangle mesh from sleep mode
   signalRotation();   // Signaling the change of orientation to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * This function rotates the triangle mesh around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the triangle mesh. The triangle mesh is rotated
 * around the given axis \a (x,y,z) by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around the origin of the global world frame.
 *
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * This function rotates the triangle mesh around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the triangle mesh. The triangle mesh is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotateAroundOrigin( const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   const Quat dq( axis, angle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the triangle mesh
   q_    = dq * q_;                // Updating the orientation of the triangle mesh
   R_    = q_.toRotationMatrix();  // Updating the rotation of the triangle mesh

   calcBoundingBox();    // Updating the axis-aligned bounding box of the triangle mesh
   wake();               // Waking the triangle mesh from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * This function rotates the triangle mesh around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the triangle mesh. The triangle mesh is rotated
 * by the Euler angles \a xangle, \a yangle and \a zangle (all components in radian measure).
 * The rotations are applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   const Quat dq( xangle, yangle, zangle );

   gpos_ = dq.rotate( gpos_ );     // Updating the global position of the triangle mesh
   q_    = dq * q_;                // Updating the orientation of the triangle mesh
   R_    = q_.toRotationMatrix();  // Updating the rotation of the triangle mesh

   calcBoundingBox();    // Updating the axis-aligned bounding box of the triangle mesh
   wake();               // Waking the triangle mesh from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * This function rotates the triangle mesh around the origin of the global world frame and changes
 * both the global position and the orientation/rotation of the triangle mesh. The triangle mesh is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are
 * applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * This function rotates the triangle mesh around the given global coordiante \a point and changes
 * both the global position and the orientation/rotation of the triangle mesh. The triangle mesh is rotated
 * around the given axis \a axis by \a angle degrees (radian measure).\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   const Quat dq( axis, angle );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the triangle mesh
   q_    = dq * q_;                  // Updating the orientation of the triangle mesh
   R_    = q_.toRotationMatrix();    // Updating the rotation of the triangle mesh

   calcBoundingBox();    // Updating the axis-aligned bounding box of the triangle mesh
   wake();               // Waking the triangle mesh from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the triangle mesh around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 * \exception std::logic_error Invalid rotation of a global triangle mesh inside an exclusive section.
 *
 * This function rotates the triangle mesh around the given global coordinate \a point and changes
 * both the global position and the orientation/rotation of the triangle mesh. The triangle mesh is rotated
 * by the Euler angles \a euler (all components in radian measure). The rotations are
 * applied in the order x, y, and z.\n
 *
 * \b Note:
 * - Rotating a triangle mesh contained in a union changes the mass distribution and geometry of the
 *   union. Therefore this changes the union and may cause an invalidation of links contained
 *   in the union.
 * - In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) triangle mesh on one
 *   process may invalidate the settings of the triangle mesh on another process. In order to synchronize
 *   all rigid bodies after local changes, the World::synchronize() function should be used to
 *   update remote rigid bodies accordingly. Note that any changes on remote rigid bodies are
 *   neglected and overwritten by the settings of the rigid body on its local process!
 * - In a <b>MPI parallel simulation</b>, if the triangle mesh is global (i.e. if it was created inside
 *   a pe_GLOBAL_SECTION) the rotation must be applied on all processes. It is not allowed to
 *   change the orientation from within a pe_EXCLUSIVE_SECTION. The attempt to do this results
 *   in a \a std::logic_error.
 */
void TriangleMesh::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid rotation of a global triangle mesh inside an exclusive section" );

   const Quat dq( euler );
   const Vec3 dp( gpos_ - point );

   gpos_ = point + dq.rotate( dp );  // Updating the global position of the triangle mesh
   q_    = dq * q_;                  // Updating the orientation of the triangle mesh
   R_    = q_.toRotationMatrix();    // Updating the rotation of the triangle mesh

   calcBoundingBox();    // Updating the axis-aligned bounding box of the triangle mesh
   wake();               // Waking the triangle mesh from sleep mode
   signalTranslation();  // Signaling the position change to the superordinate body
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the triangle mesh.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies inside the triangle mesh, \a false if not.
 */
bool TriangleMesh::containsRelPoint( real px, real py, real pz ) const
{
   return containsRelPoint(Vec3(px, py, pz));
}
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Compare to std::pair<real, size_t> and returns true if the left hand side real is small than the right hand side
 * Used in \a containsRelPoint
 * \todo Review documentation.
 */
struct TriangleMesh::LowToHighComp
{
   LowToHighComp() {}
   
   bool operator()(std::pair<real, size_t>& left, std::pair<real, size_t>& right)
   {
      return left.first > right.first;
   }
};
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies inside the triangle mesh.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies inside the triangle mesh, \a false if not.
 */


bool TriangleMesh::containsRelPoint( const Vec3& rpos ) const
{
   //rpos is center (0, 0, 0)
   if(rpos.sqrLength() == 0.0) return true;

   if(size() <= smallMeshLimit_) { //limit 512 seams to be ok
      //brute force
      for(size_t currentFace = 0; currentFace < size(); ++currentFace ) {
         const Vec3& A = verticesOriginal_[faceIndices_[currentFace][0]];
         const Vec3& B = verticesOriginal_[faceIndices_[currentFace][1]];
         const Vec3& C = verticesOriginal_[faceIndices_[currentFace][2]];

         //early out
         //check if the observed triangle defines a separating plane between mesh and point
         const Vec3 normal = (B - A) % (C - A);
         if (pointInFrontOfPlane(normal, A, rpos)) {
            return false;
         }
      }
      //point is on the back side of all triangles so it is inside the mesh
      return true;
   }
   else {
      //informed search
      static const LowToHighComp compare;

      //find the support point in direction of rpos
      size_t supportIdx = 0;
      Vec3 supportP = supportContactThreshold(rpos.getNormalized(),0, &supportIdx);

      std::vector< std::pair<real, size_t> > trianglesToProcess;
      trianglesToProcess.reserve(8); //2^3 why not :D, enough space for the initial 4 and the next -1 +3
      std::vector< char > workItems(faceIndices_.size(), 0);

      size_t currentFace = vertexEdge_[supportIdx] / 3;

      real sqrDist = (supportP-rpos).sqrLength();

      //add the found face
      trianglesToProcess.push_back(std::make_pair(sqrDist, currentFace));
      workItems[currentFace] = 1;

      //add the three neighboring faces as they are also good guesses
      size_t neigborFace= edgeEdge_[(currentFace*3 +0)] / 3;
      trianglesToProcess.push_back(std::make_pair(sqrDist, neigborFace));
      workItems[neigborFace] = 1;


      neigborFace= edgeEdge_[(currentFace*3 +1)] / 3;
      trianglesToProcess.push_back(std::make_pair(sqrDist, neigborFace));
      workItems[neigborFace] = 1;


      neigborFace= edgeEdge_[(currentFace*3 +2)] / 3;
      trianglesToProcess.push_back(std::make_pair(sqrDist, neigborFace));
      workItems[neigborFace] = 1;


      std::make_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);

      const Vec3T rposT = trans(rpos);

      while(!trianglesToProcess.empty()) {
         currentFace = trianglesToProcess[0].second;
         std::pop_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);
         trianglesToProcess.pop_back();

         const Vec3  O (0.0, 0.0, 0.0);
         const Vec3& A = verticesOriginal_[faceIndices_[currentFace][0]];
         const Vec3& B = verticesOriginal_[faceIndices_[currentFace][1]];
         const Vec3& C = verticesOriginal_[faceIndices_[currentFace][2]];

         //early out
         //check if the observed triangle defines a separating plane between mesh and point
         const Vec3 normal = (B - A) % (C - A);
         if (pointInFrontOfPlane(normal, A, rpos)) {
            return false;
         }

         if(pointInTetrahedron(O, C, B, A, rpos)) {
            return true;
         }


         //add the three neighboring faces if they are not processed or already in the set
         if(trianglesToProcess.size() + 3 > trianglesToProcess.capacity()) {
            size_t newSize = std::min(trianglesToProcess.capacity()*2, size());
            trianglesToProcess.reserve(newSize);
         }

         for(size_t i = 0; i < 3; ++i) {
            neigborFace= edgeEdge_[(currentFace*3 +i)] / 3;
            if( workItems[neigborFace] == 0) {
               workItems[neigborFace] = 1;

               const Vec3& nA = verticesOriginal_[faceIndices_[neigborFace][0]];
               const Vec3& nB = verticesOriginal_[faceIndices_[neigborFace][1]];
               const Vec3& nC = verticesOriginal_[faceIndices_[neigborFace][2]];
               const Vec3 nNormal = (nB - nA) % (nC - nA);

               const real sqrDistA = (nA-rpos).sqrLength();
               const real sqrDistB = (nB-rpos).sqrLength();
               const real sqrDistC = (nC-rpos).sqrLength();

               //if face normal points in the wrong direction skip triangle
               if(! (rposT * nNormal < -Limits<real>::fpuAccuracy()) ) {
                  trianglesToProcess.push_back(std::make_pair(std::min(sqrDistA, std::min(sqrDistB, sqrDistC)), neigborFace));
                  std::push_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);
               }
            }
         }
      }
      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the triangle mesh.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies inside the triangle mesh, \a false if not.
 */
bool TriangleMesh::containsPoint( real px, real py, real pz ) const
{
   return containsRelPoint( pointFromWFtoBF( Vec3(px, py, pz) ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies inside the triangle mesh.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies inside the triangle mesh, \a false if not.
 */
bool TriangleMesh::containsPoint( const Vec3& gpos ) const
{
   return containsRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the triangle mesh.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return \a true if the point lies on the surface of the triangle mesh, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool TriangleMesh::isSurfaceRelPoint( real px, real py, real pz ) const
{
   return isSurfaceRelPoint(Vec3(px, py, pz));
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in body relative coordinates lies on the surface of the triangle mesh.
 *
 * \param rpos The relative coordinate.
 * \return \a true if the point lies on the surface of the triangle mesh, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool TriangleMesh::isSurfaceRelPoint( const Vec3& rpos ) const
{
   return ( std::fabs( getRelDepth(rpos) ) <= surfaceThreshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the triangle mesh.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the point lies on the surface of the triangle mesh, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool TriangleMesh::isSurfacePoint( real px, real py, real pz ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks, whether a point in global coordinates lies on the surface of the triangle mesh.
 *
 * \param gpos The global coordinate.
 * \return \a true if the point lies on the surface of the triangle mesh, \a false if not.
 *
 * The tolerance level of the check is pe::surfaceThreshold.
 */
bool TriangleMesh::isSurfacePoint( const Vec3& gpos ) const
{
   return isSurfaceRelPoint( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the triangle mesh's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the triangle mesh and a negative value, if the point
 * lies outside the triangle mesh. The returned depth is calculated relative to the closest side of the triangle mesh.
 */
real TriangleMesh::getRelDepth( real px, real py, real pz ) const
{
   return getRelDepth(Vec3(px, py, pz));
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the triangle mesh's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the triangle mesh and a negative value, if the point
 * lies outside the triangle mesh. The returned depth is calculated relative to the closest side of the triangle mesh.
 */
real TriangleMesh::getRelDepth( const Vec3& rpos ) const
{
   real sqrDistance = std::numeric_limits<real>::max();
   bool internal = false;

   if(size() <= smallMeshLimit_ || true) { //TODO only brute force version is working right now
      //brute force
      for(size_t currentFace = 0; currentFace < size(); ++currentFace ) {
         const Vec3& A = verticesOriginal_[faceIndices_[currentFace][0]];
         const Vec3& B = verticesOriginal_[faceIndices_[currentFace][1]];
         const Vec3& C = verticesOriginal_[faceIndices_[currentFace][2]];
         const Vec3T normalT = trans((B - A) % (C - A));
         real a;
         real b;
         real c;

         const Vec3 currentVector = rpos - closestPointToTriangle(rpos, A, B, C, a, b, c);
         const real currentSqrDistance = currentVector.sqrLength();


         if(currentSqrDistance < sqrDistance) {
            sqrDistance = currentSqrDistance;
            if((normalT * currentVector) > real(0.0)){
               internal = false;
               //Early out: if point lies outside and projects inside a face
               //this face is the closest face as the TM is convex
               if(a!=real(0.0) && b!=real(0.0) && c!=real(0.0)) {
                  break;
               }
            }
            else {
               internal = true;
            }
         }
      }
   }
   else {
      //informed search
      //TODO the informed search does not give the right value for all internal points
      //for some reason the distance to the z-coordinate of rpos is calculated in all
      //cases whre it doesn't work
      static const LowToHighComp compare;

      static size_t runCount = 0;

      runCount++;

      //find the support point in direction of rpos
      Vec3 supportP;
      size_t supportIdx = 0;
      if(rpos.sqrLength() != 0.0) {
         supportP = supportContactThreshold(rpos.getNormalized(),0, &supportIdx);
      }
      else {
         //rpos is center (0, 0, 0) so choose any vertex as support
         supportP = verticesOriginal_[0];
      }

      std::vector< std::pair<real, size_t> > trianglesToProcess;
      trianglesToProcess.reserve(8); //2^3 why not :D
      std::vector< char > workItems(faceIndices_.size(), 0);

      size_t currentFace = vertexEdge_[supportIdx] / 3;

      //add the found face
      trianglesToProcess.push_back(std::make_pair((supportP-rpos).sqrLength(), currentFace));
      workItems[currentFace] = 1;

      std::make_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);

      real a (0.0);
      real b (0.0);
      real c (0.0);

      //std::cout << runCount << std::endl;

      while(!trianglesToProcess.empty()) {
         currentFace = trianglesToProcess[0].second;
         std::pop_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);
         trianglesToProcess.pop_back();


         const Vec3& A = verticesOriginal_[faceIndices_[currentFace][0]];
         const Vec3& B = verticesOriginal_[faceIndices_[currentFace][1]];
         const Vec3& C = verticesOriginal_[faceIndices_[currentFace][2]];
         const Vec3T normalT = trans((B - A) % (C - A));

         const Vec3 closestePoint = closestPointToTriangle(rpos, A, B, C, a, b, c);
         const Vec3 currentVector = rpos - closestePoint;
         const real currentSqrDistance = currentVector.sqrLength();

         if(currentSqrDistance < sqrDistance) {
            sqrDistance = currentSqrDistance;
            if((normalT * currentVector) > real(0.0)){
               internal = false;
               //Early out: if point lies outside and projects inside a face
               //this face is the closest face as the TM is convex
               if(a!=real(0.0) && b!=real(0.0) && c!=real(0.0)) {
                  break;
               }
            }
            else {
               internal = true;
            }
         }


         //add the three neighboring faces if they are not processed or already in the set

         if(currentSqrDistance > sqrDistance && ((normalT * currentVector) > real(0.0))) {
            continue;
         }

         //check if there is enough memory
         if(trianglesToProcess.size() + 3 > trianglesToProcess.capacity()) {
            size_t newSize = std::min(trianglesToProcess.capacity()*2, size());
            trianglesToProcess.reserve(newSize);
         }

/*       //adding all trinalge aka brute force ...
         for(size_t i = 0; i < 3; ++i) {
            size_t neigborFace= edgeEdge_[(currentFace*3 +i)] / 3;
            if( workItems[neigborFace] == 0) {
               workItems[neigborFace] = 1;
               trianglesToProcess.push_back(std::make_pair(currentSqrDistance, neigborFace));
               std::push_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);
            }
         }
*/

         //Tactic:
         //if the closest point is on a edge add the neighbor triangle at this edge
         //if the closest point is one vertex add both neighbors sharing this vertex
         //if the closest point is a internal point add all neighbors
         //side 0 == A-B
         size_t neigborFace= edgeEdge_[(currentFace*3 + 0)] / 3;
         if(workItems[neigborFace] == 0 ) {
            workItems[neigborFace] = 1;
            //(a&b) | (a&b&c) | (a&!b&!c) | (!a&b&!c)
            //== (a&b) | (a&!c) | (b&!c)
            if(  (a!=real(0.0) && b!=real(0.0))
               ||(a!=real(0.0) && c==real(0.0))
               ||(b!=real(0.0) && c==real(0.0)) ) {
               trianglesToProcess.push_back(std::make_pair(currentSqrDistance, neigborFace));
               std::push_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);
            }
         }

         //side 1 == B-C
         neigborFace= edgeEdge_[(currentFace*3 + 1)] / 3;
         if(workItems[neigborFace] == 0) {
            workItems[neigborFace] = 1;
            //(b&c) | (a&b&c) | (b&!c&!a) | (c&!a&!b)
            //== (b&c) | (b&!a) | (c&!a)
            if(  (b!=real(0.0) && c!=real(0.0))
               ||(b!=real(0.0) && a==real(0.0))
               ||(c!=real(0.0) && a==real(0.0)) ) {
               trianglesToProcess.push_back(std::make_pair(currentSqrDistance, neigborFace));
               std::push_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);
            }
         }

         //side 2 == C-A
         neigborFace= edgeEdge_[(currentFace*3 + 2)] / 3;
         if(workItems[neigborFace] == 0) {
            workItems[neigborFace] = 1;
            //(c&a) | (a&b&c) | (c&!a&!b) | (a&!b&!c)
            //== (c&a) | (c&!b) | (a&!b)
            if(  (c!=real(0.0) && a!=real(0.0))
               ||(c!=real(0.0) && b==real(0.0))
               ||(a!=real(0.0) && b==real(0.0)) ) {
               trianglesToProcess.push_back(std::make_pair(currentSqrDistance, neigborFace));
               std::push_heap(trianglesToProcess.begin(), trianglesToProcess.end(), compare);
            }
         }
      }
   }

   real distance = sqrt(sqrDistance) * (internal ? real(1.0) : real(-1.0));

   return distance;
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
 * of the triangle mesh are also considered during the collision detection process.
 */
void TriangleMesh::calcBoundingBox()
{
   //FEATURE? Eventuell entscheidung für kleine TM einbauen
   //FEATURE? eventuell einfach nur die TriangleMeshBase:support Funktion benutzen
   if(!convex_) {
      //Brute force search for AABB might be a good solution if the mash has few points
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

      //The bounding box is increased by pe::contactThreshold in all dimensions
      aabb_[0] -= contactThreshold;
      aabb_[1] -= contactThreshold;
      aabb_[2] -= contactThreshold;
      aabb_[3] += contactThreshold;
      aabb_[4] += contactThreshold;
      aabb_[5] += contactThreshold;
   }
   else {
      //finding AABB by searching for the support Points in the 6 directions
      Vec3 leftPoint   = supportContactThreshold(Vec3(-1.0,  0.0,  0.0), aabbBottomFrontLeftIndices_[0], &aabbBottomFrontLeftIndices_[0]);
      Vec3 frontPoint  = supportContactThreshold(Vec3( 0.0, -1.0,  0.0), aabbBottomFrontLeftIndices_[1], &aabbBottomFrontLeftIndices_[1]);
      Vec3 bottomPoint = supportContactThreshold(Vec3( 0.0,  0.0, -1.0), aabbBottomFrontLeftIndices_[2], &aabbBottomFrontLeftIndices_[2]);

      Vec3 rightPoint = supportContactThreshold(Vec3(1.0, 0.0, 0.0), aabbTopBackRightIndices_[0], &aabbTopBackRightIndices_[0]);
      Vec3 backPoint  = supportContactThreshold(Vec3(0.0, 1.0, 0.0), aabbTopBackRightIndices_[1], &aabbTopBackRightIndices_[1]);
      Vec3 topPoint   = supportContactThreshold(Vec3(0.0, 0.0, 1.0), aabbTopBackRightIndices_[2], &aabbTopBackRightIndices_[2]);

      aabb_[0] = leftPoint[0];
      aabb_[1] = frontPoint[1];
      aabb_[2] = bottomPoint[2];
      aabb_[3] = rightPoint[0];
      aabb_[4] = backPoint[1];
      aabb_[5] = topPoint[2];
      //The bounding box is increased by pe::contactThreshold in all dimension
      //this is already done in the supportContactThreshold() function
   }

   pe_INTERNAL_ASSERT( aabb_.isValid()        , "Invalid bounding box detected" );
   pe_INTERNAL_ASSERT( aabb_.contains( gpos_ ), "Invalid bounding box detected" );
}
//*************************************************************************************************





//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation update of a subordinate triangle mesh.
 *
 * \param dp Change in the global position of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a translational
 * movement. This movement involves a change in the global position and the axis-aligned
 * bounding box.
 */
void TriangleMesh::update( const Vec3& dp )
{
   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Updating the global position
   gpos_ += dp;

   // Setting the axis-aligned bounding box
   calcBoundingBox();

   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate triangle mesh.
 *
 * \param dq Change in the orientation of the superordinate body.
 * \return void
 *
 * This update function is triggered by the superordinate body in case of a rotational movement.
 * This movement involves a change in the global position, the orientation/rotation and the
 * axis-aligned bounding box of the triangle mesh.
 */
void TriangleMesh::update( const Quat& dq )
{
   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
   pe_INTERNAL_ASSERT( hasSuperBody(), "Invalid superordinate body detected" );

   // Calculating the new global position
   gpos_ = sb_->getPosition() + ( sb_->getRotation() * rpos_ );

   // Calculating the new orientation and rotation
   q_ = dq * q_;
   R_ = q_.toRotationMatrix();

   // Setting the axis-aligned bounding box
   calcBoundingBox();

   // Checking the state of the triangle mesh
   pe_INTERNAL_ASSERT( checkInvariants(), "Invalid triangle mesh state detected" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Checks if the surface of the triangle mesh is closed.
 *
 * \return \a true if the triangle mesh has a closed surface, \a false if not.
 *
 * Checks if the surface of the triangle mesh is closed meaning that there is now hole in it.
 * This check will take O(n) if n is the number of triangles in the mesh.
 *
 * \note This check is not performed during the initialization of the triangle mesh
 *
 * \todo Review documentation.
 */
bool TriangleMesh::hasClosedSurface() const {
   for(IndexList::const_iterator edge = edgeEdge_.begin(); edge != edgeEdge_.end(); ++edge) {
      if((faceIndices_.size() * 3) == (*edge)) {
         return false;
      }
   }
   return true;

   /* deprecated code without use of half edge data structure
    * Checks if the surface of the triangle mesh is closed. This is very compute intensive and may have
    * runtime of O(n^2) where n is the number of triangles of the mesh.
    * For each triangle of the mesh a neighboring triangle to each of the 3 sides is searched within
    * all the other triangles.


   std::cout << " Surface check...";
   bool found  ( false ); //indicates if a neighboring triangle was found
   size_t iA;
   size_t iB;
   size_t iC;

   IndicesLists::const_iterator beginIndices = faceIndices_.begin();
   IndicesLists::const_iterator endIndices   = faceIndices_.end();

   for(IndicesLists::const_iterator f=beginIndices; f != endIndices; ++f) {
      //f is the current triangle to observe
      iA = (*f)[0];
      iB = (*f)[1];
      iC = (*f)[2];

      //Check for edge A-B
      found = false;
      for(IndicesLists::const_iterator n=beginIndices; n != endIndices; ++n) {
         //n is a potential neighbor of f
         if( n == f ) continue;
         //only 3 checks because both we know that the indices are given
         //counterclockwise
         if( (*n)[1] == iA   && (*n)[0] == iB ) { found = true; break; }
         if( (*n)[0] == iA   && (*n)[2] == iB ) { found = true; break; }
         if( (*n)[2] == iA   && (*n)[1] == iB ) { found = true; break; }
      }
      if( !found ) {
         std::cout <<" Surface is not closed (iA=" << iA+1 << ", iB=" << iB+1 << ")!\n";
         return false;
      }


      //Check for edge B-C
      found = false;
      for(IndicesLists::const_iterator n=beginIndices; n != endIndices; ++n) {
         //n is a potential neighbour of f
         if( n == f ) continue;
         //only 3 checks because both we know that the indices are given
         //counterclockwise
         if( (*n)[1] == iB   && (*n)[0] == iC ) { found = true; break; }
         if( (*n)[0] == iB   && (*n)[2] == iC ) { found = true; break; }
         if( (*n)[2] == iB   && (*n)[1] == iC ) { found = true; break; }
      }
      if( !found ) {
         std::cout << " Surface is not closed (iB=" << iB+1 << ", iC=" << iC+1 << ")!\n";
         return false;
      }


      //Check for edge C-A
      found = false;
      for(IndicesLists::const_iterator n=beginIndices; n != endIndices; ++n) {
         //n is a potential neighbour of f
         if( n == f ) continue;
         //only 3 checks because both we know that the indices are given
         //counterclockwise
         if( (*n)[1] == iC   && (*n)[0] == iA ) { found = true; break; }
         if( (*n)[0] == iC   && (*n)[2] == iA ) { found = true; break; }
         if( (*n)[2] == iC   && (*n)[1] == iA ) { found = true; break; }
      }
      if( !found ) {
         std::cout << " Surface is not closed (iC=" << iC+1 << ", iA=" << iA+1 << ")!\n";
         return false;
      }
   }

   std::cout << " Surface is closed!\n";

   return true;
   */
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Estimates the point which is farthest in direction \a d.
 *
 * \param d The normalized search direction in world-frame coordinates.
 * \param start The index where the search should start.
 * \param pointIndex If not NULL, the index of the returned point is set
 * \return Returns the support point in world-frame coordinates in direction a\ d.
 *
 * \todo Review documentation.
 */
Vec3 TriangleMesh::support( const Vec3& d, size_t startIndex = 0, size_t* pointIndex = NULL) const
{
   pe_INTERNAL_ASSERT( d.sqrLength() != 0.0, "Zero length search direction" );
   pe_INTERNAL_ASSERT( 1.0-Limits<real>::fpuAccuracy() <= d.length() && d.length() <= 1.0+Limits<real>::fpuAccuracy(), "Search direction is not normalised");
   pe_INTERNAL_ASSERT( startIndex < verticesOriginal_.size(), "Start index out of bound");

   //FEATURE? Brute force search for small triangle meshes

#ifdef SUPPORT_STATISTIC
   static size_t calls = 0;
   static size_t totalSteps = 0;
          size_t steps = 0;

   calls++;
#endif

   //transform the search direction form world-frame coordinates to body-frame
   //triangle mesh cache doesn't have to be up to date, so this might save time
   Vec3T  dT         = trans(vectorFromWFtoBF(d)); //the transposed search direction in BF, length should be still 1


   size_t lastIndex  = startIndex; //the vertex index last checked
                                   //if this never changes this means the start point is already on the hill

   size_t bestIndex  = startIndex;
   real bestScalar = -1.0; //scalar product must be positive or the point is not further in the search direction

   //start greedy hill climbing
   while (true) {
#ifdef SUPPORT_STATISTIC
      steps++;//vNeighbor will be observed
#endif
      //cache the last visited point
      Vec3   last = verticesOriginal_[lastIndex];

      //Start by checking if the virtual neighbor is further on the "hill"
      size_t currentIndex = vertexVNeighbor_[lastIndex];
      Vec3 current        = verticesOriginal_[currentIndex];

      real scalar = dT * (current - last);
      if( scalar > Limits<real>::accuracy() ) { //virtual neighbor is on the hill
         //the step made is quit big so take this virtual neighbor greedily

         //save last point index
         lastIndex = currentIndex;
         continue;
      }
      else if( scalar > real( 0 ) ) {
         //there might be still a little hill climbing be done
         if(scalar > bestScalar) {
            //virtual neighbor is the best on hill climbing in small steps
            bestScalar = scalar;
            bestIndex = currentIndex;
         }
      }
      else {
         //virtual neighbor is not on the hill
      }

      //search the neighborhood clockwise
      //lastIndex is the point in the middle of the neighborhood
      size_t startEdge    = vertexEdge_[lastIndex];
      size_t lastEdge     = startEdge;
      bool neighborOnHill = false;

      do {
#ifdef SUPPORT_STATISTIC
         steps++;//a neighbor will be observed
#endif
         size_t currentEdge  = edgeEdge_[lastEdge]; //find pair
         size_t currentFace  = currentEdge / 3;     //find the face pair is part of
         size_t currentPos   = currentEdge % 3;     //estimate which of the 3 edges on the face pair is
         currentIndex = faceIndices_[currentFace][currentPos]; //get the vertex-index the pair-edge starts at
         current = verticesOriginal_[currentIndex];

         scalar = dT * (current - last);
         if(scalar > Limits<real>::accuracy()) { //neighbor is on the hill
            //the step made is quit big so take this neighbor greedily

            //save last points index
            lastIndex = currentIndex;

            neighborOnHill = true;
            break;
         }
         else if (scalar > 0.0) {
            //there might be still a little hill climbing be done
            if(scalar > bestScalar) {
               //current neighbor is the best on hill climbing in small steps
               bestScalar = scalar;
               bestIndex = currentIndex;
            }
         }
         else {
            //neighbor is not on the hill
         }

         //estimate the edge which is on the border to the next clockwise face
         //the edge is on the currentFace and the clockwise neighbor to the last checked edge
         lastEdge = currentFace*3 + (currentPos+1)%3;

      } while(lastEdge != startEdge);

      if(neighborOnHill) {
         continue; //neighbor is on the hill go on climbing
      }

      //Check if there could be at least a bit of hill climbing be done
      if(bestScalar > Limits<real>::fpuAccuracy()) {
         bestScalar = -1.0;
         lastIndex = bestIndex;
         last = verticesOriginal_[lastIndex];
      }
      else {
         //no hill climbing at all so this means we are on the hill
         bestIndex = lastIndex;
         break;
      }
   }

#ifdef SUPPORT_STATISTIC
   totalSteps += steps;
   std::cout << "\nend hill climbing with " << steps << " observed points. d=" <<d <<std::endl;
   std::cout << "  startPoint= " << verticesOriginal_[startIndex] << " startIndex=" << startIndex <<std::endl;
   std::cout << "  resultPoint=" << verticesOriginal_[bestIndex] <<" resultIndex=" << bestIndex  <<std::endl;
   std::cout << "  total support calls=" << calls << " average steps=" << static_cast<real>(totalSteps)/static_cast<real>(calls)<< std::endl;

#endif

   if(pointIndex) *pointIndex = bestIndex;
   return pointFromBFtoWF(verticesOriginal_[bestIndex]);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Gives the triangle mesh an alternating red green blue yellow pattern.
 *
 * \return void
 *
 * Assigns texture coordinates to each triangle. The coordinates are chosen in such a way that tow
 * triangles which are next to each other in the memory have different colors (red, green, blue, yellos).
 * The coloredTriangles.png texture file has to be placed in the same folder as the *.pov files.
 *
 * \NOTE This function gives no guarantee that two neighboring triangles have different colors.
 *
 * \todo Review documentation.
 */
void TriangleMesh::setColoredTriangleTexture(povray::WriterID pov)
{
   using namespace pe::povray;

   textureCoordinates_.resize(6);
   textureIndices_.resize(faceIndices_.size());

   textureCoordinates_[0] = Vec2(0.0, 0.0); // unten links
   textureCoordinates_[1] = Vec2(0.5, 0.0); // unten mitte
   textureCoordinates_[2] = Vec2(0.5, 1.0); // oben mitte
   textureCoordinates_[3] = Vec2(0.0, 1.0); // oben links
   textureCoordinates_[4] = Vec2(1.0, 0.0); // unten rechts
   textureCoordinates_[5] = Vec2(1.0, 1.0); // oben rechts

   size_t i=0;
   for(i=0; i < faceIndices_.size()/4; i++) {
      textureIndices_[i*4 +0] = Vector3<size_t>(0, 1, 2); //gruen
      textureIndices_[i*4 +1] = Vector3<size_t>(2, 3, 0); //rot
      textureIndices_[i*4 +2] = Vector3<size_t>(1, 4, 5); //gelb
      textureIndices_[i*4 +3] = Vector3<size_t>(5, 2, 1); //blau
   }

   switch(faceIndices_.size()%4) {
      case 3:  textureIndices_[i*4 +2] = Vector3<size_t>(0, 1, 2); //gruen
      /* no break */
      case 2:  textureIndices_[i*4 +1] = Vector3<size_t>(2, 3, 0); //rot
      /* no break */
      case 1:  textureIndices_[i*4 +0] = Vector3<size_t>(1, 4, 5); //gelb
      /* no break */
      default: break; //alles bunt
   }

   povray::Finish coloredFinish(
      Ambient( real(0.2) )
   );
   PlainTexture coloredTrianglesTexture(
      ImagePigment( png, "coloredTriangles.png", planar, true ) //,
      //coloredFinish
   );

   pov->setTexture( static_cast<ConstTriangleMeshID>(this), coloredTrianglesTexture);
   updatePOVcacheString();
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of a triangle mesh.
 *
 * \param os Reference to the output stream.
 * \param tab Indentation in front of every line of the triangle mesh output.
 * \return void
 *
 * \todo Review documentation.
 */
void TriangleMesh::print( std::ostream& os, const char* tab ) const
{
   using std::setw;

   os << tab << " Triangle mesh " << uid_ << " consisting of " << size() << " triangles\n";

   if( verboseMode ) {
      os << tab << "   Fixed: " << fixed_ << " , sleeping: " << !awake_ << "\n";
   }

   os << tab << "   Total mass        = " << mass_ << "\n"
      << tab << "   Material          = " << Material::getName( material_ ) << "\n"
      << tab << "   Global Position   = " << gpos_ << "\n"
      << tab << "   Relative Position = " << rpos_ << "\n"
      << tab << "   Linear velocity   = " << getLinearVel() << "\n"
      << tab << "   Angular velocity  = " << getAngularVel() << "\n";

   if( verboseMode )
   {
      os << tab << "   Bounding Box      = " << aabb_ << "\n"
         << tab << "   Quaternion        = " << q_ << "\n"
         << tab << "   Rotation matrix   = ( " << setw(9) << R_[0] << " , " << setw(9) << R_[1] << " , " << setw(9) << R_[2] << " )\n"
         << tab << "                       ( " << setw(9) << R_[3] << " , " << setw(9) << R_[4] << " , " << setw(9) << R_[5] << " )\n"
         << tab << "                       ( " << setw(9) << R_[6] << " , " << setw(9) << R_[7] << " , " << setw(9) << R_[8] << " )\n";

      os << std::setiosflags(std::ios::right)
         << tab << "   Moment of inertia = ( " << setw(9) << I_[0] << " , " << setw(9) << I_[1] << " , " << setw(9) << I_[2] << " )\n"
         << tab << "                       ( " << setw(9) << I_[3] << " , " << setw(9) << I_[4] << " , " << setw(9) << I_[5] << " )\n"
         << tab << "                       ( " << setw(9) << I_[6] << " , " << setw(9) << I_[7] << " , " << setw(9) << I_[8] << " )\n"
         << std::resetiosflags(std::ios::right);

      os << tab << "   Contained triangles : \n";
      for(size_t t = 0; t < faceIndices_.size(); ++t) {
         os << tab << "           ";
         os << "(" << verticesOriginal_[faceIndices_[t][0]] << ", " <<
                      verticesOriginal_[faceIndices_[t][1]] << ", " <<
                      verticesOriginal_[faceIndices_[t][2]] << ", normal=" <<
                      faceNormals_[t] << ")\n";
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the triangle mesh as a POV-ray mesh2.
 *
 * \param os Reference to the output stream.
 * \param texture Refefrenc to the texture tht shoud be used.
 * \param euler Rotation corresponding to the PovRay coordinate system
 * \param povPos Global position within the PovRay coordinate system
 * \return void
 *
 * \todo Review documentation.
 */
void TriangleMesh::printPOVmesh2(std::ostream& os, const povray::Texture& texture,
         const Vec3 euler, const Vec3 povPos) const
{
   os << "mesh2 {\n";

   //output the cached mesh info
   os << cachedPOVString_;

   //add texture information
   texture.print( os, "   ", true );

   //rotation and translation in PovRay coordinates
   os << "   rotate " << euler << "\n";
   os << "   translate " << povPos << "\n";

   os << "}\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Updates the cachedPOVString_ with constant part of the triangle meshes
 *  information of the a PovRay mesh2.
 *
 *  \return void
 *
 *  \todo Review documentation.
 */
void TriangleMesh::updatePOVcacheString()
{
   char const * delim = "";
   static const Vec3 repariere(1.0, 1.0, -1.0);

   std::stringstream ss;

   //print vertex list
   ss << "   vertex_vectors {\n";
   ss << "      " << verticesOriginal_.size() << ",\n";
   ss << "      ";

   delim = "";
   for ( Vertices::const_iterator vertex = verticesOriginal_.begin(); vertex != verticesOriginal_.end(); ++vertex, delim = ", "){
      ss << delim << (*vertex)*repariere;
   }
   ss << "\n   }\n";


   //print normal list if necessary
   //NOTE: be aware of http://wiki.povray.org/content/Knowledgebase:The_Shadow_Line_Artifact
   if(renderSmooth_ && vertexNormals_.size() != 0) {
      ss << "   normal_vectors {\n";
      ss << "      " << vertexNormals_.size() << ",\n";
      ss << "      ";

      delim = "";
      for ( Normals::const_iterator normal = vertexNormals_.begin(); normal != vertexNormals_.end(); ++normal, delim = ", "){
         ss << delim << (*normal)*repariere;
      }
      ss << "\n   }\n";
   }

   //print uv texture coordinates if given
   if(textureCoordinates_.size() != 0) {
      ss << "    uv_vectors {\n";
      ss << "      " << textureCoordinates_.size() << ",\n";
      ss << "      ";

      delim = "";
      for ( TextureCoordinates::const_iterator textCoord = textureCoordinates_.begin(); textCoord != textureCoordinates_.end(); ++textCoord, delim = ", "){
         ss << delim << *textCoord;
      }
      ss << "\n   }\n";
   }

   //print face indices
   ss << "   face_indices {\n";
   ss << "      " << faceIndices_.size() << ",\n";
   ss << "      ";

   delim = "";
   for ( IndicesLists::const_iterator face = faceIndices_.begin(); face != faceIndices_.end(); ++face, delim=", "){
      ss << delim << *face;
   }
   ss << "\n   }\n";


   //print normal indices for each vertex if necessary
   if(renderSmooth_ && normalIndices_.size() != 0) {
      ss << "   normal_indices {\n";
      ss << "      " << normalIndices_.size() << ",\n";
      ss << "      ";

      delim = "";
      for ( IndicesLists::const_iterator faceNormals = normalIndices_.begin(); faceNormals != normalIndices_.end(); ++faceNormals, delim=", "){
         ss << delim << *faceNormals;
      }
      ss << "\n   }\n";
   }

   //print texture coordinate indices if given
   if(textureIndices_.size() != 0 && textureCoordinates_.size() != 0) {
      ss << "   uv_indices {\n";
      ss << "      " << textureIndices_.size() << ",\n";
      ss << "      ";

      delim = "";
      for ( IndicesLists::const_iterator textIndex = textureIndices_.begin(); textIndex != textureIndices_.end(); ++textIndex, delim=", "){
         ss << delim << *textIndex;
      }
      ss << "\n   }\n";
   }

   if(textureCoordinates_.size() != 0) {
      ss << "   uv_mapping\n";
   }

   cachedPOVString_ = ss.str();
}
//*************************************************************************************************


#if HAVE_IRRLICHT
//*************************************************************************************************
/*!\brief Updtaes the cachedIrrlichtBuffer_ with constant part of the triangle meshes
 *
 * \return void
 *
 * \todo Review documentation.
 * TODO surface normals, texturs and so on
 */
void TriangleMesh::updateIrrlichtCacheBuffer()
{
   using ::irr::u16;
   using ::irr::f32;
   using ::irr::video::S3DVertex;
   using ::irr::video::SColor;
   using ::irr::scene::SMeshBuffer;


   u16 index( 0 );
   S3DVertex v;
   //SMeshBuffer*   mb = cachedIrrlichtBuffer_;
   if(cachedIrrlichtBuffer_ != NULL) {
      cachedIrrlichtBuffer_->drop();
      cachedIrrlichtBuffer_ = NULL;
   }
   cachedIrrlichtBuffer_ = new SMeshBuffer();

   // Setting the default color of the triangle mesh
   v.Color = SColor( 255, 255, 255, 255 );

   // Building the Irrlicht mesh geometrie
   // Note: the Irrlicht engine expects the vertices of the triangles in clockwise
   // (left-handed) order!

   for(size_t face=0; face < faceIndices_.size(); ++face) {
      const Vec3& normal(faceNormals_[face]);
      v.Normal.X = static_cast<f32>( normal[0] );
      v.Normal.Y = static_cast<f32>( normal[2] );
      v.Normal.Z = static_cast<f32>( normal[1] );

      const Vec3& a( verticesOriginal_[faceIndices_[face][0]] );
      v.Pos.X = static_cast<f32>( a[0] );
      v.Pos.Y = static_cast<f32>( a[2] );
      v.Pos.Z = static_cast<f32>( a[1] );
      cachedIrrlichtBuffer_->Vertices.push_back( v );
      cachedIrrlichtBuffer_->Indices.push_back( index++ );

      const Vec3& c( verticesOriginal_[faceIndices_[face][2]] );
      v.Pos.X = static_cast<f32>( c[0] );
      v.Pos.Y = static_cast<f32>( c[2] );
      v.Pos.Z = static_cast<f32>( c[1] );
      cachedIrrlichtBuffer_->Vertices.push_back( v );
      cachedIrrlichtBuffer_->Indices.push_back( index++ );

      const Vec3& b( verticesOriginal_[faceIndices_[face][1]] );
      v.Pos.X = static_cast<f32>( b[0] );
      v.Pos.Y = static_cast<f32>( b[2] );
      v.Pos.Z = static_cast<f32>( b[1] );
      cachedIrrlichtBuffer_->Vertices.push_back( v );
      cachedIrrlichtBuffer_->Indices.push_back( index++ );
   }
}
//*************************************************************************************************
#endif




//=================================================================================================
//
//  TRIANGLE MESH SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of a new triangle mesh described by the given file
 * \ingroup triangleMesh
 *
 * \param uid User-specific ID for the triangle mesh.
 * \param gpos Global geometric center of the triangle mesh.
 * \param material The material of the triangle mesh.
 * \param convex \a true if the triangle mesh described in \a file is convex, otherwise false.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 * \param scale Scaling factors of the triangle mesh described in \a file, in the directions of the coordinate axis.
 * \param clockwise \a true if the triangles described in \a file are ordered clockwise, otherwise \a false, which is more common.
 * \param lefthanded \a true if the coordinates used in the OBJ-file are from a left-handed coordinate system, \a false if the coordinate system is right-handed (pe default)
 * \return Handle for the new triangle mesh.
 * \exception std::invalid_argument Invalid scaling factor, only positive scaling allowed
 * \exception std::invalid_argument Unsupported triangle mesh format.
 * \exception std::invalid_argument Invalid global triangle mesh position.
 *
 * \note currently only OBJ-files are supported as input files
 *
 * \todo Review documentation.
 */
PE_PUBLIC TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const std::string file,
                                     MaterialID material, bool convex,
                                     bool visible, const Vec3& scale,  bool clockwise, bool lefthanded )
{
   pe_INTERNAL_ASSERT( convex, "Only convex triangle meshes are allowed right now" );

   if(scale[0] <= real(0) || scale[1] <= real(0) || scale[2] <= real(0)){
      throw std::invalid_argument("Invalid scaling factor, only positive scaling allowed.");
   }

   const bool global( GlobalSection::isActive() );

   // Checking for the input file formte
   if(file.find(".obj") == std::string::npos && file.find(".OBJ") == std::string::npos
            /*&& file.find(".stl") == std::string::npos && file.find(".STL") == std::string::npos*/) {
      throw std::invalid_argument("Unsupported triangle mesh format.");
   }

   // Checking the global position of the triangle mesh
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global triangle mesh position." );

   //Variables to hold mesh information during inizialisierung
   Vertices         vertices;
   IndicesLists     faceIndices;
   Normals          faceNormals;
   Normals          vertexNormals;
   IndicesLists     normalIndices;
   TextureCoordinates texturCoordinates;
   IndicesLists     texturIndices;

   //reading the actual geometry file
   if((file.find(".obj") != std::string::npos) || (file.find(".OBJ") != std::string::npos)) {
      //Reading triangle mesh form OBJ-File
      TriangleMesh::initOBJ(file,
               vertices, faceIndices,
               faceNormals,
               vertexNormals, normalIndices,
               texturCoordinates, texturIndices,
               clockwise, lefthanded);
   }
   /*NOT supported any longer as there is no halfe endge informiateon
   else if ((file.find(".stl") != std::string::npos) || (file.find(".STL") != std::string::npos)) {
      //Reading triangle mesh form STL-File
      TriangleMesh::initSTL(file.c_str(), vertices, faceIndices, faceNormals);
   }
   */
   else {
      throw std::invalid_argument( "Invalid input file type for triangle mesh" );
   }

   //Calculate centre of mass and volume
   //http://stackoverflow.com/questions/2083771/a-method-to-calculate-the-centre-of-mass-from-a-stl-stereo-lithography-file
   real totalVolume ( 0.0 );
   real currentVolume ( 0.0 );
   Vec3 center (0.0, 0.0, 0.0);

   for (size_t i = 0; i < faceIndices.size(); ++i) {
      const Vec3& a = vertices[faceIndices[i][0]];
      const Vec3& b = vertices[faceIndices[i][1]];
      const Vec3& c = vertices[faceIndices[i][2]];

      //http://mathworld.wolfram.com/Tetrahedron.html
      currentVolume = (trans(a) * ( b % c ));
      totalVolume += currentVolume;
      center += (a + b +c) * currentVolume; //* 0.25
   }

   center /= totalVolume*real(4.0); //anstelle von *0.25
/*   static const real sixth = 1.0 / 6.0;
   totalVolume *= sixth;

   std::cout << file << std::endl;
   std::cout << "Total Volume = " << totalVolume << std::endl;
   std::cout << "X center = " << center[0] << std::endl; //xCenter/totalVolume
   std::cout << "Y center = " << center[1] << std::endl; //yCenter/totalVolume
   std::cout << "Z center = " << center[2] << std::endl; //zCenter/totalVolume
*/
   //move the triangle mesh so that the COM is the coordinate origin
   //and scale it
   for(Vertices::iterator v=vertices.begin(); v != vertices.end(); ++v) {
      (*v) = ((*v) - center) * scale;
   }


   //Determining  the SID
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   //Creating the new triangle mesh
   TriangleMeshID mesh = new TriangleMesh( sid, uid, gpos,
            vertices, faceIndices,
            faceNormals,
            vertexNormals, normalIndices,
            texturCoordinates, texturIndices,
            material, visible, convex );


   // Checking if the triangle mesh is created inside a global section
   if( global )
      mesh->setGlobal();

   // Checking if the triangle mesh has to be permanently fixed
   else if( mesh->isAlwaysFixed() )
      mesh->setFixed( true );


   try {
      // Registering the new triangle mesh with the default body manager
      theDefaultManager()->add( mesh );
   }
   catch( ... ) {
      delete mesh;
      throw;
   }

   // Logging the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created triangle mesh " << sid << "\n"
          << "   User-ID             = " << uid << "\n"
          << "   Global position     = " << gpos << "\n"
          << "   Number of triangles = " << mesh->size() << "\n"
          << "   Material            = " << Material::getName( material );
   }

   return mesh;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new triangle mesh.
 * \ingroup triangleMesh

 * \param uid The user-specific ID of the triangle mesh.
 * \param gpos The global position of the center of the triangle mesh.
 * \param source The source triangle mesh a copy is be made of.
 * \param material The material of the triangle mesh.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 * \return Handle for the copy triangle mesh.
 * \exception std::invalid_argument Invalid global triangle mesh position.
 *
 * This is a quick and dirty implementation to create triangle mesh copy
 * only the mesh data is copied and the mesh is placed to \a gpos, the 
 * rotation of the \a source mesh is not copied 
 */
PE_PUBLIC TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, const TriangleMeshID source, 
                                     MaterialID material, bool visible)
{
   const bool global( GlobalSection::isActive() );


   // Checking the global position of the triangle mesh
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global triangle mesh position" );

   //FEATURE? nicht einfach eine Kopie machen lassen, sondern alles was berechnet ist und kopiert werden kann
   //FEATURE? auch wirklich kopieren und nicht neu berechen!

   //Determining the SID
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   //Creating the new triangle mesh
   TriangleMeshID copy = new TriangleMesh( sid, uid, gpos,
            source->verticesOriginal_, source->faceIndices_,
            source->faceNormals_,
            source->vertexNormals_, source->normalIndices_,
            source->textureCoordinates_, source->textureIndices_,
            material, visible, source->convex_ );


   // Checking if the triangle mesh is created inside a global section
   if( global )
      copy->setGlobal();

   // Checking if the triangle mesh has to be permanently fixed
   else if( copy->isAlwaysFixed() )
      copy->setFixed( true );


   try {
      // Registering the new triangle mesh with the default body manager
      theDefaultManager()->add( copy );
   }
   catch( ... ) {
      delete copy;
      throw;
   }

   // Logging the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created triangle mesh " << sid << "\n"
          << "   User-ID             = " << uid << "\n"
          << "   Global position     = " << gpos << "\n"
          << "   Number of triangles = " << copy->size() << "\n"
          << "   Material            = " << Material::getName( material );
   }

   return copy;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a triangle mesh given vertex and face lists.
 * \ingroup triangleMesh
 *
 * \param uid The user-specific ID of the triangle mesh.
 * \param gpos The global position of the center of the triangle mesh.
 * \param vertices The vector of all vertices in the mesh.
 * \param faces The vector of all faces defined by indices into the vertices vector.
 * \param material The material of the triangle mesh.
 * \param convex Must be \a false if the mesh is not convex.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 * \return Handle for the new triangle mesh.
 * \exception std::invalid_argument Invalid global position.
 * \exception std::invalid_argument Invalid input.
 *
 * \todo Document.
 */
PE_PUBLIC TriangleMeshID createTriangleMesh( id_t uid, const Vec3& gpos, Vertices vertices,
                                     const IndicesLists& faces, MaterialID material,
                                     bool convex, bool visible )
{
   pe_INTERNAL_ASSERT( convex, "Only convex triangle meshes are allowed right now" );

   const bool global( GlobalSection::isActive() );

   // Checking the side lengths
   if( vertices.size() == 0 || faces.size() == 0 )
      throw std::invalid_argument( "Invalid input" );

   // Checking the global position of the triangle mesh
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global position" );
 

   // Calculate center of mass and volume
   real totalVolume( 0.0 );
   real currentVolume( 0.0 );
   Vec3 center;

   for( size_t i = 0; i < faces.size(); ++i ) {
      const Vec3& a = vertices[faces[i][0]];
      const Vec3& b = vertices[faces[i][1]];
      const Vec3& c = vertices[faces[i][2]];

      currentVolume = ( trans(a) * ( b % c ) );
      totalVolume += currentVolume;
      center += (a + b + c) * currentVolume;
   }

   center /= totalVolume * real( 4 );

   // Move the triangle mesh so that the COM is the coordinate origin
   for( Vertices::iterator v = vertices.begin(); v != vertices.end(); ++v )
      *v -= center;

   // Creating the new triangle mesh
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   TriangleMeshID mesh = new TriangleMesh( sid, uid, gpos, vertices, faces, Normals(), Normals(),
                                   IndicesLists(), TextureCoordinates(), IndicesLists(), material,
                                   visible, convex );

   // Checking if the triangle mesh is created inside a global section
   if( global )
      mesh->setGlobal();

   // Checking if the triangle mesh has to be permanently fixed
   else if( mesh->isAlwaysFixed() )
      mesh->setFixed( true );

   // Registering the new triangle mesh with the default body manager
   try {
      theDefaultManager()->add( mesh );
   }
   catch( std::exception& ) {
      delete mesh;
      throw;
   }

   // Logging the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created triangle mesh " << sid << "\n"
          << "   User-ID             = " << uid << "\n"
          << "   Global position     = " << gpos << "\n"
          << "   Number of triangles = " << mesh->size() << "\n"
          << "   Material            = " << Material::getName( material );
   }

   return mesh;
}
//*************************************************************************************************


//*************************************************************************************************
/*
 *
 *   https://sites.google.com/site/dlampetest/python/triangulating-a-sphere-recursively
 *
 *                     Make new points
 *                       a = (0+2)/2
 *                       b = (0+1)/2
 *                       c = (1+2)/2
 *          1
 *         /\          Normalize a, b, c
 *        /  \
 *      b/____\ c      Construct new faces (triangles)
 *      /\    /\         f1 [0,b,a]
 *     /  \  /  \        f2 [b,1,c]
 *    /____\/____\       f3 [a,b,c]
 *   0      a     2      f4 [a,c,2]
 *
 *                     Delete old face [0,2,1]
 *
 */
void subdivideTriangles( Vertices & vertices, IndicesLists & faceIndices )
{
   size_t oldNumFaces = faceIndices.size();
   
   for( size_t i = 0; i < oldNumFaces; i++ )
   {
      const Vec3 v0 = vertices[ faceIndices[i][0] ];
      const Vec3 v1 = vertices[ faceIndices[i][1] ];
      const Vec3 v2 = vertices[ faceIndices[i][2] ];
      
      Vec3 a( ( v0 + v2 ) * 0.5 );  //<-- vertex index = vertices.size() - 1
      Vec3 b( ( v0 + v1 ) * 0.5 );  //<-- vertex index = vertices.size() - 2
      Vec3 c( ( v1 + v2 ) * 0.5 );  //<-- vertex index = vertices.size() - 3
      
      a.normalize();
      b.normalize();
      c.normalize();
      
      vertices.push_back( a );
      vertices.push_back( b );
      vertices.push_back( c );
      
      const size_t aIdx = vertices.size() - 1;
      const size_t bIdx = vertices.size() - 2;
      const size_t cIdx = vertices.size() - 3;
      
      faceIndices.push_back( Vector3<size_t>( faceIndices[i][0], bIdx             , aIdx              ) );  // (v0, b, a)
      faceIndices.push_back( Vector3<size_t>( bIdx             , faceIndices[i][1], cIdx              ) );  // (b, v1, c)
      faceIndices.push_back( Vector3<size_t>( aIdx             , bIdx             , cIdx              ) );  // (a, b, c)
      faceIndices.push_back( Vector3<size_t>( aIdx             , cIdx             , faceIndices[i][2] ) );  // (a, c, v2)
   }
   
   // delete old (big) faces
   faceIndices.erase( faceIndices.begin(), faceIndices.begin() + oldNumFaces );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new triangle mesh representing a sphere
 * \ingroup triangleMesh
 *
 * \param uid The user-specific ID of the sphere.
 * \param gpos The global position of the center of the sphere.
 * \param radius The radius of the sphere  \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param recursionLevel The number of recursive face refinement \f$ [0..\infty) \f$.
 * \param visible Specifies if the mesh is visible in a visualization.
 * \return Handle for the new sphere triangle mesh.
 * \exception std::invalid_argument Invalid radius.
 * \exception std::invalid_argument Invalid recursion level.
 * \exception std::invalid_argument Invalid global mesh position.
 *
 * This function creates a triangle mesh with the shape of a sphere in the \b pe simulation system.
 * The new mesh with the user-specific ID \a uid is placed at the global position \a gpos.
 * The sphere has the radius \a radius, and consists of the material \a material. The \a visible flag sets the mesh
 * (in-)visible in all visualizations.
 *
 * If the \a recursionLevel is zero, the mesh will have 6 vertices and 8 faces, i.e. will represent an octahedron.
 * With every increase of the recursion level, each face will be be subdivided into 4 faces, leading to a smoother
 * approximation of a spherical shape.
 *
 */
TriangleMeshID createTriangulatedSphere(id_t uid, const Vec3& gpos, real radius, MaterialID material, int recursionLevel, bool visible)
{
   const bool global( GlobalSection::isActive() );
   
   if( radius <= real(0) )
      throw std::invalid_argument( "Invalid radius. The radius must not be smaller/equal zero." );
      
   if( recursionLevel < 0 )
      throw std::invalid_argument( "Invalid recursionLevel. The recursion level must not be smaller than zero." );
      
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global position" );
      
   Vertices vertices;
   IndicesLists faceIndices;
   IndicesLists subdividedFaceIndices;
   
   // initialize vertices and faces with unit octahedron geometry 
   vertices.push_back( Vec3(  1.0,  0.0,  0.0 ) );
   vertices.push_back( Vec3( -1.0,  0.0,  0.0 ) );
   vertices.push_back( Vec3(  0.0,  1.0,  0.0 ) );
   vertices.push_back( Vec3(  0.0, -1.0,  0.0 ) );
   vertices.push_back( Vec3(  0.0,  0.0,  1.0 ) );
   vertices.push_back( Vec3(  0.0,  0.0, -1.0 ) );
                               
   faceIndices.push_back( Vector3<size_t>( 0, 4, 2 ) );
   faceIndices.push_back( Vector3<size_t>( 2, 4, 1 ) );
   faceIndices.push_back( Vector3<size_t>( 1, 4, 3 ) );
   faceIndices.push_back( Vector3<size_t>( 3, 4, 0 ) );
   faceIndices.push_back( Vector3<size_t>( 0, 2, 5 ) );
   faceIndices.push_back( Vector3<size_t>( 2, 1, 5 ) );
   faceIndices.push_back( Vector3<size_t>( 1, 3, 5 ) );
   faceIndices.push_back( Vector3<size_t>( 3, 0, 5 ) );
   
   
   // recursively determine mesh vertices and faces
   for( int i = 0; i < recursionLevel; i++ ) {
      subdivideTriangles( vertices, faceIndices );
   }
   
   // move vertices to the correct positions
   for( size_t i = 0; i < vertices.size(); ++i ) {
      vertices[i] *= radius;
      vertices[i] += gpos;
   }
   
   Normals faceNormals( faceIndices.size() );
   Normals vertexNormals;
   IndicesLists normalIndices;
   TextureCoordinates textureCoordinates;
   IndicesLists textureIndices;
   
   // calculate normals
   for( size_t f = 0; f < faceIndices.size(); ++f )
   {
        Vec3 & a = vertices[faceIndices[f][0]];
        Vec3 & b = vertices[faceIndices[f][1]];
        Vec3 & c = vertices[faceIndices[f][2]];

        // calculate face normal
        Vec3 ab = b-a;
        Vec3 ac = c-a;
        Vec3 n  = ab % ac;
        n.normalize();
        faceNormals[f] = n;
   }
   
   // determine the system ID
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   
   // creating the new triangle mesh
   TriangleMeshID mesh = new TriangleMesh( sid, uid, gpos, vertices, faceIndices,
                                           faceNormals,
                                           vertexNormals, normalIndices,
                                           textureCoordinates, textureIndices,
                                           material, visible, true );

   // check if the triangle mesh is created inside a global section
   if( global )
      mesh->setGlobal();

   // check if the triangle mesh has to be permanently fixed
   else if( mesh->isAlwaysFixed() )
      mesh->setFixed( true );

   try {
      // register the new triangle mesh with the default body manager
      theDefaultManager()->add( mesh );
   }
   catch( ... ) {
      delete mesh;
      throw;
   }

   // log the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created triangle mesh " << sid << "\n"
          << "   User-ID             = " << uid << "\n"
          << "   Global position     = " << gpos << "\n"
          << "   Number of triangles = " << faceIndices.size() << "\n"
          << "   Material            = " << Material::getName( material );
   }
   return mesh;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Setup of a new regular tetrahedron triangle mesh.
 * \ingroup triangleMesh
 *
 * \param uid The user-specific ID of the tetrahedron.
 * \param gpos The global position of the center of the tetrahedron.
 * \param radius of circumsphere  \f$ (0..\infty) \f$.
 * \param material The material of the tetrahedron.
 * \param visible Specifies if the tetrahedron is visible in a visualization.
 * \return Handle for the new tetrahedron triangle mesh.
 * \exception std::invalid_argument Invalid radius.
 * \exception std::invalid_argument Invalid global tetrahedron position.
 *
 * This function creates a triangle mesh with the shape of a regular tetrahedron in the \b pe simulation system.
 * The new tetrahedron with the user-specific ID \a uid is placed at the global position \a gpos.
 * Its circumsphere has the radius \a radius, and consists of the material \a material. The \a visible flag sets the box
 * (in-)visible in all visualizations.
 *
 * The tetrahedron points in positive z-Direction the opposite triangle is parallel to the x-y-Plane.
 * one point lays within the y-z-Plane in negative y-direction
 *
 * \todo Review documentation.
 */
TriangleMeshID createRegularTetrahedron(id_t uid, const Vec3& gpos, real radius, MaterialID material,  bool visible)
{
   const bool global( GlobalSection::isActive() );

   // Checking the radius
   if( radius <= real(0)  )
      throw std::invalid_argument( "Invalid radius" );

   // Checking the global position of the triangle mesh
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global tetrahedron position" );


   //Variables to hold mesh information during initialization
   Vertices         vertices(4);
   IndicesLists     faceIndices(4);
   Normals          faceNormals(4);
   Normals          vertexNormals;
   IndicesLists     normalIndices;
   TextureCoordinates texturCoordinates;
   IndicesLists     texturIndices;

   //calculate points
   vertices[0] = Vec3(0.0, 0.0, radius); //oben

   Quat tetraederwinkel(acos( real(-1.0) / real(3.0) ), real(0.0) , real(0.0) );

   vertices[1] = tetraederwinkel.rotate(vertices[0]); //vorne

   Quat hundertzwanzig( real(0.0), real(0.0), real(2.0)/real(3.0) * M_PI);
   vertices[2] = hundertzwanzig.rotate(vertices[1]); //hinten rechts
   vertices[3] = hundertzwanzig.rotate(vertices[2]); //hinten links


   //assemble faces
   faceIndices[0] = Vector3< size_t >(0, 1, 2); //vorne rechts
   faceIndices[1] = Vector3< size_t >(0, 2, 3); //hinten
   faceIndices[2] = Vector3< size_t >(0, 3, 1); //vorne links
   faceIndices[3] = Vector3< size_t >(1, 3, 2); //unten

   //calculate normals
   for(size_t f = 0; f < faceIndices.size(); ++f) {
        Vec3& a = vertices[faceIndices[f][0]];
        Vec3& b = vertices[faceIndices[f][1]];
        Vec3& c = vertices[faceIndices[f][2]];

        //calculate face normal
        Vec3 ab = b-a;
        Vec3 ac = c-a;
        Vec3 n  = ab % ac;
        n.normalize();
        faceNormals[f] = n;
   }


   //Determining the SID
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   //Creating the new triangle mesh
   TriangleMeshID tetra = new TriangleMesh( sid, uid, gpos,
            vertices, faceIndices,
            faceNormals,
            vertexNormals, normalIndices,
            texturCoordinates, texturIndices,
            material, visible, true );

   // Checking if the triangle mesh is created inside a global section
   if( global )
      tetra->setGlobal();

   // Checking if the triangle mesh has to be permanently fixed
   else if( tetra->isAlwaysFixed() )
      tetra->setFixed( true );


   try {
      // Registering the new triangle mesh with the default body manager
      theDefaultManager()->add( tetra );
   }
   catch( ... ) {
      delete tetra;
      throw;
   }

   // Logging the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created triangle mesh " << sid << "\n"
          << "   User-ID             = " << uid << "\n"
          << "   Global position     = " << gpos << "\n"
          << "   Number of triangles = " << tetra->size() << "\n"
          << "   Material            = " << Material::getName( material );
   }
   return tetra;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new box as a triangle mesh.
 * \ingroup triangleMesh
 *
 * \param uid The user-specific ID of the box.
 * \param gpos The global position of the center of the box.
 * \param material The material of the box.
 * \param lengths The side length of the box \f$ (0..\infty) \f$.
 * \param visible Specifies if the box is visible in a visualization.
 * \param teselation The number of division in each direction \f$ (1..\infty) \f$.
 * \return Handle for the new box triangle mesh.
 * \exception std::invalid_argument Invalid side length.
 * \exception std::invalid_argument Invalid number of divisions.
 * \exception std::invalid_argument Invalid global box position.
 *
 * This function creates a triangle mesh with the shape of a box in the \b pe simulation system.
 * The new box with the user-specific ID \a uid is placed at the global position \a gpos, has the side lengths
 * \a lengths, and consists of the material \a material. The \a visible flag sets the box
 * (in-)visible in all visualizations. The box surface of the box is divided into rectangles, the number of
 * divisions in each direction is given by \a tessellation. Each rectangle is also divided into two triangles.
 *
 * \todo Review documentation.
 */
TriangleMeshID createTriangulatedBox( id_t uid, const Vec3& gpos, const Vec3& lengths, MaterialID material,
                             bool visible, const Vector3<size_t>& teselation)
{
   const bool global( GlobalSection::isActive() );

   // Checking the side lengths
   if( lengths[0] <= real(0) || lengths[1] <= real(0) || lengths[2] <= real(0) )
      throw std::invalid_argument( "Invalid side length" );

   // Checking the number of devisions
   if( teselation[0] == 0 || teselation[1] == 0 || teselation[2] == 0 )
      throw std::invalid_argument( "Invalid number of divisions" );

   // Checking the global position of the box
   if( !global && !CreateUnion::isActive() && !theCollisionSystem()->getDomain().ownsPoint( gpos ) )
      throw std::invalid_argument( "Invalid global box(triangle mesh) position" );

   const size_t& tesX = teselation[0]; //x right
   const size_t& tesY = teselation[1]; //y back (away from viewer)
   const size_t& tesZ = teselation[2]; //z up

   const size_t rangeX = tesX + 1;
   const size_t rangeXY = (tesX + 1) * (tesY + 1);

   const size_t numberOfPoints =  2*(1+tesX)*(1+tesZ) //vorder und rueckseite
                                 +2*(tesY-1)*(1+tesZ) //rechts und links
                                 +2*(tesX-1)*(tesY-1);//oben und unten
   const size_t numberOfFaces  = 2*( 2*tesX*tesZ  //vorder und rueckseite
                                    +2*tesY*tesZ  //rechts und links
                                    +2*tesX*tesY); //oben und unten

   //Variables to hold mesh information during inizialisierung
   Vertices           vertices;
   IndicesLists       faceIndices;
   Normals            faceNormals;
   Normals            vertexNormals;
   IndicesLists       normalIndices;
   TextureCoordinates texturCoordinates;
   IndicesLists       texturIndices;

   vertices.reserve(numberOfPoints);
   faceIndices.reserve(numberOfFaces);
   faceNormals.reserve(numberOfFaces);


   std::map<size_t, size_t> posToVertexIndex; //pos = x + y*rangeX + z*rangeXY
   //std::map<Vector3< size_t >, size_t> posToVertexIndex; //pos = x + y*rangeX + z*rangeXY
   size_t currentIndex = 0;


   //calculate the vertex coordinates
   const real stepX = lengths[0] / tesX;
   const real stepY = lengths[1] / tesY;
   const real stepZ = lengths[2] / tesZ;

   //front side
   const Vec3 frontBottomLeft( real(-0.5)*lengths[0], real(-0.5)*lengths[1], real(-0.5)*lengths[2]);
   for(size_t x = 0; x <= tesX; ++x) {
      for(size_t z =0; z <= tesZ; ++z) {
         vertices.push_back(frontBottomLeft + Vec3(x*stepX, 0.0, z*stepZ));

         posToVertexIndex[x + 0*rangeX + z*rangeXY] = currentIndex;
         //posToVertexIndex[Vector3<size_t>(x,0,z)] = currentIndex;
         ++currentIndex;
      }
   }

   //back side
   const Vec3 backBottomLeft( real(-0.5)*lengths[0], real(0.5)*lengths[1], real(-0.5)*lengths[2]);
   for(size_t x = 0; x <= tesX; ++x) {
      for(size_t z =0; z <= tesZ; ++z) {
         vertices.push_back(backBottomLeft + Vec3(x*stepX, 0.0, z*stepZ));

         posToVertexIndex[x + tesY*rangeX + z*rangeXY] = currentIndex;
         //posToVertexIndex[Vector3<size_t>(x,tesY,z)] = currentIndex;
         ++currentIndex;
      }
   }

   //left side
   for(size_t y = 1; y < tesY; ++y) { //side points are part of front side and back side
      for(size_t z = 0; z <= tesZ; ++z) {
         vertices.push_back(frontBottomLeft + Vec3(0.0, y*stepY, z*stepZ));

         posToVertexIndex[0 + y*rangeX + z*rangeXY] = currentIndex;
         //posToVertexIndex[Vector3<size_t>(0,y,z)] = currentIndex;
         ++currentIndex;
      }
   }

   //right side
   const Vec3 frontBottomRight( real(0.5)*lengths[0], real(-0.5)*lengths[1], real(-0.5)*lengths[2]);
   for(size_t y = 1; y < tesY; ++y) { //side points are part of front side and back side
      for(size_t z = 0; z <= tesZ; ++z) {
         vertices.push_back(frontBottomRight + Vec3(0.0, y*stepY, z*stepZ));

         posToVertexIndex[tesX + y*rangeX + z*rangeXY] = currentIndex;
         //posToVertexIndex[Vector3<size_t>(tesX,y,z)] = currentIndex;
         ++currentIndex;
      }
   }

   //bottom
   for(size_t x = 1; x < tesX; ++x) { //side points are part of left side and right side
      for(size_t y = 1; y < tesY; ++y) { //side points are part of front and back side
         vertices.push_back(frontBottomLeft + Vec3(x*stepX, y*stepY, 0.0));

         posToVertexIndex[x + y*rangeX + 0*rangeXY] = currentIndex;
         //posToVertexIndex[Vector3<size_t>(x,y,0)] = currentIndex;
         ++currentIndex;
      }
   }

   //top
   const Vec3 frontTopLeft( real(-0.5)*lengths[0], real(-0.5)*lengths[1], real(0.5)*lengths[2]);
   for(size_t x = 1; x < tesX; ++x) { //side points are part of left side and right side
      for(size_t y = 1; y < tesY; ++y) { //side points are part of front and back side
         vertices.push_back(frontTopLeft + Vec3(x*stepX, y*stepY, 0.0));

         posToVertexIndex[x + y*rangeX + tesZ*rangeXY] = currentIndex;
         //posToVertexIndex[Vector3<size_t>(x,y,tesZ)] = currentIndex;
         ++currentIndex;
      }
   }

   //arrange the triangles
   //front side
   for(size_t x = 0; x < tesX; ++x) {
      for(size_t z = 0; z < tesZ; ++z) {
         size_t indexLinksUnten  = posToVertexIndex[(x+0) + 0*rangeX + (z+0)*rangeXY]; //Vector3<size_t>(x+0,0,z+0)]; //
         size_t indexRechtsUnten = posToVertexIndex[(x+1) + 0*rangeX + (z+0)*rangeXY]; //Vector3<size_t>(x+1,0,z+0)]; //
         size_t indexLinksOben   = posToVertexIndex[(x+0) + 0*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(x+0,0,z+1)]; //
         size_t indexRechtsOben  = posToVertexIndex[(x+1) + 0*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(x+1,0,z+1)]; //

         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsOben,  indexLinksOben));  //oberes Dreieck
         faceNormals.push_back(Vec3(0.0, -1.0, 0.0));
         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsUnten, indexRechtsOben)); //unteres Dreieck
         faceNormals.push_back(Vec3(0.0, -1.0, 0.0));
      }
   }

   //back side
   for(size_t x = 0; x < tesX; ++x) {
      for(size_t z = 0; z < tesZ; ++z) {
         size_t indexLinksUnten  = posToVertexIndex[(x+1) + tesY*rangeX + (z+0)*rangeXY]; // Vector3<size_t>(x+1,tesY,z+1)]; //
         size_t indexRechtsUnten = posToVertexIndex[(x+0) + tesY*rangeX + (z+0)*rangeXY]; //Vector3<size_t>(x+1,tesY,z+1)]; //
         size_t indexLinksOben   = posToVertexIndex[(x+1) + tesY*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(x+1,tesY,z+1)]; //
         size_t indexRechtsOben  = posToVertexIndex[(x+0) + tesY*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(x+1,tesY,z+1)]; //

         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsOben,  indexLinksOben));  //oberes Dreieck
         faceNormals.push_back(Vec3(0.0, 1.0, 0.0));
         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsUnten, indexRechtsOben)); //unteres Dreieck
         faceNormals.push_back(Vec3(0.0, 1.0, 0.0));
      }
   }

   //left side
   for(size_t y = 0; y < tesY; ++y) {
      for(size_t z = 0; z < tesZ; ++z) {
         size_t indexLinksUnten  = posToVertexIndex[0 + (y+1)*rangeX + (z+0)*rangeXY]; //Vector3<size_t>(0,y+1,z+0)]; //
         size_t indexRechtsUnten = posToVertexIndex[0 + (y+0)*rangeX + (z+0)*rangeXY]; //Vector3<size_t>(0,y+0,z+0)]; //
         size_t indexLinksOben   = posToVertexIndex[0 + (y+1)*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(0,y+1,z+1)]; //
         size_t indexRechtsOben  = posToVertexIndex[0 + (y+0)*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(0,y+0,z+1)]; //

         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsOben,  indexLinksOben));  //oberes Dreieck
         faceNormals.push_back(Vec3(-1.0, 0.0, 0.0));
         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsUnten, indexRechtsOben)); //unteres Dreieck
         faceNormals.push_back(Vec3(-1.0, 0.0, 0.0));
      }
   }

   //right side
   for(size_t y = 0; y < tesY; ++y) {
      for(size_t z = 0; z < tesZ; ++z) {
         size_t indexLinksUnten  = posToVertexIndex[tesX + (y+0)*rangeX + (z+0)*rangeXY]; //Vector3<size_t>(tesX,y+0,z+0)]; //
         size_t indexRechtsUnten = posToVertexIndex[tesX + (y+1)*rangeX + (z+0)*rangeXY]; //Vector3<size_t>(tesX,y+1,z+0)]; //
         size_t indexLinksOben   = posToVertexIndex[tesX + (y+0)*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(tesX,y+0,z+1)]; //
         size_t indexRechtsOben  = posToVertexIndex[tesX + (y+1)*rangeX + (z+1)*rangeXY]; //Vector3<size_t>(tesX,y+1,z+1)]; //

         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsOben,  indexLinksOben));  //oberes Dreieck
         faceNormals.push_back(Vec3(1.0, 0.0, 0.0));
         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsUnten, indexRechtsOben)); //unteres Dreieck
         faceNormals.push_back(Vec3(1.0, 0.0, 0.0));
      }
   }



   //bottom
   for(size_t x = 0; x < tesX; ++x) {
      for(size_t y = 0; y < tesY; ++y) {
         size_t indexLinksUnten  = posToVertexIndex[(x+0) + (y+1)*rangeX + 0*rangeXY]; //Vector3<size_t>(x+0,y+1,0)]; //
         size_t indexRechtsUnten = posToVertexIndex[(x+1) + (y+1)*rangeX + 0*rangeXY]; //Vector3<size_t>(x+1,y+1,0)]; //
         size_t indexLinksOben   = posToVertexIndex[(x+0) + (y+0)*rangeX + 0*rangeXY]; //Vector3<size_t>(x+0,y+0,0)]; //
         size_t indexRechtsOben  = posToVertexIndex[(x+1) + (y+0)*rangeX + 0*rangeXY]; //Vector3<size_t>(x+1,y+0,0)]; //

         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsOben,  indexLinksOben));  //oberes Dreieck
         faceNormals.push_back(Vec3(0.0, 0.0, -1.0));
         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsUnten, indexRechtsOben)); //unteres Dreieck
         faceNormals.push_back(Vec3(0.0, 0.0, -1.0));
      }
   }


   //top
   for(size_t x = 0; x < tesX; ++x) {
      for(size_t y = 0; y < tesY; ++y) {
         size_t indexLinksUnten  = posToVertexIndex[(x+0) + (y+0)*rangeX + tesZ*rangeXY]; //Vector3<size_t>(x+0,y+0,tesZ)]; //
         size_t indexRechtsUnten = posToVertexIndex[(x+1) + (y+0)*rangeX + tesZ*rangeXY]; //Vector3<size_t>(x+1,y+0,tesZ)]; //
         size_t indexLinksOben   = posToVertexIndex[(x+0) + (y+1)*rangeX + tesZ*rangeXY]; //Vector3<size_t>(x+0,y+1,tesZ)]; //
         size_t indexRechtsOben  = posToVertexIndex[(x+1) + (y+1)*rangeX + tesZ*rangeXY]; //Vector3<size_t>(x+1,y+1,tesZ)]; //

         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsOben,  indexLinksOben));  //oberes Dreieck
         faceNormals.push_back(Vec3(0.0, 0.0, 1.0));
         faceIndices.push_back(Vector3< size_t >(indexLinksUnten, indexRechtsUnten, indexRechtsOben)); //unteres Dreieck
         faceNormals.push_back(Vec3(0.0, 0.0, 1.0));
      }
   }


   //Determining the SID
   const id_t sid( global ? UniqueID<RigidBody>::createGlobal() : UniqueID<RigidBody>::create() );
   //Creating the new triangle mesh
   TriangleMeshID box = new TriangleMesh( sid, uid, gpos,
            vertices, faceIndices,
            faceNormals,
            vertexNormals, normalIndices,
            texturCoordinates, texturIndices,
            material, visible, true );

   // Checking if the triangle mesh is created inside a global section
   if( global )
      box->setGlobal();

   // Checking if the triangle mesh has to be permanently fixed
   else if( box->isAlwaysFixed() )
      box->setFixed( true );


   try {
      // Registering the new triangle mesh with the default body manager
      theDefaultManager()->add( box );
   }
   catch( ... ) {
      delete box;
      throw;
   }

   // Logging the successful creation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Created triangle mesh " << sid << "\n"
          << "   User-ID             = " << uid << "\n"
          << "   Global position     = " << gpos << "\n"
          << "   Number of triangles = " << box->size() << "\n"
          << "   Material            = " << Material::getName( material );
   }
   return box;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new rock.
 * \ingroup triangleMesh
 *
 * \param uid The user-specific ID of the rock.
 * \param gpos The global position of the center of the rock.
 * \param radius The radius of a sphere which is guaranteed to bound the rock \f$ (0..\infty) \f$.
 * \param material The material of the rock.
 * \param visible Specifies if the rock is visible in a visualization.
 * \return Handle for the new rock triangle mesh.
 * \exception std::invalid_argument Invalid radius.
 * \exception std::invalid_argument Invalid global rock position.
 *
 * This function creates a triangle mesh with the shape of a rock in the \b pe simulation system.
 * The new rock with the user-specific ID \a uid is placed at the global position \a gpos.
 * The rock's bounding radius is guarantee to be less than \a radius. The rock consists of the
 * material \a material. The \a visible flag sets the rock (in-)visible in all visualizations.
 *
 * The rock triangle mesh is the convex hull of 15 randomly generated unit vectors scaled to
 * fit inside the bounding sphere.
 */
TriangleMeshID createRock( id_t uid, const Vec3& gpos, real radius, MaterialID material,  bool visible )
{
   Vertices vertices;
   IndicesLists faces;
   pe::chull::generateRock( 15, vertices, faces, radius );
   return createTriangleMesh( uid, gpos, vertices, faces, material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Local instantiation of a triangle mesh given vertex and face lists.
 * \ingroup trianglemesh
 *
 * \param sid The unique system-specific ID of the triangle mesh.
 * \param uid The user-specific ID of the triangle mesh.
 * \param gpos The global position of the center of the triangle mesh.
 * \param rpos The relative position within the body frame of a superordinate body.
 * \param q The orientation of the triangle mesh's body frame in the global world frame.
 * \param vertices The vector of all vertices in the mesh.
 * \param faces The vector of all faces defined by indices into the vertices vector.
 * \param material The material of the triangle mesh.
 * \param visible Specifies if the triangle mesh is visible in a visualization.
 * \param fixed \a true to fix the triangle mesh, \a false to unfix it.
 * \param reg \a true to register the object in the default body manager.
 * \return Handle for the new triangle mesh.
 * \exception std::invalid_argument Invalid global position.
 * \exception std::invalid_argument Invalid input.
 *
 * This function instantiates a copy of a triangle mesh with a certain system-specific ID. For
 * instance, it is used to locally instantiate a copy of a triangle mesh residing on a remote
 * MPI process. This function must NOT be called explicitly, but is reserved for internal
 * use only!
 */
TriangleMeshID instantiateTriangleMesh( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                const Quat& q, const Vertices& vertices,
                                const IndicesLists& faces, MaterialID material, bool visible,
                                bool fixed, bool reg )
{
   // Checking the mesh
   if( vertices.size() == 0 || faces.size() <= 4 )
      throw std::invalid_argument( "Invalid input" );

   // Creating the new triangle mesh
   TriangleMeshID mesh = new TriangleMesh( sid, uid, gpos, rpos, q, vertices, faces, Normals(), Normals(),
                                   IndicesLists(), TextureCoordinates(), IndicesLists(), material,
                                   visible, fixed, true );

   // Registering the triangle mesh with the default body manager
   if( reg ) {
      try {
         theDefaultManager()->add( mesh );
      }
      catch( std::exception& ) {
         delete mesh;
         throw;
      }
   }

   // Logging the successful instantiation of the triangle mesh
   pe_LOG_DETAIL_SECTION( log ) {
      log << "Instantiated triangle mesh " << sid << "\n"
          << "   User-ID             = " << uid << "\n"
          << "   Global position     = " << gpos << "\n"
          << "   Number of triangles = " << mesh->size() << "\n"
          << "   Material            = " << Material::getName( material );
   }

   return mesh;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for triangle meshes.
 *
 * \param os Reference to the output stream.
 * \param m Reference to a constant triangle mesh object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const TriangleMesh& m )
{
   os << "--" << pe_BROWN << "TRIANGLE MESH PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------\n";
   m.print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for triangle mesh handles.
 *
 * \param os Reference to the output stream.
 * \param m Constant triangle mesh handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, ConstTriangleMeshID m )
{
   os << "--" << pe_BROWN << "TRIANGLE MESH PARAMETERS" << pe_OLDCOLOR
      << "------------------------------------------------------\n";
   m->print( os, "" );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
