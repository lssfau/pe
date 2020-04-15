//=================================================================================================
/*!
 *  \file pe/core/detection/fine/EPA.h
 *  \brief The Expanding-Polytope Algorithm
 *
 *  Copyright (C) 2001-2003 Dtecta
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
 *
 *  DISCLAIMER: The following source file contains modified code from the SOLID-3.5 library for
 *  interference detection as it is published in the book "Collision Detection in Interactive
 *  3D Environments" by Gino van den Bergen <info@dtecta.com>. Even though the original source
 *  was published under the GPL version 2 not allowing later versions, the original author of the
 *  source code permitted the relicensing of the SOLID-3.5 library under the GPL version 3 license.
 */
//=================================================================================================

#ifndef _PE_CORE_DETECTION_FINE_EPA_H_
#define _PE_CORE_DETECTION_FINE_EPA_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/detection/fine/GJK.h>
#include <pe/core/Thresholds.h>
#include <pe/math/Quaternion.h>
#include <pe/math/RotationMatrix.h>


namespace pe {

namespace detection {

namespace fine {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The Expanding-Polytope Algorithm.
 * \ingroup fine_collision_detection
 */
class EPA
{
private :
   //**Type definitions****************************************************************************
   class EPA_Edge;
   class EPA_Triangle;
   class EPA_TriangleComp;

   typedef std::vector<EPA_Triangle>  EPA_EntryBuffer;
   typedef std::vector<EPA_Triangle*> EPA_EntryHeap;
   typedef std::vector<EPA_Edge>      EPA_EdgeBuffer;
   //**********************************************************************************************

public:
   //**Query functions*****************************************************************************
   /*!\name Query functions */
   //@{
   template < typename Type1, typename Type2 >
   inline bool doEPAcontactThreshold( Type1 geom1, Type2 geom2, const GJK& gjk, Vec3& normal,
                                      Vec3& contactPoint, real& penetrationDepth );

   template < typename Type >
   inline bool doEPAcontactThreshold( Type geom, TriangleMeshID mesh, const GJK& gjk, Vec3& normal,
                                      Vec3& contactPoint, real& penetrationDepth );

   inline bool doEPAcontactThreshold( TriangleMeshID mA, TriangleMeshID mB, const GJK& gjk,
                                      Vec3& normal, Vec3& contactPoint, real& penetrationDepth );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool originInTetrahedron            ( const Vec3& A, const Vec3& B, const Vec3& C,
                                                const Vec3& D );
   inline bool originInTetrahedronVolumeMethod( const Vec3& A, const Vec3& B, const Vec3& C,
                                                const Vec3& D );
   inline bool pointInTetrahedron             ( const Vec3& A, const Vec3& B, const Vec3& C,
                                                const Vec3& D, const Vec3& point );
   inline void createInitialTetrahedron       ( size_t top, size_t frontLeft, size_t frontRight,
                                                size_t back, std::vector<Vec3>& epaVolume,
                                                EPA_EntryBuffer& entryBuffer );
   template < typename Type1, typename Type2 >
   inline void createInitialSimplex           ( unsigned char numPoints, Type1 geom1, Type2 geom2,
                                                std::vector<Vec3>& supportA,
                                                std::vector<Vec3>& supportB,
                                                std::vector<Vec3>& epaVolume,
                                                EPA_EntryBuffer& entryBuffer );
   //@}
   //**********************************************************************************************


private:
   //EPA constants
   static const size_t maxSupportPoints_ = 100;
   static const size_t maxTriangles_     = 200;
};
//*************************************************************************************************




//=================================================================================================
//
//  EPA::EPA_EDGE CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
class EPA::EPA_Edge {
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   EPA_Edge( EPA_Triangle* triangle, unsigned char index );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   EPA_Triangle* getTriangle() const;
   unsigned char getIndex()    const;
   size_t        getStart()    const;
   size_t        getEnd()      const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   EPA_Triangle* triangle_; //!< the EPA triangle the edge is contained in
   unsigned char startIdx_; //!< the index of the point the edge starts at (0, 1, 2)
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  EPA::EPA_TRIANGLE CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * see Collision detction in interactiv 3D environments; Gino van den bergen page 155
 */
class EPA::EPA_Triangle {
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline EPA_Triangle( size_t a, size_t b, size_t c, const std::vector<Vec3>& points );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t      operator[]( unsigned char i )                    const;
   inline const Vec3& getClosest()                                     const;
   inline const Vec3& getNormal()                                      const;
   inline Vec3        getClosestPoint(const std::vector<Vec3>& points) const;
   inline real        getSqrDist()                                     const;
   inline bool        isObsolete()                                     const;
   inline bool        isClosestInternal()                              const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool        link( unsigned char edge0, EPA_Triangle* tria, unsigned char edge1 );
   inline void        silhouette( const Vec3& w, EPA_EdgeBuffer& edgeBuffer );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void        silhouette( unsigned char index, const Vec3& w, EPA_EdgeBuffer& edgeBuffer );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t         indices_[3];     //!< indices of the vertices of the triangle
   bool           obsolete_;       //!< flag to denote whether die triangle is visible from the new support point

   Vec3           closest_;        //!< the point closest to the origin of the affine hull of the triangle
   Vec3           normal_;         //!< normal pointing away from the origin
   real           bar_[3];         //!< the barycentric coordinate of closest_
   real           sqrDist_;        //!< =key; square distance of closest_ to the origin

   EPA_Triangle*  adjTriangle_[3]; //!< pointer to the triangle adjacent to edge i(=0,1,2)
   unsigned char  adjEdges_[3];    //!< for each adjoining triangle adjTriangle_[i], the index of the adjoining edge
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  EPA::EPA_TRIANGLECOMP CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * Compare class to sort the triangle heap
 */
class EPA::EPA_TriangleComp {
public:
   //**Binary function call operator***************************************************************
   /*!\name Binary function call operator */
   //@{
   inline bool operator()( const EPA_Triangle *tria1, const EPA_Triangle *tria2 );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  EPA_EDGE CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
inline EPA::EPA_Edge::EPA_Edge( EPA_Triangle* triangle, unsigned char index )
   : triangle_(triangle)
   , startIdx_(index)
{
}
//*************************************************************************************************




//=================================================================================================
//
//  EPA_EDGE GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
inline EPA::EPA_Triangle* EPA::EPA_Edge::getTriangle() const
{
   return triangle_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline unsigned char EPA::EPA_Edge::getIndex() const
{
   return startIdx_;
}
//*************************************************************************************************



//*************************************************************************************************
/*!\brief TODO
 * needs to be defined after the EPA_Triangle class
 */
inline size_t EPA::EPA_Edge::getStart() const
{
   return (*triangle_)[startIdx_];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline size_t EPA::EPA_Edge::getEnd() const
{
   return (*triangle_)[(startIdx_+1) % 3];
}
//*************************************************************************************************




//=================================================================================================
//
//  EPA::EPA_TRIANGLE CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
inline EPA::EPA_Triangle::EPA_Triangle( size_t a, size_t b, size_t c,
                                        const std::vector<Vec3>& points )
{
   pe_INTERNAL_ASSERT(a != b && b != c && c != a, "EPA_Triangle impoosible indices"); //TODO kein assert im constructor

   const Vec3& A = points[a];
   const Vec3& B = points[b];
   const Vec3& C = points[c];

   indices_[0] = a;
   indices_[1] = b;
   indices_[2] = c;

   //calculate the closest point to the origin
   //Real-Time Collsion Buch Seite 137
   Vec3 ab = B-A;
   Vec3 ac = C-A;
   //Vec3 bc = C-B;

   normal_ = ab % ac;
   Vec3T nT = trans(normal_);

   //
   real vc = nT * (A % B);
   real va = nT * (B % C);
   real vb = nT * (C % A);
   real denom = real(1.0) / (va + vb + vc);

   bar_[0] = va * denom;
   bar_[1] = vb * denom;
   bar_[2] = real(1.0) - bar_[0] - bar_[1];

   closest_ = bar_[0] * A + bar_[1] * B + bar_[2] * C;

   //sqrDist=key is square distance of v to origin
   sqrDist_ = closest_.sqrLength();

   //adjoined triangles not set jet
   adjTriangle_[0] = adjTriangle_[1] = adjTriangle_[2] = NULL;
   adjEdges_[0]    = adjEdges_[1]    = adjEdges_[2] = 4;

   obsolete_ = false;
}




//=================================================================================================
//
//  EPA::EPA_TRIANGLE GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \brief Returns the index of the internal vertex i(=0,1,2) within the EPA scope.
 */
inline size_t EPA::EPA_Triangle::operator[]( unsigned char i ) const
{
   return indices_[i];
}
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Returns the point closest to the origin of the affine hull of the triangle, which is also the normal.
 */
inline const Vec3& EPA::EPA_Triangle::getClosest() const
{
   return closest_;
}
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Returns the normal of the triangle. Normal is not normalized!
 */
inline const Vec3& EPA::EPA_Triangle::getNormal() const
{
   return normal_;
}
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Calculates the corresponding closest point form the given points, using barycentric coordinates.
 */
inline Vec3 EPA::EPA_Triangle::getClosestPoint(const std::vector<Vec3>& points) const
{
   return   bar_[0] * points[indices_[0]]
          + bar_[1] * points[indices_[1]]
          + bar_[2] * points[indices_[2]];
}
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Returns the squared distance to the closest to the origin of the affine hull of the triangle.
 */
inline real EPA::EPA_Triangle::getSqrDist() const
{
   return sqrDist_;
}
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Returns true if the triangle is no longer part of the EPA polygon.
 */
inline bool EPA::EPA_Triangle::isObsolete() const
{
   return obsolete_;
}
//*************************************************************************************************


//*************************************************************************************************
/*! Returns true if the point closest to the origin of the affine hull of the triangle, lies inside the triangle.
 */
inline bool EPA::EPA_Triangle::isClosestInternal() const
{
   return bar_[0] >= real(0.0)
            && bar_[1] >= real(0.0)
            && bar_[2] >= real(0.0);
}
//*************************************************************************************************




//=================================================================================================
//
//  EPA::EPA_TRIANGLE UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \brief TODO
 */
inline bool EPA::EPA_Triangle::link( unsigned char edge0, EPA_Triangle* tria, unsigned char edge1 )
{
   pe_INTERNAL_ASSERT(edge0 < 3 && edge1 < 3, "link: invalid edge index");

   adjTriangle_[edge0] = tria;
   adjEdges_[edge0] = edge1;
   tria->adjTriangle_[edge1] = this;
   tria->adjEdges_[edge1] = edge0;
   /*
   size_t v1 = indices_[edge0] ;
   size_t v2 = tria->indices_[(edge1+1)%3];
   size_t v3 = indices_[(edge0+1)%3];
   size_t v4 = tria->indices_[edge1];

   if( !(v1==v2 && v3==v4)) {
      std::cout << "kaputt" << std::endl;
   }
   */

   bool b = indices_[edge0]       == tria->indices_[(edge1+1)%3] &&
            indices_[(edge0+1)%3] == tria->indices_[edge1];
   return b;
}
//*************************************************************************************************


//*************************************************************************************************
/*! \brief Fills edgeBuffer with the CCW contour of triangles not seen from point w which is in normal direction of the triangle.
 */
inline void EPA::EPA_Triangle::silhouette( const Vec3& w, EPA_EdgeBuffer& edgeBuffer )
{
   edgeBuffer.clear();
   obsolete_ = true;

   adjTriangle_[0]->silhouette(adjEdges_[0], w, edgeBuffer);
   adjTriangle_[1]->silhouette(adjEdges_[1], w, edgeBuffer);
   adjTriangle_[2]->silhouette(adjEdges_[2], w, edgeBuffer);
}
//*************************************************************************************************


//*************************************************************************************************
/*! \brief TODO
 */
inline void EPA::EPA_Triangle::silhouette( unsigned char index, const Vec3& w,
                                           EPA_EdgeBuffer& edgeBuffer )
{
   if (!obsolete_) {
      real test = (trans(closest_) * w);
      // TODO
      //if ((trans(closest_) * w) < sqrDist_) {
      if (test < sqrDist_) {
         edgeBuffer.push_back(EPA_Edge(this, index));
      }
      else {
         obsolete_ = true; // Facet is visible
         int next = (index+1) % 3;
         adjTriangle_[next]->silhouette(adjEdges_[next], w, edgeBuffer);
         next = (next+1) % 3;
         adjTriangle_[next]->silhouette(adjEdges_[next], w, edgeBuffer);
      }
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  EPA::EPA_TRIANGLECOMP BINARY FUNCTION CALL OPERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
inline bool EPA::EPA_TriangleComp::operator()( const EPA_Triangle *tria1,
                                               const EPA_Triangle *tria2 )
{
   return tria1->getSqrDist() > tria2->getSqrDist();
}
//*************************************************************************************************




//=================================================================================================
//
//  EPA QUERY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type1, typename Type2 > // ID-Type of the geometry
inline bool EPA::doEPAcontactThreshold( Type1 geom1, Type2 geom2, const GJK& gjk, Vec3& retNormal,
                                        Vec3& contactPoint, real& penetrationDepth )
{
   //have in mind that we use a support mapping which blows up the objects a wee bit so
   //zero penetraion aka toching contact means that the original bodies have a distance of 2*contactTrashold between them

   //Set references to the results of GJK
   pe_INTERNAL_ASSERT( gjk.getSimplexSize() <= std::numeric_limits<unsigned char>::max(), "SimplexSize must fit unsigned char!" );
   unsigned char     numPoints( static_cast<unsigned char>( gjk.getSimplexSize() ) );
   std::vector<Vec3> epaVolume( gjk.getSimplex() );
   std::vector<Vec3> supportA ( gjk.getSupportA() );
   std::vector<Vec3> supportB ( gjk.getSupportB() );

   //Setup all the inicial valus
   Vec3 support;

   epaVolume.reserve( maxSupportPoints_ );
   supportA.reserve ( maxSupportPoints_ );
   supportB.reserve ( maxSupportPoints_ );

   EPA_EntryBuffer entryBuffer;
   entryBuffer.reserve(maxTriangles_);

   EPA_EntryHeap entryHeap;
   entryHeap.reserve(maxTriangles_);

   EPA_EdgeBuffer edgeBuffer;
   edgeBuffer.reserve(20);

   real lowerBoundSqr = 0.0;
   real upperBoundSqr = Limits<real>::inf();

   Vec3T rightDirechtionT = trans(geom2->getPosition() - geom1->getPosition());

   //create an Initial simplex
   if(numPoints == 1) {
      //Bodys are in contact, as they are grown by contactThreshold within the support function
      //they can be considered not intersecting
      return false;
   }
   else {
      createInitialSimplex< Type1, Type2 >(numPoints, geom1, geom2, supportA, supportB, epaVolume, entryBuffer); 
   }

   for(EPA_EntryBuffer::iterator it=entryBuffer.begin(); it != entryBuffer.end(); ++it) {
      if(it->isClosestInternal()) {
         //triangle is candidate
         if(it->getSqrDist() < Limits<real>::fpuAccuracy()) {
            //triangle contains the origin so we have zero penetration and touching contact.
            return false;
         }

         if(rightDirechtionT * it->getClosest() > 0){ //check if the triangle ist in the generll right direktion
            entryHeap.push_back(&(*it));
         }
      }
   }

   if(entryHeap.size() == 0) {
      //for some reason all triangles have no internal closest point
      //std::cerr << "for some reason all triangles have no internal closest point"<<std::endl;
      return false;
   }

   std::make_heap(entryHeap.begin(), entryHeap.end(), EPA::EPA_TriangleComp());

   EPA_Triangle* current = NULL;
   do {
      current = entryHeap[0];
      std::pop_heap(entryHeap.begin(), entryHeap.end(), EPA::EPA_TriangleComp());
      entryHeap.pop_back(); //TODO might be better to keep track of the end and not pop the last element but just overwrite it if needed

      if(!current->isObsolete()) {
         pe_INTERNAL_ASSERT(current->getSqrDist() > real(0.0), "EPA_Trianalge distant negativ must be possitive");
         lowerBoundSqr = current->getSqrDist();

         if(epaVolume.size() == maxSupportPoints_) {
            pe_INTERNAL_ASSERT(false, "EPA no convergece");
            //TODO wie soll ich das behandeln.
            return false;
         }

         Vec3 normal = current->getClosest().getNormalized();

         supportA.push_back(geom1->supportContactThreshold(normal));
         supportB.push_back(geom2->supportContactThreshold(-normal));
         support = supportA.back() - supportB.back();
         epaVolume.push_back(support);

         real farDist = trans(support) * current->getClosest(); //not yet squared

         pe_INTERNAL_ASSERT(farDist > real(0.0), "EPA support mapping gave invalid point in expansion direction");

         upperBoundSqr = std::min(upperBoundSqr, farDist*farDist / lowerBoundSqr);

         //terminating criteria's
         //- we found that the two bounds are close enough
         //- the added support point was already in the epaVolumn
         if( upperBoundSqr <= 1.0006 * lowerBoundSqr //TODO remove magic number
            || support == epaVolume[(*current)[0]]
         || support == epaVolume[(*current)[1]]
         || support == epaVolume[(*current)[2]])
         {
            break;
         }

         // Compute the silhouette cast by the new vertex
         // Note that the new vertex is on the positive side
         // of the current triangle, so the current triangle
         // will not be in the convex hull. Start local search
         // from this facet.

         current->silhouette(support, edgeBuffer);
         if(edgeBuffer.size() < 3 ) {
            return false;
         }

         if(entryBuffer.size() == maxSupportPoints_) {
            //"out of memory" so stop here
            break;
         }

         EPA_EdgeBuffer::const_iterator it = edgeBuffer.begin();
         entryBuffer.push_back(EPA_Triangle(it->getEnd(), it->getStart(), epaVolume.size()-1, epaVolume));

         EPA_Triangle* firstTriangle = &(entryBuffer.back());
         //if it is expanding candidate add to heap
         if(firstTriangle->isClosestInternal()
            && firstTriangle->getSqrDist() > lowerBoundSqr
            && firstTriangle->getSqrDist() < upperBoundSqr)
         {
            entryHeap.push_back(firstTriangle);
            std::push_heap(entryHeap.begin(), entryHeap.end(), EPA::EPA_TriangleComp());
         }

         firstTriangle->link(0, it->getTriangle(), it->getIndex());

         EPA_Triangle* lastTriangle = firstTriangle;

         ++it;
         for(; it != edgeBuffer.end(); ++it){
            if(entryBuffer.size() == maxSupportPoints_) {
               //"out of memory" so stop here
               break;
            }

            entryBuffer.push_back(EPA_Triangle(it->getEnd(), it->getStart(), epaVolume.size()-1, epaVolume));

            EPA_Triangle* newTriangle = &(entryBuffer.back());
            //if it is expanding candidate add to heap
            if(newTriangle->isClosestInternal()
               && newTriangle->getSqrDist() > lowerBoundSqr
               && newTriangle->getSqrDist() < upperBoundSqr
               && rightDirechtionT * newTriangle->getClosest() > 0) //check if the triangle ist in the generll right direktion
            {
               entryHeap.push_back(newTriangle);
               std::push_heap(entryHeap.begin(), entryHeap.end(), EPA::EPA_TriangleComp());
            }

            if(!newTriangle->link(0, it->getTriangle(), it->getIndex())) {
               break;
            }

            if(!newTriangle->link(2, lastTriangle, 1)) {
               break;
            }

            lastTriangle = newTriangle;
         }

         if(it != edgeBuffer.end()) {
            //For some reason the silhouette couldn't be processed completely
            //so we stop here and take the last result
            break;
         }

         firstTriangle->link(2, lastTriangle, 1);
      }
   } while (entryHeap.size() > 0 && entryHeap[0]->getSqrDist() <= upperBoundSqr);
   //} while (entryHeap.size() > 0 && lowerBoundSqr <= upperBoundSqr);

   retNormal   = -current->getClosest().getNormalized();
   //retNormal   = -current->getNormal().getNormalized();

   const Vec3 wittnesA = current->getClosestPoint(supportA);
   const Vec3 wittnesB = current->getClosestPoint(supportB);

   if( trans(retNormal) * (geom1->getPosition() - geom2->getPosition()) < 0) {
      //std::cerr << "das darf nicht sein" << std::endl;
      return false; //TODO completly wrong normal, so better not use it
   }

   contactPoint = real(0.5) * (wittnesA + wittnesB);
   penetrationDepth = -(current->getClosest().length() - real(2.0) * contactThreshold);
   /*
   std::cerr << "normal=" << normal <<std::endl;
   std::cerr << "close =" << current->getClosest() << std::endl;
   std::cerr << "diff  =" << wittnesA - wittnesB  <<std::endl;
   std::cerr << "wittnesA    =" << wittnesA <<std::endl;
   std::cerr << "wittnesB    =" << wittnesB <<std::endl;
   std::cerr << "contactPoint=" << contactPoint << std::endl;
   std::cerr << "penDepth=" << penetrationDepth  <<std::endl;
   std::cerr << "lowerBound=" << sqrt(lowerBoundSqr) <<std::endl;
   std::cerr << "curreBound=" << current->getClosest().length() << std::endl;
   std::cerr << "upperBound=" << sqrt(upperBoundSqr) <<std::endl <<std::endl;
   */
   if(penetrationDepth < contactThreshold) {
      //normal points form object2 (mesh) to object1 (geom)
      //as the wittnes points are placed within the other body the normal must be inverted
      //negative penetraionDepth means penetration

      return true;
   }
   //no intersection found!
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type > // ID-Type of the geometry which is not a triangle mesh
inline bool EPA::doEPAcontactThreshold( Type geom, TriangleMeshID mesh, const GJK& gjk,
                                        Vec3& retNormal, Vec3& contactPoint,
                                        real& penetrationDepth )
{
   return doEPAcontactThreshold<Type, TriangleMeshID>( geom, mesh, gjk, retNormal, contactPoint, penetrationDepth );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline bool EPA::doEPAcontactThreshold( TriangleMeshID mA, TriangleMeshID mB, const GJK& gjk,
                                        Vec3& retNormal, Vec3& contactPoint,
                                        real& penetrationDepth )
{
   return doEPAcontactThreshold< TriangleMeshID, TriangleMeshID >( mA, mB, gjk, retNormal, contactPoint, penetrationDepth );
}
//*************************************************************************************************




//=================================================================================================
//
//  EPA UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * see Book "collision detection in interactive 3D environments" page161
 * ATTENTION seems to have no consistent behavior on the surface and vertices
 */
inline bool EPA::originInTetrahedron( const Vec3& p0, const Vec3& p1, const Vec3& p2,
                                      const Vec3& p3 )
{
   Vec3T normal0T = trans((p1 -p0) % (p2-p0));
   if( (normal0T*p0 > 0.0) == (normal0T*p3 > 0.0) ) {
      return false;
   }
   Vec3T normal1T = trans((p2 -p1) % (p3-p1));
   if( (normal1T*p1 > 0.0) == (normal1T*p0 > 0.0) ) {
      return false;
   }
   Vec3T normal2T = trans((p3 -p2) % (p0-p2));
   if( (normal2T*p2 > 0.0) == (normal2T*p1 > 0.0) ) {
      return false;
   }
   Vec3T normal3T = trans((p0 -p3) % (p1-p3));
   if( (normal3T*p3 > 0.0) == (normal3T*p2 > 0.0) ) {
      return false;
   }

   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline bool EPA::originInTetrahedronVolumeMethod( const Vec3& A, const Vec3& B, const Vec3& C,
                                                  const Vec3& D )
{
   Vec3T aoT = trans(A);
   if((aoT * (B % C)) <= real(0.0)) {
      //if volume of ABC and Origin <0.0 than the origin is on the wrong side of ABC
      //http://mathworld.wolfram.com/Tetrahedron.html volume formula
      return false;
   }
   if((aoT * (C % D)) <= real(0.0)) {
      return false;
   }
   if((aoT * (D % B)) <= real(0.0)) {
      return false;
   }
   if((trans(B) * (D % C)) <= real(0.0)) {
      return false;
   }
   return true;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline bool EPA::pointInTetrahedron( const Vec3& A, const Vec3& B, const Vec3& C, const Vec3& D,
                                     const Vec3& point )
{
   return originInTetrahedronVolumeMethod( A-point, B-point, C-point, D-point );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 * top, frontLeft ... are indices
 */
inline void EPA::createInitialTetrahedron( size_t top, size_t frontLeft, size_t frontRight,
                                           size_t back, std::vector<Vec3>& epaVolume,
                                           EPA_EntryBuffer& entryBuffer )
{
   //insert triangle 1
   entryBuffer.push_back(EPA_Triangle(top, frontLeft, frontRight, epaVolume)); //[0] vorne
   //insert triangle 2
   entryBuffer.push_back(EPA_Triangle(top, frontRight, back, epaVolume)); //[1] rechts hinten
   //insert triangle 3
   entryBuffer.push_back(EPA_Triangle(top, back, frontLeft, epaVolume)); //[2] links hinten
   //insert triangle 4
   entryBuffer.push_back(EPA_Triangle(back, frontRight, frontLeft, epaVolume)); //[3] unten

   //make links between the triangles
   entryBuffer[0].link(0, &(entryBuffer[2]), 2); //Kante vorne links
   entryBuffer[0].link(2, &(entryBuffer[1]), 0); //Kante vorne rechts
   entryBuffer[0].link(1, &(entryBuffer[3]), 1); //kante vorne unten

   entryBuffer[1].link(2, &(entryBuffer[2]), 0); //Kante hinten
   entryBuffer[1].link(1, &(entryBuffer[3]), 0); //kante rechts unten

   entryBuffer[2].link(1, &(entryBuffer[3]), 2); //kante links unten
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type1, typename Type2 >
inline void EPA::createInitialSimplex( unsigned char numPoints, Type1 geom1, Type2 geom2,
                                       std::vector<Vec3>& supportA, std::vector<Vec3>& supportB,
                                       std::vector<Vec3>& epaVolume, EPA_EntryBuffer& entryBuffer )
{
   switch(numPoints) {
   case 2:
      {
         //simplex is a line segement
         //add 3 points around the this segment
         //the COS is konvex so the resultig hexaheadron should be konvex too

         Vec3 d = epaVolume[1] - epaVolume[0];
         //find coordinate axis e_i which is furthest form paralell to d
         //and therefor d hast the smallest abs(d[i])
         real abs0 = std::abs(d[0]);
         real abs1 = std::abs(d[1]);
         real abs2 = std::abs(d[2]);

         Vec3 axis;
         if( abs0 < abs1 && abs0 < abs2) {
            axis = Vec3(1.0, 0.0, 0.0);
         }
         else if( abs1 < abs0 && abs1 < abs2) {
            axis = Vec3(0.0, 1.0, 0.0);
         }
         else {
            axis = Vec3(0.0, 0.0, 1.0);
         }

         Vec3 direction1 = (d % axis).getNormalized();
         Quat q(d, real(2.0)/real(3.0) * M_PI);
         Rot3 rot = q.toRotationMatrix();
         Vec3 direction2 = (rot*direction1).getNormalized();
         Vec3 direction3 = (rot*direction2).getNormalized();

         //add point in positive normal direction1
         supportA.push_back(geom1->supportContactThreshold(direction1));
         supportB.push_back(geom2->supportContactThreshold(-direction1));
         Vec3 support1 = supportA.back() - supportB.back();
         epaVolume.push_back(support1); //epaVolume[2]

         //add point in negative normal direction2
         supportA.push_back(geom1->supportContactThreshold(direction2));
         supportB.push_back(geom2->supportContactThreshold(-direction2));
         Vec3 support2 = supportA.back() - supportB.back();
         epaVolume.push_back(support2); //epaVolume[3]

          //add point in negative normal direction3
         supportA.push_back(geom1->supportContactThreshold(direction3));
         supportB.push_back(geom2->supportContactThreshold(-direction3));
         Vec3 support3 = supportA.back() - supportB.back();
         epaVolume.push_back(support3); //epaVolume[4]

         // TODO check what happens if two of the new support points are identical

         //Build the hexahedron as it is convex
         //epaVolume[1] = up
         //epaVolume[0] = down
         //epaVolume[2] = ccw1
         //epaVolume[3] = ccw2
         //epaVolume[4] = ccw3


         //insert triangle 1
         entryBuffer.push_back(EPA_Triangle(1, 2, 3, epaVolume)); //[0] up->ccw1->ccw2
         //insert triangle 2
         entryBuffer.push_back(EPA_Triangle(1, 3, 4, epaVolume)); //[1] up->ccw2->ccw3
         //insert triangle 3
         entryBuffer.push_back(EPA_Triangle(1, 4, 2, epaVolume)); //[2] up->ccw3->ccw1

         //link these 3 triangles
         entryBuffer[0].link(2, &(entryBuffer[1]), 0); //edge up->ccw1
         entryBuffer[1].link(2, &(entryBuffer[2]), 0); //edge up->ccw2
         entryBuffer[2].link(2, &(entryBuffer[0]), 0); //edge up->ccw3


         //insert triangle 4
         entryBuffer.push_back(EPA_Triangle(0, 2, 4, epaVolume)); //[3] down->ccw1->ccw3
         //insert triangle 5
         entryBuffer.push_back(EPA_Triangle(0, 4, 3, epaVolume)); //[4] down->ccw3->ccw2
         //insert triangle 6
         entryBuffer.push_back(EPA_Triangle(0, 3, 2, epaVolume)); //[5] down->ccw2->ccw1

         //link these 3 triangles
         entryBuffer[3].link(2, &(entryBuffer[4]), 0); //edge down->ccw3
         entryBuffer[4].link(2, &(entryBuffer[5]), 0); //edge down->ccw1
         entryBuffer[5].link(2, &(entryBuffer[3]), 0); //edge down->ccw1

         //link the two pyramids
         entryBuffer[0].link(1, &(entryBuffer[5]), 1); //edge ccw1->ccw2
         entryBuffer[1].link(1, &(entryBuffer[4]), 1); //edge ccw2->ccw3
         entryBuffer[2].link(1, &(entryBuffer[3]), 1); //edge ccw3->ccw1

         break;
      }
   case 3:
      {
         //simplex is a triangle, add tow points in positive and negative normal direction

         const Vec3& A = epaVolume[2];  //The Point last added to the simplex
         const Vec3& B = epaVolume[1];  //One Point that was already in the simplex
         const Vec3& C = epaVolume[0];  //One Point that was already in the simplex
         //ABC is a conterclockwise triangle

         const Vec3  AB  = B-A;       //The vector A->B
         const Vec3  AC  = C-A;       //The vector A->C
         const Vec3  ABC = (AB%AC).getNormalized();     //The the normal pointing towards the viewer if he sees a CCW triangle ABC

         //add point in positive normal direction
         supportA.push_back(geom1->supportContactThreshold(ABC));
         supportB.push_back(geom2->supportContactThreshold(-ABC));
         Vec3 support1 = supportA.back() - supportB.back();
         epaVolume.push_back(support1); //epaVolume[3]

         //add point in negative normal direction
         supportA.push_back(geom1->supportContactThreshold(-ABC));
         supportB.push_back(geom2->supportContactThreshold(ABC));
         Vec3 support2 = supportA.back() - supportB.back();
         epaVolume.push_back(support2); //epaVolume[4]


         //check if the hexahedron is convex aka check if a partial tetrahedron contains the last point
         if(pointInTetrahedron(epaVolume[3], epaVolume[4], epaVolume[0], epaVolume[2], epaVolume[1])) {
            //epaVolumne[1] is whithin the tetraheadron 3-4-0-2 so this is the epaVolume to take
            createInitialTetrahedron(3,4,0,2, epaVolume, entryBuffer);
         }
         else if(pointInTetrahedron(epaVolume[3], epaVolume[4], epaVolume[1], epaVolume[0], epaVolume[2])) {
            createInitialTetrahedron(3,4,1,0, epaVolume, entryBuffer);
         }
         else if(pointInTetrahedron(epaVolume[3], epaVolume[4], epaVolume[2], epaVolume[1], epaVolume[0])) {

            createInitialTetrahedron(3,4,2,1, epaVolume, entryBuffer);
         }
         else {
            //Build the hexahedron as it is convex
            //insert triangle 1
            entryBuffer.push_back(EPA_Triangle(3, 2, 1, epaVolume)); //[0] support1->A->B
            //insert triangle 2
            entryBuffer.push_back(EPA_Triangle(3, 1, 0, epaVolume)); //[1] support1->B->C
            //insert triangle 3
            entryBuffer.push_back(EPA_Triangle(3, 0, 2, epaVolume)); //[2] support1->C->A

            //link these 3 triangles
            entryBuffer[0].link(2, &(entryBuffer[1]), 0); //edge support1->A
            entryBuffer[1].link(2, &(entryBuffer[2]), 0); //edge support1->B
            entryBuffer[2].link(2, &(entryBuffer[0]), 0); //edge support1->C


            //insert triangle 4
            entryBuffer.push_back(EPA_Triangle(4, 2, 0, epaVolume)); //[3] support2->A->C
            //insert triangle 5
            entryBuffer.push_back(EPA_Triangle(4, 0, 1, epaVolume)); //[4] support2->C->B
            //insert triangle 6
            entryBuffer.push_back(EPA_Triangle(4, 1, 2, epaVolume)); //[5] support2->B->A

            //link these 3 triangles
            entryBuffer[3].link(2, &(entryBuffer[4]), 0); //edge support2->C
            entryBuffer[4].link(2, &(entryBuffer[5]), 0); //edge support2->B
            entryBuffer[5].link(2, &(entryBuffer[3]), 0); //edge support2->A

            //link the two pyramids
            entryBuffer[0].link(1, &(entryBuffer[5]), 1); //edge A->B
            entryBuffer[1].link(1, &(entryBuffer[4]), 1); //edge B->C
            entryBuffer[2].link(1, &(entryBuffer[3]), 1); //edge C->A

         }

         break;
      }
   case 4:
      {
         createInitialTetrahedron(3,2,1,0, epaVolume, entryBuffer);
         break;
      }
   default:
      {
         pe_INTERNAL_ASSERT( false, "invalid number of simplex points in EPA" );
         break;
      }
   }
}
//*************************************************************************************************


} // namespace fine

} // namespace detection

} // namespace pe

#endif
