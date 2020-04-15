//=================================================================================================
/*!
 *  \file pe/core/detection/fine/GJK.h
 *  \brief The Gilbert-Johnson-Keerthi Algorithm
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

#ifndef _PE_CORE_DETECTION_FINE_GJK_H_
#define _PE_CORE_DETECTION_FINE_GJK_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <vector>

#include <pe/math/Vector3.h>
#include <pe/util/Limits.h>



namespace pe {

namespace detection {

namespace fine {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The Gilbert-Johnson-Keerthi Algorithm.
 * \ingroup fine_collision_detection
 */
class GJK
{
public:

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline GJK();
   //@}
   //**********************************************************************************************

   //**Query functions*****************************************************************************
   /*!\name Query functions */
   //@{
   template < typename Type1 , typename Type2 >
   inline real doGJK( Type1 geom1, Type2 geom2, Vec3& normal, Vec3& contactPoint );

   template < typename Type1 , typename Type2 >
   inline bool doGJKcontactThreshold( Type1 geom1, Type2 geom2 );

   inline bool doGJKcontactThreshold( TriangleMeshID mA, TriangleMeshID mB );

   template < typename Type >
   inline bool doGJKcontactThreshold( Type geom, TriangleMeshID mesh );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const std::vector<Vec3>& getSimplex()     const;
   inline size_t                   getSimplexSize() const;
   inline const std::vector<Vec3>& getSupportA()    const;
   inline const std::vector<Vec3>& getSupportB()    const;
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool simplex2(Vec3& d); //TODO rename 0-Simplex is a line not 1-Simplex
   inline bool simplex3(Vec3& d);
   inline bool simplex4(Vec3& d);

   inline bool sameDirection   ( const Vec3T& vec1, const Vec3& vec2 ) const;
   inline bool zeroLengthVector( const Vec3& vec )                     const;
   inline real calcDistance    ( Vec3& normal, Vec3& contactPoint );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   std::vector<Vec3> simplex_;   //<! Container to hold the simplex.
   std::vector<Vec3> supportA_;  //<! Container to hold the support points generated in triangle mesh mA
   std::vector<Vec3> supportB_;  //<! Container to hold the support points generated in triangle mesh mB
   unsigned char     numPoints_; //<! Current number of points in the simplex.
   Vec3              d_;         //<! The next search direction.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
inline GJK::GJK() : simplex_(4), supportA_(4), supportB_(4), numPoints_(0)
{
   //d = theWorld()->getGravity(); //the gravity direction might be a good start direction as objects
   //tend to get into contact from falling/resting on each other
   //theWorld() is not accessible for some reason

   d_ = Vec3(real(0.0),real(0.6),real(0.8)); //Vec3(0.0, 3.0, 4.0)/5.0 just start with any vector of length 1
}
//*************************************************************************************************




//=================================================================================================
//
//  QUERY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * returns distance between geom1 and geaom2 or 0.0 if they are intersecting
 */
template < typename Type1 , typename Type2 >
inline real GJK::doGJK(Type1 geom1, Type2 geom2, Vec3& normal, Vec3& contactPoint)
{


   //Variables
   Vec3 support;     //the current support point
   real ret;         //return value aka distance between geom1 and geom2


   ////////////////////////////////////////////////////////////////////////
   //Initial initialisation step
   ret = 0.0;
   supportA_.resize(4);
   supportB_.resize(4);
   simplex_.resize(4);

   //get any first support point
   if(numPoints_ != 0) {
      d_.normalize();
   }
   supportA_[0] = geom1->support(d_);
   supportB_[0] = geom2->support(-d_);
   support = supportA_[0] - supportB_[0];


   //add this point to the simplex_
   simplex_[0] = support;
   numPoints_ = 1;

   //first real search direction is in the opposite direction of the first support point
   d_ = -support;

/*
   if(trans(support) * d_ < 0.0){
      //we went as far as we could in direction 'd' but not passed the origin
      //this means the triangle mashes don't overlap
      //and as the support()-function extends the support point by contactThreshold
      //the mashes are not even close enough to be considered in contact.
      return false;
   }
   */
   ////////////////////////////////////////////////////////////////////////
   //GJK main loop
   while (true) {
      //get the support point in the current search direction
      d_.normalize();
      supportA_[numPoints_] = geom1->support(d_);
      supportB_[numPoints_] = geom2->support(-d_);
      support = supportA_[numPoints_] - supportB_[numPoints_];


      //check if "support" is passed the origin in search direction
      if(trans(support) * d_ < 0.0){
         //we went as far as we could in direction 'd' but not passed the origin
         //this means the triangle mashes don't overlap

         //calc distance simplex to Origin
         ret = calcDistance(normal, contactPoint);

         return ret;
      }

      //add the new support point into the simplex
      simplex_[numPoints_] = support;
      numPoints_++;

      ////////////////////////////////////////////////////////////////
      //check if the origin is in the simplex
      //if it is the triangle mashes are overlapping
      switch(numPoints_)
      {
      case 2:
         {
            if(simplex2(d_)) {
               simplex_.pop_back();
               simplex_.pop_back();
               supportA_.pop_back();
               supportA_.pop_back();
               supportB_.pop_back();
               supportB_.pop_back();
               return ret;
            }
         }
         break;

      case 3:
         {
            if(simplex3(d_)) {
               simplex_.pop_back();
               supportA_.pop_back();
               supportB_.pop_back();
               return ret;
            }
         }
         break;

      case 4:
         {
            if(simplex4(d_)) {
               return ret;
            }
         }
         break;

      default:
         {
            //std::cerr << "numPoints_="<< numPoints_ <<std::endl;
            pe_INTERNAL_ASSERT( false, "Number of points in the simplex is not 1<=n<=4" );
         }
         break;
      }
   }

   return ret; //never reach this point
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type1 , typename Type2 >
inline bool GJK::doGJKcontactThreshold(Type1 geom1, Type2 geom2)
{
   //Variables
   Vec3 support;     //the current support point

   ////////////////////////////////////////////////////////////////////////
   //Initial initialisation step
   supportA_.resize(4);
   supportB_.resize(4);
   simplex_.resize(4);

   //get any first support point
   if(numPoints_ != 0) {
      d_.normalize();
   }
   supportA_[0] = geom1->supportContactThreshold(d_);
   supportB_[0] = geom2->supportContactThreshold(-d_);
   support = supportA_[0] - supportB_[0];


   //add this point to the simplex_
   simplex_[0] = support;
   numPoints_ = 1;

   //first real search direction is in the opposite direction of the first support point
   d_ = -support;

/*
   if(trans(support) * d_ < 0.0){
         //we went as far as we could in direction 'd' but not passed the origin
         //this means the triangle mashes don't overlap
         //and as the support()-function extends the support point by contactThreshold
         //the mashes are not even close enough to be considered in contact.
         return false;
   }
   */
   ////////////////////////////////////////////////////////////////////////
   //GJK main loop
   while (true) {
      //get the support point in the current search direction
      d_.normalize();
      supportA_[numPoints_] = geom1->supportContactThreshold(d_);
      supportB_[numPoints_] = geom2->supportContactThreshold(-d_);
      support = supportA_[numPoints_] - supportB_[numPoints_];


      //check if "support" is passed the origin in search direction
      if(trans(support) * d_ < 0.0){
         //we went as far as we could in direction 'd' but not passed the origin
         //this means the triangle mashes don't overlap
         //and as the support()-function extends the support point by contactThreshold
         //the mashes are not even close enough to be considered in contact.
         return false;
      }

      //add the new support point into the simplex
      simplex_[numPoints_] = support;
      numPoints_++;

      ////////////////////////////////////////////////////////////////
      //check if the origin is in the simplex
      //if it is the triangle mashes are overlapping
      switch(numPoints_)
      {
      case 2:
         {
            if(simplex2(d_)) {
               simplex_.pop_back();
               simplex_.pop_back();
               supportA_.pop_back();
               supportA_.pop_back();
               supportB_.pop_back();
               supportB_.pop_back();
               return true;
            }
         }
         break;

      case 3:
         {
            if(simplex3(d_)) {
               simplex_.pop_back();
               supportA_.pop_back();
               supportB_.pop_back();
               return true;
            }
         }
         break;

      case 4:
         {
            if(simplex4(d_)) {
               return true;
            }
         }
         break;

      default:
         {
            //std::cerr << "numPoints_="<< numPoints_ <<std::endl;
            pe_INTERNAL_ASSERT( false, "Number of points in the simplex is not 1<=n<=4" );
         }
         break;
      }
   }

   return false; //never reach this point
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline bool GJK::doGJKcontactThreshold( TriangleMeshID mA, TriangleMeshID mB )
{
   return doGJKcontactThreshold<TriangleMeshID, TriangleMeshID>(mA, mB);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
template < typename Type > // ID-Type of the geometry which is not a triangle mesh
inline bool GJK::doGJKcontactThreshold( Type geom, TriangleMeshID mesh )
{
   return doGJKcontactThreshold<Type, TriangleMeshID>(geom, mesh);
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
inline const std::vector<Vec3>& GJK::getSimplex() const
{
   return simplex_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline size_t GJK::getSimplexSize() const
{
   return numPoints_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline const std::vector<Vec3>& GJK::getSupportA() const
{
   return supportA_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline const std::vector<Vec3>& GJK::getSupportB() const
{
   return supportB_;
}
//*************************************************************************************************



//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 */
inline bool GJK::simplex2(Vec3& d)
{
   //the simplex is a line
   const Vec3& A = simplex_[1];  //The Point last added to the simplex
   const Vec3& B = simplex_[0];  //The Point that was already in the simplex
   const Vec3  AO  = -A;        //The vector A->O with 0 the origin
   const Vec3T AOt = trans(AO); //The transposed vector A->O with O the origin
   const Vec3  AB  = B-A;       //The vector A->B

   if( sameDirection(AOt, AB) ) {
      //The origin O is in the same direction as B is so the line AB is closest to the origin
      //=> keep A and B in the simplex
      d = AB % AO % AB;
   }
   else {
      //The origin is not in the direction of B seen from A.
      //So O lies in the voronoi region of A
      //=> simplex is just A
      simplex_[0] = A; //aka simplex_[1]
      supportA_[0] = supportA_[1];
      supportB_[0] = supportB_[1];
      numPoints_  = 1;
      d = AO;
   }

   //if the new search direction has zero length
   //than the origin is on the simplex
   if(zeroLengthVector(d)) {
      d_ = Vec3(real(0.0),real(0.6),real(0.8)); // give the GJK a chance to rerun
      return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline bool GJK::simplex3(Vec3& d)
{
   //the simplex is a triangle
   const Vec3& A = simplex_[2];  //The Point last added to the simplex
   const Vec3& B = simplex_[1];  //One Point that was already in the simplex
   const Vec3& C = simplex_[0];  //One Point that was already in the simplex
   //ABC is a conterclockwise triangle

   const Vec3  AO  = -A;        //The vector A->O with 0 the origin
   const Vec3T AOt = trans(AO); //The transposed vector A->O with O the origin
   const Vec3  AB  = B-A;       //The vector A->B
   const Vec3  AC  = C-A;       //The vector A->C
   const Vec3  ABC = AB%AC;     //The the normal pointing towards the viewer if he sees a CCW triangle ABC

   if( sameDirection(AOt, (AB % ABC)) ) {
      //Origin is on the outside of the triangle of the line AB
      if( AOt * AB > 0.0) {
         //Origin in the voronoi region of AB outside the triangle
         //=> AB is the new simplex
         simplex_[0] = B; //aka simplex_[1]
         simplex_[1] = A; //aka simplex_[2]
         supportA_[0] = supportA_[1];
         supportA_[1] = supportA_[2];
         supportB_[0] = supportB_[1];
         supportB_[1] = supportB_[2];
         numPoints_  = 2;
         d = AB % AO % AB;


      }
      else {
         //STAR
         if( sameDirection(AOt,AC) ) {
            //Origin is on a subspace of the voronio region of AC
            //=> AC is the new simplex
            //simplex_[0] = C; //aka simplex_[0] already there
            simplex_[1] = A; //aka simplex_[2]
            supportA_[1] = supportA_[2];
            supportB_[1] = supportB_[2];
            numPoints_  = 2;
            d = AC % AO % AC;
         }
         else {
            //Origin is in the voronio region of A
            //=> A is the new simplex
            simplex_[0] = A; //aka simplex_[2]
            supportA_[0] = supportA_[2];
            supportB_[0] = supportB_[2];
            numPoints_  = 1;
            d = AO;
         }
      }
   }
   else {
      if( sameDirection(AOt, (ABC % AC)) ) {
         //Origin is on the outside of the triangle of the line AC
         //STAR
         if( AOt * AC > 0.0) {
            //Origin is on a subspace of the voronio region of AC
            //=> AC is the new simplex
            //simplex_[0] = C; //aka simplex_[0] already there
            simplex_[1] = A; //aka simplex_[2]
            supportA_[1] = supportA_[2];
            supportB_[1] = supportB_[2];
            numPoints_  = 2;
            d = AC % AO % AC;
         }
         else {
            //Origin is in the voronio region of A
            //=> A is the new simplex
            simplex_[0] = A; //aka simplex_[2]
            supportA_[0] = supportA_[2];
            supportB_[0] = supportB_[2];
            numPoints_  = 1;
            d = AO;
         }
      }
      else {
         //origin is above or below the triangle ABC but its mapping on the plane ABC lies within ABC
         if( sameDirection(AOt, ABC) ) {
            //Origin is above the triangle
            //=>Keep triangle as simplex seen from the origin it is already CCW
            d = ABC;
         }
         else {
            if( sameDirection(AOt, -ABC) ) {
               //Origin is below the triangle
               //=>Keep triangle as simplex.
               //seen from the origin ABC is CW so change the winding
               Vec3 temp = B; //aka simplex_[1]
               simplex_[1] = C; //aka simplex_[0]
               simplex_[0] = temp;
               //simplex_[2] = A; //aka simplex_[2] already there
               //old simplex 2:A 1:B 0:C
               //simplex now contains 2:A 1:C 0:B

               temp = supportA_[1];
               supportA_[1] = supportA_[0];
               supportA_[0] = temp;
               temp = supportB_[1];
               supportB_[1] = supportB_[0];
               supportB_[0] = temp;

               d = -ABC;
            }
            else{
               //Origin lies in the triangle
               return true;
            }
         }
      }
   }

   //if the new search direction has zero length
   //than the origin is on the boundary of the simplex
   if(zeroLengthVector(d)) {
      d_ = Vec3(real(0.0),real(0.6),real(0.8)); // give the GJK a chance to rerun
      return true;
   }
   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline bool GJK::simplex4(Vec3& d)
{
   //the simplex is a tetrahedron
   const Vec3& A  = simplex_[3];  //The Point last added to the tetrahedron
   //t in front mens just a temp varialble
   const Vec3& B = simplex_[2];  //One Point that was already in the simplex
   const Vec3& C = simplex_[1];  //One Point that was already in the simplex
   const Vec3& D = simplex_[0];
   //BCD is a clockwise triangle wenn seen from A

   const Vec3  AO  = -A;        //The vector A->O with 0 the origin
   const Vec3T AOt = trans(AO); //The transposed vector A->O with O the origin
   const Vec3  AB  = B-A;     //The vector A->B
   const Vec3  AC  = C-A;     //The vector A->C
   const Vec3  AD  = D-A;     //The vector A-D

   //https://mollyrocket.com/forums/viewtopic.php?p=1829#1829
   unsigned char testWhere = 0;

   const Vec3 ABC = AB % AC; //The the normal pointing out of the tetrahedron towards the viewer if he sees a CCW triangle ABC
   const Vec3 ACD = AC % AD; //The the normal pointing out of the tetrahedron towards the viewer if he sees a CCW triangle ACD
   const Vec3 ADB = AD % AB; //The the normal pointing out of the tetrahedron towards the viewer if he sees a CCW triangle ADB

   if(sameDirection(AOt, ABC)) {
      testWhere |= 0x1;
   }

   if(sameDirection(AOt, ACD)) {
      testWhere |= 0x2;
   }

   if(sameDirection(AOt, ADB)) {
      testWhere |= 0x4;
   }

   switch(testWhere)
   {
   case 0:
      {
         //origin is in the tetrahedro
         //=> the two triangle mashes overlap
         //std::cout << "Origin is within the tetrahedron\nA=" << A << " B=" << B << " C="<< C << " D="<< D << std::endl;
         return true;
      } break;

   case 1:
      {
         // In front of ABC only
         //Origin is outside the tetrahedron above ABC
         //=> rearrange simplex to use the triangle case
         simplex_[0] = C;  //aka simplex_[1] 0:C
         simplex_[1] = B;  //aka simplex_[2] 1:B
         simplex_[2] = A;  //aka simplex_[3] 2:A

         supportA_[0] = supportA_[1];
         supportA_[1] = supportA_[2];
         supportA_[2] = supportA_[3];
         supportB_[0] = supportB_[1];
         supportB_[1] = supportB_[2];
         supportB_[2] = supportB_[3];

         numPoints_ = 3;

         return simplex3(d);
      } break;

   case 2:
      {
         // In front of ACD only
         //Origin is outside the tetrahedron above ACD
         //=> rearrange simplex to use the triangle case
         //simplex_[0] = D; //aka simplex_[0] 0:D already the case
         //simplex_[1] = C; //aka simplex_[1] 1:C already the case
         simplex_[2] = A;   //aka simplex_[3] 2:A

         supportA_[2] = supportA_[3];
         supportB_[2] = supportB_[3];

         numPoints_ = 3;

         return simplex3(d);
      } break;

   case 4:
      {
         // In front of ADB only
         //Origin is outside the tetrahedron above ADB
         //=> rearrange simplex to use the triangle case
         simplex_[1] = D; //aka simplex_[0] 1:D
         simplex_[0] = B; //aka simplex_[2] 0:B already there
         simplex_[2] = A; //aka simplex_[3] 2:A

         supportA_[1] = supportA_[0];
         supportA_[0] = supportA_[2];
         supportA_[2] = supportA_[3];
         supportB_[1] = supportB_[0];
         supportB_[0] = supportB_[2];
         supportB_[2] = supportB_[3];

         numPoints_ = 3;

         return simplex3(d);
      } break;

   case 3:
      {
         // In front of ABC and ACD
         if(sameDirection(AOt, ABC%AC)) {
            if(sameDirection(AOt, AC%ACD)) {
               if(sameDirection(AOt, AC)) {
                  //AddEdgeSimplex(A, C);
                  simplex_[0] = C; //aka simplex_[1] 0:C
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[0] = supportA_[1];
                  supportA_[1] = supportA_[3];
                  supportB_[0] = supportB_[1];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AC % AO % AC;
               }
               else {
                  //AddPointSimplex;
                  simplex_[0] = A; //aka simplex_[3] 0:A

                  supportA_[0] = supportA_[3];
                  supportB_[0] = supportB_[3];

                  numPoints_ = 1;
                  d = AO;
               }
            }
            else
            {
               if(sameDirection(AOt, ACD%AD)) {
                  //AddEdgeSimplex(A, D);
                  //simplex_[0] = D; //aka simplex_[0] 0:D already there
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[1] = supportA_[3];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AD % AO % AD;
               }
               else {
                  //AddTriangleSimplex(A, C, D);
                  //simplex_[0] = D; //aka simplex_[0] 0:D already there
                  //simplex_[1] = C; //aka simplex_[1] 1:C already there
                  simplex_[2] = A; //aka simplex_[3] 2:A

                  supportA_[2] = supportA_[3];
                  supportB_[2] = supportB_[3];

                  numPoints_ = 3;
                  d = ACD;
               }
            }
         }
         else
         {
            if(sameDirection(AOt, AB%ABC)) {
               if(sameDirection(AOt, AB)) {
                  //AddEdgeSimplex(A, B);
                  simplex_[0] = B; //aka simplex_[2] 0:B
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[0] = supportA_[2];
                  supportA_[1] = supportA_[3];
                  supportB_[0] = supportB_[2];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AB % AO % AB;
               }
               else {
                  //AddPointSimplex;
                  simplex_[0] = A; //aka simplex_[3] 0:A

                  supportA_[0] = supportA_[3];
                  supportB_[0] = supportB_[3];

                  numPoints_ = 1;
                  d = AO;
               }
            }
            else {
               //AddTriangleSimplex(A, B, C);
               simplex_[0] = C; //aka simplex_[1] 0:C
               simplex_[1] = B; //aka simplex_[2] 1:B
               simplex_[2] = A; //aka simplex_[3] 2:A

               supportA_[0] = supportA_[1];
               supportA_[1] = supportA_[2];
               supportA_[2] = supportA_[3];
               supportB_[0] = supportB_[1];
               supportB_[1] = supportB_[2];
               supportB_[2] = supportB_[3];

               numPoints_ = 3;
               d = ABC;
            }
         }
      } break;


   case 5:
      {
         // In front of ADB and ABC
         if(sameDirection(AOt, ADB%AB)) {
            if(sameDirection(AOt, AB%ABC)) {
               if(sameDirection(AOt, AB)) {
                  //AddEdgeSimplex(A, B);
                  simplex_[0] = B; //aka simplex_[2] 0:B
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[0] = supportA_[2];
                  supportA_[1] = supportA_[3];
                  supportB_[0] = supportB_[2];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AB % AO % AB;
               }
               else {
                  //AddPointSimplex;
                  simplex_[0] = A; //aka simplex_[3] 0:A

                  supportA_[0] = supportA_[3];
                  supportB_[0] = supportB_[3];

                  numPoints_ = 1;
                  d = AO;
               }
            }
            else
            {
               if(sameDirection(AOt, ABC%AC)) {
                  //AddEdgeSimplex(A, C);
                  simplex_[0] = C; //aka simplex_[1] 0:C
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[0] = supportA_[1];
                  supportA_[1] = supportA_[3];
                  supportB_[0] = supportB_[1];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AC % AO % AC;
               }
               else {
                  //AddTriangleSimplex(A, B, C);
                  simplex_[0] = C; //aka simplex_[1] 0:C
                  simplex_[1] = B; //aka simplex_[2] 1:B
                  simplex_[2] = A; //aka simplex_[3] 2:A

                  supportA_[0] = supportA_[1];
                  supportA_[1] = supportA_[2];
                  supportA_[2] = supportA_[3];
                  supportB_[0] = supportB_[1];
                  supportB_[1] = supportB_[2];
                  supportB_[2] = supportB_[3];

                  numPoints_ = 3;
                  d = ABC;
               }
            }
         }
         else
         {
            if(sameDirection(AOt, AD%ADB)) {
               if(sameDirection(AOt, AD)) {
                  //AddEdgeSimplex(A, D);
                  //simplex_[0] = D; //aka simplex_[0] 0:D already there
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[1] = supportA_[3];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AD % AO % AD;
               }
               else {
                  //AddPointSimplex;
                  simplex_[0] = A; //aka simplex_[3] 0:A

                  supportA_[0] = supportA_[3];
                  supportB_[0] = supportB_[3];

                  numPoints_ = 1;
                  d = AO;
               }
            }
            else {
               //AddTriangleSimplex(A, D, B);
               simplex_[1] = D; //aka simplex[0] 1:D
               simplex_[0] = B; //aka simplex[2] 0:B
               simplex_[2] = A;  //aka simplex[3] 2:A

               supportA_[1] = supportA_[0];
               supportA_[0] = supportA_[2];
               supportA_[2] = supportA_[3];
               supportB_[1] = supportB_[0];
               supportB_[0] = supportB_[2];
               supportB_[2] = supportB_[3];

               numPoints_  = 3;

               numPoints_ = 3;
               d = ADB;
            }
         }
      } break;

   case 6:
      {
         // In front of ACD and ADB
         if(sameDirection(AOt, ACD%AD)) {
            if(sameDirection(AOt, AD%ADB)) {
               if(sameDirection(AOt, AD)) {
                  //AddEdgeSimplex(A, D);
                  //simplex_[0] = D; //aka simplex_[0] 0:D already there
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[1] = supportA_[3];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AD % AO % AD;
               }
               else {
                  //AddPointSimplex;
                  simplex_[0] = A; //aka simplex_[3] 0:A

                  supportA_[0] = supportA_[3];
                  supportB_[0] = supportB_[3];

                  numPoints_ = 1;
                  d = AO;
               }
            }
            else
            {
               if(sameDirection(AOt, ADB%AB)) {
                  //AddEdgeSimplex(A, B);
                  simplex_[0] = B; //aka simplex_[2] 0:B
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[0] = supportA_[2];
                  supportA_[1] = supportA_[3];
                  supportB_[0] = supportB_[2];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AB % AO % AB;
               }
               else {
                  //AddTriangleSimplex(A, D, B);
                  simplex_[1] = D; //aka simplex[0] 1:D
                  simplex_[0] = B; //aka simplex[2] 0:B
                  simplex_[2] = A;  //aka simplex[3] 2:A

                  supportA_[1] = supportA_[0];
                  supportA_[0] = supportA_[2];
                  supportA_[2] = supportA_[3];
                  supportB_[1] = supportB_[0];
                  supportB_[0] = supportB_[2];
                  supportB_[2] = supportB_[3];

                  numPoints_  = 3;

                  numPoints_ = 3;
                  d = ADB;
               }
            }
         }
         else
         {
            if(sameDirection(AOt, AC%ACD)) {
               if(sameDirection(AOt, AC)) {
                  //AddEdgeSimplex(A, C);
                  simplex_[0] = C; //aka simplex_[1] 0:C
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[0] = supportA_[1];
                  supportA_[1] = supportA_[3];
                  supportB_[0] = supportB_[1];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AC % AO % AC;
               }
               else
               {
                  //AddPointSimplex;
                  simplex_[0] = A; //aka simplex_[3] 0:A

                  supportA_[0] = supportA_[3];
                  supportB_[0] = supportB_[3];

                  numPoints_ = 1;
                  d = AO;
               }
            }
            else
            {
               //AddTriangleSimplex(A, C, D);
               //simplex_[0] = D; //aka simplex_[0] 0:D already there
               //simplex_[1] = C; //aka simplex_[1] 1:C already there
               simplex_[2] = A; //aka simplex_[3] 2:A

               supportA_[2] = supportA_[3];
               supportB_[2] = supportB_[3];

               numPoints_ = 3;
               d = ACD;
            }
         }
      } break;

   case 7:
      {
         // In front of ABC, ACD and ADB
         if(sameDirection(AOt, AB)) {
            simplex_[0] = B; //aka simplex_[2] 0:B
            simplex_[1] = A; //aka simplex_[3] 1:A

            supportA_[0] = supportA_[2];
            supportA_[1] = supportA_[3];
            supportB_[0] = supportB_[2];
            supportB_[1] = supportB_[3];

            numPoints_ = 2;
            d = AB % AO % AB;
         }
         else
         {
            if(sameDirection(AOt, AC)) {
               simplex_[0] = C; //aka simplex_[1] 0:C
               simplex_[1] = A; //aka simplex_[3] 1:A

               supportA_[0] = supportA_[1];
               supportA_[1] = supportA_[3];
               supportB_[0] = supportB_[1];
               supportB_[1] = supportB_[3];

               numPoints_ = 2;
               d = AC % AO % AC;

            }
            else
            {
               if(sameDirection(AOt, AD)) {
                  //simplex_[0] = D; //aka simplex_[1] 0:D already there
                  simplex_[1] = A; //aka simplex_[3] 1:A

                  supportA_[1] = supportA_[3];
                  supportB_[1] = supportB_[3];

                  numPoints_ = 2;
                  d = AD % AO % AD;
               }
               else {
                  simplex_[0] = A; //aka simplex_[3] 0:A

                  supportA_[0] = supportA_[3];
                  supportB_[0] = supportB_[3];

                  numPoints_ = 1;
                  d = AO;
               }
            }
         }
      } break;
   default:
      {
         //all 8 cases 0-7 are covered
      } break;
   }

   return false;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * Checks if tow vectors roughly point in the same direction
 */
inline bool GJK::sameDirection(const Vec3T& vec1, const Vec3& vec2) const
{
   return vec1 * vec2 > real(0.0);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * Checks if the length of a vector is zero or as close to zero that it can not be distinguished form zero
 */
inline bool GJK::zeroLengthVector(const Vec3& vec) const
{
   return vec.sqrLength() < Limits<real>::fpuAccuracy();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 */
inline real GJK::calcDistance( Vec3& normal, Vec3& contactPoint )
{
   //find the point in the simplex closest to the origin#
   //its distance to the origin is the distance of the two objects
   real dist= 0.0;

   real barCoords[3] = { 0.0, 0.0, 0.0};
   real& u = barCoords[0];
   real& v = barCoords[1];
   real& w = barCoords[2];

   Vec3& A = simplex_[0];
   Vec3& B = simplex_[1];
   Vec3& C = simplex_[2];

   switch(numPoints_){
   case 1:
   {
      //the only point in simplex is closest to Origin
      dist = A.sqrLength();
      u = real(1.0);
      break;
   }
   case 2:
   {
      //calc distance Origin to line segment
      //it is definitively closest do the segment not to one of the end points
      //as the voronoi regions of the points also consist of the border borderline between
      //point region and segment region
      // compare "Real-Time Collision Detection" by Christer Ericson page 129ff
      Vec3 ab = B - A;
      //Vec3 ac = -A;
      //Vec3 bc = -simplex[1];

      //calc baryzenctric coordinats
      // compare "Real-Time Collision Detection" by Christer Ericson page 129
      //double t = trans(ac)*ab;
      real t     = real(-1.0) * trans(A)*ab;
      real denom = ab.sqrLength();
      u = t / denom;
      v = real(1.0) - u;

      // compare "Real-Time Collision Detection" by Christer Ericson page 130
      //double& e = t;
      //double& f = denom;
      //dist = ac.sqrLength() -  e*e/f;
      dist = A.sqrLength() - t*u;


      break;
   }
   case 3:
   {
      //origin is surly in the voronoi region of the face itself
      //not the bordering lines or one of the 3 points
      //to be more precise it is also in normal direction.
      // compare "Real-Time Collision Detection" by Christer Ericson page 139
      //TODO: evlt kann man das berechnen ohne den projektionspunkt zu bestimmen

      //Vec3 ab= B - A;
      //Vec3 ac= C - A;
      //Vec3 bc= C - B;
      Vec3& n = d_; //we already know the normal
      Vec3T nT = trans(n);

      real vc = nT * (A % B);
      real va = nT * (B % C);
      real vb = nT * (C % A);
      real denom = real(1.0) / (va + vb + vc);
      u = va * denom;
      v = vb * denom;
      w = real(1.0) - u - v;

      Vec3 closestPoint = u * A + v*B + w*C;
      dist = closestPoint.sqrLength();

      break;
   }
   default:
   {
      std::cout << "falsche anzahl an Punkten im simplex WTF" <<std::endl;
      break;
   }
   }

   Vec3 pointOnA = u * supportA_[0];
   Vec3 pointOnB = u * supportB_[0];
   for( int i = 1; i < numPoints_; ++i) {
      pointOnA += barCoords[i] * supportA_[i];
      pointOnB += barCoords[i] * supportB_[i];
   }

   normal = (pointOnA - pointOnB).getNormalized();
   contactPoint = (pointOnA + pointOnB) * real(0.5);


   return dist;
}
//*************************************************************************************************


} // namespace fine

} // namespace detection

} // namespace pe

#endif
