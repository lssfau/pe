//=================================================================================================
/*!
 *  \file pe/core/contact/contacttrait/PolyhedralFrictionSolver.h
 *  \brief Specialization of the ContactTrait class template for the polyhedral friction solver
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

#ifndef _PE_CORE_CONTACT_CONTACTTRAIT_POLYHEDRALFRICTIONSOLVER_H_
#define _PE_CORE_CONTACT_CONTACTTRAIT_POLYHEDRALFRICTIONSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/contact/ContactBase.h>
#include <pe/core/contact/contacttrait/Default.h>
#include <pe/core/Materials.h>
#include <pe/core/response/Types.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/Types.h>
#include <pe/math/MatrixMxN.h>
#include <pe/math/shims/Square.h>
#include <pe/math/Vector3.h>
#include <pe/math/VectorN.h>
#include <pe/system/ConstraintConfig.h>
#include <pe/system/LCPConfig.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  SPECIALIZATION FOR THE POLYHEDRAL FRICTION CONSTRAINT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the ContactTrait class template for the polyhedral friction constraint solver.
 * \ingroup core
 *
 * This specialization of the ContactTrait class template adapts all contacts to the requirements
 * of the polyhedral friction constraint solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class ContactTrait< C<CD,FD,BG,response::PolyhedralFrictionSolver> > : public ContactBase
{
protected:
   //**Type definitions****************************************************************************
   typedef ContactBase  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   inline ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal, real dist );
   inline ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                        const Vec3& e1, const Vec3& e2, real dist );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~ContactTrait();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t getRows() const;
   //@}
   //**********************************************************************************************

   //**Calculation functions***********************************************************************
   /*!\name Calculation functions */
   //@{
   void calcMat     ( MatN& JLin1, MatN& JAng1, MatN& JLin2, MatN& JAng2 ) const;
   void calcErrorVec( VecN& bError )                                       const;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief PolyhedralFrictionSolver specialization of the ContactTrait constructor for vertex/face contacts.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 * \return void
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline ContactTrait< C<CD,FD,BG,response::PolyhedralFrictionSolver> >::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos,
                                                                                     const Vec3& normal, real dist )
   : ContactBase( g1, g2, gpos, normal, dist )  // Initialization of the ContactBase base object
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief PolyhedralFrictionSolver specialization of the ContactTrait constructor for edge/edge contacts.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \param e1 Edge direction of the colliding edge of the first colliding rigid body.
 * \param e2 Edge direction of the colliding edge of the second colliding rigid body.
 * \param dist The distance between the surfaces of the contacting rigid bodies.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline ContactTrait< C<CD,FD,BG,response::PolyhedralFrictionSolver> >::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal,
                                                                                     const Vec3& e1, const Vec3& e2, real dist )
   : ContactBase( g1, g2, gpos, normal, e1, e2, dist )  // Initialization of the ContactBase base object
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief PolyhedralFrictionSolver specialization of the ContactTrait destructor.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline ContactTrait< C<CD,FD,BG,response::PolyhedralFrictionSolver> >::~ContactTrait()
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rows occupied in the Jacobian matrices.
 *
 * \return The number of rows occupied in the Jacobian matrices.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t ContactTrait< C<CD,FD,BG,response::PolyhedralFrictionSolver> >::getRows() const
{
   return response::lcp::facets + size_t(2);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the matrices for a polyhedral friction contact.
 *
 * \param JLin1 The linear Jacobian matrix for body 1.
 * \param JAng1 The angular Jacobian matrix for body 1.
 * \param JLin2 The linear Jacobian matrix for body 2.
 * \param JAng2 The angular Jacobian matrix for body 2.
 * \return void
 *
 * This function calculates the Jacobian matrices for a polyhedral friction contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void ContactTrait< C<CD,FD,BG,response::PolyhedralFrictionSolver> >::calcMat( MatN& JLin1, MatN& JAng1, MatN& JLin2, MatN& JAng2 ) const{

   const Vec3 r1( b1_->getPosition() );
   const Vec3 r2( b2_->getPosition() );

   const Vec3 a1( gpos_ - r1 );
   const Vec3 a2( gpos_ - r2 );

   const size_t rows( response::lcp::facets + 2 );

   JLin1 = MatN( rows, 3, real(0) );
   JAng1 = MatN( rows, 3, real(0) );
   JLin2 = MatN( rows, 3, real(0) );
   JAng2 = MatN( rows, 3, real(0) );

   for( size_t i=0; i<3; ++i)
   {
      JLin1( 0, i ) =  normal_[i];
      JLin2( 0, i ) = -normal_[i];
   }

   JAng1( 0, 0 ) = -normal_[1] * a1[2] + normal_[2] * a1[1];
   JAng1( 0, 1 ) =  normal_[0] * a1[2] - normal_[2] * a1[0];
   JAng1( 0, 2 ) = -normal_[0] * a1[1] + normal_[1] * a1[0];

   JAng2( 0, 0 ) =  normal_[1] * a2[2] - normal_[2] * a2[1];
   JAng2( 0, 1 ) = -normal_[0] * a2[2] + normal_[2] * a2[0];
   JAng2( 0, 2 ) =  normal_[0] * a2[1] - normal_[1] * a2[0];

   for( size_t i=0; i<response::lcp::facets; ++i )
   {
      const Vec3 t( this->getTangent( i ) );

      JLin1( i+1, 0 ) = t[0];
      JLin1( i+1, 1 ) = t[1];
      JLin1( i+1, 2 ) = t[2];

      JAng1( i+1, 0 ) = -t[1] * a1[2] + t[2] * a1[1];
      JAng1( i+1, 1 ) =  t[0] * a1[2] - t[2] * a1[0];
      JAng1( i+1, 2 ) = -t[0] * a1[1] + t[1] * a1[0];

      JLin2( i+1, 0 ) = -t[0];
      JLin2( i+1, 1 ) = -t[1];
      JLin2( i+1, 2 ) = -t[2];

      JAng2( i+1, 0 ) =  t[1] * a2[2] - t[2] * a2[1];
      JAng2( i+1, 1 ) = -t[0] * a2[2] + t[2] * a2[0];
      JAng2( i+1, 2 ) =  t[0] * a2[1] - t[1] * a2[0];
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculating the error vector for a polyhedral friction contact.
 *
 * \param bError The velocity error correcting vector.
 * \return void
 *
 * This function calculates the velocity error correcting term for a polyhedral friction contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void ContactTrait< C<CD,FD,BG,response::PolyhedralFrictionSolver> >::calcErrorVec ( VecN& bError ) const
{
   real bounce( 0 );

   if( pe::bouncing )
      bounce = getRestitution() * getNormalRelVel();

   const real error( getCorParam() * getDistance() );
   bError = VecN( response::lcp::facets + 2, real(0) );
   bError[0] = ( error - bounce );
}
//*************************************************************************************************

} // namespace pe

#endif
