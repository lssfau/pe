//=================================================================================================
/*!
 *  \file pe/core/response/ContactTrait.h
 *  \brief Header file for the ContactTrait class
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

#ifndef _PE_CORE_RESPONSE_CONTACTTRAIT_H_
#define _PE_CORE_RESPONSE_CONTACTTRAIT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/Types.h>
#include <pe/core/response/Types.h>
#include <pe/math/Constants.h>
#include <pe/math/shims/Square.h>
#include <pe/math/Vector3.h>
#include <pe/system/LCPConfig.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact customization class for the collision response.
 * \ingroup collision_response
 *
 * The ContactTrait class template is used to adapt the Contact class to the used collision
 * response algorithm. Depending on the used algorithm, a contact may require additional data
 * or functionality to efficiently support the collision response calculations.\n
 * In order to adapt the Contact class to a particular algorithm, the base template needs
 * to be specialized.
 */
template< typename C >  // Type of the configuration
class ContactTrait
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit inline ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default implementation of the ContactTrait constructor.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \return void
 *
 * Default implementation of the ContactTrait constructor.
 */
template< typename C >  // Type of the configuration
inline ContactTrait<C>::ContactTrait( GeomID /*g1*/, GeomID /*g2*/,
                                      const Vec3& /*gpos*/, const Vec3& /*normal*/ )
{}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE BOX FRICTION CONTACT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the ContactTrait class template for the box friction contact solver.
 * \ingroup collision_response
 *
 * This specialization of the ContactTrait class template adapts contacts to the box friction
 * contact solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
struct ContactTrait< C<CD,FD,BG,BoxFrictionSolver> >
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getTangentX() const;
   inline const Vec3& getTangentY() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 tangentx_;  //!< X-tangent of the approximated friction cone.
   Vec3 tangenty_;  //!< Y-tangent of the approximated friction cone.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the ContactTrait<BoxFrictionSolver> specialization.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 he second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \return void
 *
 * This constructor initializes the two tangential vectors for the friction cone of the contact.
 * The x-tangent is chosen such that is either directly pointing in the direction of the tangential
 * contact velocity or the tangential contact acceleration. If both are equal to zero, an arbitrary
 * x-tangent is chosen. The y-tangent is chosen such that the normal vector, the x-tangent and the
 * y-tangent form an orthonormal coordinate system.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
ContactTrait< C<CD,FD,BG,BoxFrictionSolver> >::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal )
{
   const Vec3 rvel( g1->velFromWF( gpos ) - g2->velFromWF( gpos ) );  // Relative velocity
   const Vec3 tvel( rvel - normal * ( trans(normal) * rvel ) );       // Tangential relative velocity

   const Vec3 racc( g1->accFromWF( gpos ) - g2->accFromWF( gpos ) );  // Relative acceleration
   const Vec3 tacc( racc - normal * ( trans(normal) * racc ) );       // Tangential relative acceleration


   /////////////////////////////////////////////////////////
   // Calculation of the orthonormal basis of the contact

   // Using the relative tangential velocity as x-tangent
   if( tvel.sqrLength() > real(1E-8) )
   {
      tangentx_ = tvel.getNormalized();
      tangenty_ = normal % tangentx_;
   }

   // Using the relative tangential acceleration as x-tangent
   else if( tacc.sqrLength() > real(1E-8) )
   {
      tangentx_ = tacc.getNormalized();
      tangenty_ = normal % tangentx_;
   }

   // Using the y-axis of the global world frame as initial guess
   else if( std::fabs( normal[0] ) > std::fabs( normal[1] ) )
   {
      const real s( real(1) / std::sqrt( sq( normal[0] ) + sq( normal[2] ) ) );

      tangentx_[0] =  s*normal[2];
      tangentx_[1] =  real(0);
      tangentx_[2] = -s*normal[0];

      tangenty_[0] =  normal[1]*tangentx_[2];
      tangenty_[1] =  normal[2]*tangentx_[0] - normal[0]*tangentx_[2];
      tangenty_[2] = -normal[1]*tangentx_[0];
   }

   // Using the x-axis of the global world frame as initial guess
   else
   {
      const real s( real(1) / std::sqrt( sq( normal[1] ) + sq( normal[2] ) ) );

      tangentx_[0] =  real(0);
      tangentx_[1] = -s*normal[2];
      tangentx_[2] =  s*normal[1];

      tangenty_[0] =  normal[1]*tangentx_[2] - normal[2]*tangentx_[1];
      tangenty_[1] = -normal[0]*tangentx_[2];
      tangenty_[2] =  normal[0]*tangentx_[1];
   }

   pe_INTERNAL_ASSERT( ( real(1) - tangentx_.sqrLength() ) < real(1E-8), "Invalid contact x tangent" );
   pe_INTERNAL_ASSERT( ( real(1) - tangenty_.sqrLength() ) < real(1E-8), "Invalid contact y tangent" );
   pe_INTERNAL_ASSERT( tangentx_ % tangenty_ == normal, "Invalid right-handed coordinate basis" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first tangent of the contact.
 *
 * \return Reference to the first tangent of the contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& ContactTrait< C<CD,FD,BG,BoxFrictionSolver> >::getTangentX() const
{
   return tangentx_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second tangent of the contact.
 *
 * \return Reference to the second tangent of the contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& ContactTrait< C<CD,FD,BG,BoxFrictionSolver> >::getTangentY() const
{
   return tangenty_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE CONE FRICTION CONTACT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the ContactTrait class template for the cone friction contact solver.
 * \ingroup collision_response
 *
 * This specialization of the ContactTrait class template adapts contacts to the cone friction
 * contact solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
struct ContactTrait< C<CD,FD,BG,ConeFrictionSolver> >
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getTangentX() const;
   inline const Vec3& getTangentY() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 tangentx_;  //!< X-tangent of the approximated friction cone.
   Vec3 tangenty_;  //!< Y-tangent of the approximated friction cone.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the ContactTrait<ConeFrictionSolver> specialization.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 he second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \return void
 *
 * This constructor initializes the two tangential vectors for the friction cone of the contact.
 * The x-tangent is chosen such that is either directly pointing in the direction of the tangential
 * contact velocity or the tangential contact acceleration. If both are equal to zero, an arbitrary
 * x-tangent is chosen. The y-tangent is chosen such that the normal vector, the x-tangent and the
 * y-tangent form an orthonormal coordinate system.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
ContactTrait< C<CD,FD,BG,ConeFrictionSolver> >::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal )
{
   const Vec3 rvel( g1->velFromWF( gpos ) - g2->velFromWF( gpos ) );  // Relative velocity
   const Vec3 tvel( rvel - normal * ( trans(normal) * rvel ) );       // Tangential relative velocity

   const Vec3 racc( g1->accFromWF( gpos ) - g2->accFromWF( gpos ) );  // Relative acceleration
   const Vec3 tacc( racc - normal * ( trans(normal) * racc ) );       // Tangential relative acceleration


   /////////////////////////////////////////////////////////
   // Calculation of the orthonormal basis of the contact

   // Using the relative tangential velocity as x-tangent
   if( tvel.sqrLength() > real(1E-8) )
   {
      tangentx_ = tvel.getNormalized();
      tangenty_ = normal % tangentx_;
   }

   // Using the relative tangential acceleration as x-tangent
   else if( tacc.sqrLength() > real(1E-8) )
   {
      tangentx_ = tacc.getNormalized();
      tangenty_ = normal % tangentx_;
   }

   // Using the y-axis of the global world frame as initial guess
   else if( std::fabs( normal[0] ) > std::fabs( normal[1] ) )
   {
      const real s( real(1) / std::sqrt( sq( normal[0] ) + sq( normal[2] ) ) );

      tangentx_[0] =  s*normal[2];
      tangentx_[1] =  real(0);
      tangentx_[2] = -s*normal[0];

      tangenty_[0] =  normal[1]*tangentx_[2];
      tangenty_[1] =  normal[2]*tangentx_[0] - normal[0]*tangentx_[2];
      tangenty_[2] = -normal[1]*tangentx_[0];
   }

   // Using the x-axis of the global world frame as initial guess
   else
   {
      const real s( real(1) / std::sqrt( sq( normal[1] ) + sq( normal[2] ) ) );

      tangentx_[0] =  real(0);
      tangentx_[1] = -s*normal[2];
      tangentx_[2] =  s*normal[1];

      tangenty_[0] =  normal[1]*tangentx_[2] - normal[2]*tangentx_[1];
      tangenty_[1] = -normal[0]*tangentx_[2];
      tangenty_[2] =  normal[0]*tangentx_[1];
   }

   pe_INTERNAL_ASSERT( ( real(1) - tangentx_.sqrLength() ) < real(1E-8), "Invalid contact x tangent" );
   pe_INTERNAL_ASSERT( ( real(1) - tangenty_.sqrLength() ) < real(1E-8), "Invalid contact y tangent" );
   pe_INTERNAL_ASSERT( tangentx_ % tangenty_ == normal, "Invalid right-handed coordinate basis" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first tangent of the contact.
 *
 * \return Reference to the first tangent of the contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& ContactTrait< C<CD,FD,BG,ConeFrictionSolver> >::getTangentX() const
{
   return tangentx_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second tangent of the contact.
 *
 * \return Reference to the second tangent of the contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& ContactTrait< C<CD,FD,BG,ConeFrictionSolver> >::getTangentY() const
{
   return tangenty_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE POLYHEDRAL FRICTION CONTACT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the ContactTrait class template for the polyhedral friction contact solver.
 * \ingroup collision_response
 *
 * This specialization of the ContactTrait class template adapts contacts to the polyhedral
 * friction contact solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
struct ContactTrait< C<CD,FD,BG,PolyhedralFrictionSolver> >
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getTangent( size_t i ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 tangents_[response::lcp::facets];  //!< Tangents of the approximated friction cone.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the ContactTrait<PolyhedralFrictionSolver> specialization.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 The second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \return void
 *
 * This constructor of the ContractTrait specialization for the polyhedral friction contact
 * solver initializes the approximated friction cone of the contact. The number of facets for
 * the approximation is configured by the pe::facets constant.
 *
 * \image html frictioncone.png
 * \image latex frictioncone.eps "Friction cone approximation for the polyhedral friction contact solver" width=700pt
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
ContactTrait< C<CD,FD,BG,PolyhedralFrictionSolver> >::ContactTrait( GeomID /*g1*/, GeomID /*g2*/, const Vec3& /*gpos*/, const Vec3& normal )
{
   using namespace pe::response::lcp;

   const real angle( ( real(2) * M_PI ) / facets );
   real x[facets];
   real y[facets];
   Vec3 tangentx, tangenty;

   for( unsigned int i=0; i<facets; ++i ) {
      x[i] = std::cos( i * angle );
      y[i] = std::sin( i * angle );
   }

   // Calculation of polyhedral friction cone
   if( std::fabs( normal[0] ) > std::fabs( normal[1] ) )
   {
      // Using the y-axis of the global world frame as initial guess

      const real s( real(1) / std::sqrt( sq( normal[0] ) + sq( normal[2] ) ) );

      tangentx[0] =  s*normal[2];
      tangentx[1] =  real(0);
      tangentx[2] = -s*normal[0];

      tangenty[0] =  normal[1]*tangentx[2];
      tangenty[1] =  normal[2]*tangentx[0] - normal[0]*tangentx[2];
      tangenty[2] = -normal[1]*tangentx[0];
   }
   else
   {
      // Using the x-axis of the global world frame as initial guess

      const real s( real(1) / std::sqrt( sq( normal[1] ) + sq( normal[2] ) ) );

      tangentx[0] =  real(0);
      tangentx[1] = -s*normal[2];
      tangentx[2] =  s*normal[1];

      tangenty[0] =  normal[1]*tangentx[2] - normal[2]*tangentx[1];
      tangenty[1] = -normal[0]*tangentx[2];
      tangenty[2] =  normal[0]*tangentx[1];
   }

   pe_INTERNAL_ASSERT( ( real(1) - tangentx.sqrLength() ) < real(1E-8), "Invalid contact x tangent" );
   pe_INTERNAL_ASSERT( ( real(1) - tangenty.sqrLength() ) < real(1E-8), "Invalid contact y tangent" );
   pe_INTERNAL_ASSERT( tangentx % tangenty == normal, "Invalid right-handed coordinate basis" );

   for( unsigned int i=0; i<facets/2; ++i ) {
      tangents_[i] = tangentx * x[i] + tangenty * y[i];
      tangents_[i+facets/2] = -tangents_[i];
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a tangent of the approximated friction cone of the contact.
 *
 * \return Reference to a tangent of the contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& ContactTrait< C<CD,FD,BG,PolyhedralFrictionSolver> >::getTangent( size_t i ) const
{
   pe_USER_ASSERT( i < response::lcp::facets, "Invalid tangent access index" );
   return tangents_[i];
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATION FOR THE OPENCL CONTACT SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the ContactTrait class template for the box friction contact solver.
 * \ingroup collision_response
 *
 * This specialization of the ContactTrait class template adapts contacts to the box friction
 * contact solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
struct ContactTrait< C<CD,FD,BG,OpenCLSolver> >
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal );
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline const Vec3& getTangentX() const;
   inline const Vec3& getTangentY() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Vec3 tangentx_;  //!< X-tangent of the approximated friction cone.
   Vec3 tangenty_;  //!< Y-tangent of the approximated friction cone.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for the ContactTrait<BoxFrictionSolver> specialization.
 *
 * \param g1 The first contacting geometric primitive.
 * \param g2 he second contacting geometric primitive.
 * \param gpos The global position of the contact.
 * \param normal The global normal of the contact (running from body 2 to body 1).
 * \return void
 *
 * This constructor initializes the two tangential vectors for the friction cone of the contact.
 * The x-tangent is chosen such that is either directly pointing in the direction of the tangential
 * contact velocity or the tangential contact acceleration. If both are equal to zero, an arbitrary
 * x-tangent is chosen. The y-tangent is chosen such that the normal vector, the x-tangent and the
 * y-tangent form an orthonormal coordinate system.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
ContactTrait< C<CD,FD,BG,OpenCLSolver> >::ContactTrait( GeomID g1, GeomID g2, const Vec3& gpos, const Vec3& normal )
{
   const Vec3 rvel( g1->velFromWF( gpos ) - g2->velFromWF( gpos ) );  // Relative velocity
   const Vec3 tvel( rvel - normal * ( trans(normal) * rvel ) );       // Tangential relative velocity

   const Vec3 racc( g1->accFromWF( gpos ) - g2->accFromWF( gpos ) );  // Relative acceleration
   const Vec3 tacc( racc - normal * ( trans(normal) * racc ) );       // Tangential relative acceleration


   /////////////////////////////////////////////////////////
   // Calculation of the orthonormal basis of the contact

   // Using the relative tangential velocity as x-tangent
   if( tvel.sqrLength() > real(1E-8) )
   {
      tangentx_ = tvel.getNormalized();
      tangenty_ = normal % tangentx_;
   }

   // Using the relative tangential acceleration as x-tangent
   else if( tacc.sqrLength() > real(1E-8) )
   {
      tangentx_ = tacc.getNormalized();
      tangenty_ = normal % tangentx_;
   }

   // Using the y-axis of the global world frame as initial guess
   else if( std::fabs( normal[0] ) > std::fabs( normal[1] ) )
   {
      const real s( real(1) / std::sqrt( sq( normal[0] ) + sq( normal[2] ) ) );

      tangentx_[0] =  s*normal[2];
      tangentx_[1] =  real(0);
      tangentx_[2] = -s*normal[0];

      tangenty_[0] =  normal[1]*tangentx_[2];
      tangenty_[1] =  normal[2]*tangentx_[0] - normal[0]*tangentx_[2];
      tangenty_[2] = -normal[1]*tangentx_[0];
   }

   // Using the x-axis of the global world frame as initial guess
   else
   {
      const real s( real(1) / std::sqrt( sq( normal[1] ) + sq( normal[2] ) ) );

      tangentx_[0] =  real(0);
      tangentx_[1] = -s*normal[2];
      tangentx_[2] =  s*normal[1];

      tangenty_[0] =  normal[1]*tangentx_[2] - normal[2]*tangentx_[1];
      tangenty_[1] = -normal[0]*tangentx_[2];
      tangenty_[2] =  normal[0]*tangentx_[1];
   }

   pe_INTERNAL_ASSERT( ( real(1) - tangentx_.sqrLength() ) < real(1E-8), "Invalid contact x tangent" );
   pe_INTERNAL_ASSERT( ( real(1) - tangenty_.sqrLength() ) < real(1E-8), "Invalid contact y tangent" );
   pe_INTERNAL_ASSERT( tangentx_ % tangenty_ == normal, "Invalid right-handed coordinate basis" );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the first tangent of the contact.
 *
 * \return Reference to the first tangent of the contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& ContactTrait< C<CD,FD,BG,OpenCLSolver> >::getTangentX() const
{
   return tangentx_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the second tangent of the contact.
 *
 * \return Reference to the second tangent of the contact.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Vec3& ContactTrait< C<CD,FD,BG,OpenCLSolver> >::getTangentY() const
{
   return tangenty_;
}
//*************************************************************************************************





} // namespace response

} // namespace pe

#endif
