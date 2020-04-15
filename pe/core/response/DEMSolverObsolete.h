//=================================================================================================
/*!
 *  \file pe/core/response/DEMSolverObsolete.h
 *  \brief Discrete element solver
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

#ifndef _PE_CORE_RESPONSE_DEMSOLVEROBSOLETE_H_
#define _PE_CORE_RESPONSE_DEMSOLVEROBSOLETE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/Materials.h>
#include <pe/core/MPISection.h>
#include <pe/math/Vector3.h>
#include <pe/system/DEMConfig.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/NullType.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Discrete element solver
 * \ingroup collision_response
 *
 * TODO The DEMSolverObsolete does nothing...
 */
template< typename C              // Type of the configuration
        , typename U1=NullType    // First unused auxiliary template parameter
        , typename U2=NullType >  // Second unused auxiliary template parameter
class DEMSolverObsolete : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                                Config;          //!< Type of the configuration.
   typedef DEMSolverObsolete<C,U1,U2>               This;            //!< Type of this DEMSolverObsolete instance.
   typedef typename Config::BodyType        BodyType;        //!< Type of the rigid bodies.
   typedef typename Config::BodyID          BodyID;          //!< Handle for a rigid body.
   typedef typename Config::ConstBodyID     ConstBodyID;     //!< Handle for a constant rigid body.
   typedef typename Config::ContactType     ContactType;     //!< Type of the contacts.
   typedef typename Config::ContactID       ContactID;       //!< Handle for a contact.
   typedef typename Config::ConstContactID  ConstContactID;	 //!< Handle for a constant contact.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit DEMSolverObsolete( const Domain& domain );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~DEMSolverObsolete();
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   inline real resolveContact( ContactID c ) const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   const Domain& domain_;             //!< The local process domain.
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U1, NullType );
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U2, NullType );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The default constructor for the DEMSolverObsolete class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
DEMSolverObsolete<C,U1,U2>::DEMSolverObsolete( const Domain& domain )
   : domain_( domain )
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for the DEMSolverObsolete class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
DEMSolverObsolete<C,U1,U2>::~DEMSolverObsolete()
{}
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resolves the given colliding contact.
 *
 * \param c The colliding contact.
 * \return The overlap of the contact.
 *
 * This function resolves the given contact according based on the approach to consider the
 * penetration depth of the two colliding rigid bodies as virtual deformation and to represent
 * this deformation with a spring-dashpot system. The normal force is either computed with a
 * linear spring-dashpot force model

   \f[ F^n = F^n_{el} + F^n_{diss} = k^n \xi + \gamma^n \dot{\xi} \f]

 * where \f$ \xi \f$ is the virtual deformation of the rigid bodies (i.e. the overlap between
 * the bodies), \f$ k^n \f$ is the stiffness of the spring-dashpot system in normal direction,
 * and \f$ \gamma^n \f$ is the viscous damping coefficient. The parameters are composed
 * of the materials' \a stiffness and \a dampingN parameters. The composite parameters are
 * derived from a series connection of the springs and dampers at the contact.
 *
 * Or alternatively the normal force is computed with a non-linear force model based on Hertz's theory:

   \f[ F^n = F^n_{el} + F^n_{diss} = \hat{k}^n \xi^{3/2} + \hat{\gamma}^n \dot{\xi} \xi^{1/2}. \f]

 * Which of the two models is used is specified by the dem::forceModel configuration parameter (for
 * more information on the normal force models see the ForceModel description). The tangential
 * force is computed with the force model by Haff and Werner:

   \f[ \vec{F}^t = -min( \mu |\vec{F}_n|, \gamma^t |\vec{V}^t| ) \frac{\vec{V}^t}{|\vec{V}^t|}, \f]

 * where \f$ V^t \f$ is the relative velocity in tangential direction, \f$ \mu \f$ is the friction
 * coefficient, and \f$ \gamma^t \f$ is a damping constant in tangential direction.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline real DEMSolverObsolete<C,U1,U2>::resolveContact( ContactID c ) const
{
   using namespace pe::response::dem;

   // Rejecting the contact in case it is not contained in the local domain
   pe_MPI_SECTION {
      if( !domain_.ownsPoint( c->getPosition() ) )
         return real(0);
   }

   real delta( 0.0 );

   if( c->isPenetrating() )
   {
      // Global position of contact
      const Vec3 gpos( c->getPosition() );

      // The absolute value of the penetration length
      delta = -c->getDistance();

      // The two bodies in contact
      BodyID b1( c->getBody1() );
      BodyID b2( c->getBody2() );

      // Calculating the relative velocity in normal and tangential direction
      // The negative signs result from the different definition of the relative
      // normal velocity of the pe (see Contact::getType)
      const real relVelN( -c->getNormalRelVel() );
      const Vec3 relVel ( -c->getRelVel() );
      const Vec3 relVelT( relVel - ( relVelN * c->getNormal() ) );

      real fNabs( 0 );
      Vec3 fN;

      // Calculating the normal force based on the non-linear extended Hertz model
      // This force model is only applied in case of a sphere/sphere collision, since only
      // then an effective radius can be computed.
      if( dem::forceModel == dem::hertz && c->hasEffectiveRadius() )
      {
         const real alpha   ( 1.5 );
         const real beta    ( 0.5 );
         const real Reff    ( c->getEffectiveRadius() );
         const real Eeff    ( c->getEffectiveYoungModulus() );
         const real k       ( ( real(4)/real(3) ) * Eeff * sqrt( Reff ) );

         fNabs = k*std::pow( delta, alpha ) + c->getDampingN()*relVelN*std::pow( delta, beta );
         if( fNabs < real(0) ) fNabs = real(0);
         fN = fNabs * c->getNormal();
      }

      // Calculating the normal force based on a linear spring-dashpot force model
      else
      {
         fNabs = c->getStiffness() * delta + c->getDampingN() * relVelN;
         if( fNabs < real(0) ) fNabs = real(0);
         fN = fNabs * c->getNormal();
      }

      // Calculating the tangential force based on the model by Haff and Werner
      const real fTabs( min( c->getDampingT() * relVelT.length(), c->getFriction() * fNabs ) );
      const Vec3 fT   ( fTabs * relVelT.getNormalized() );

      // Add normal force at contact point
      b1->addForceAtPos(  fN, gpos );
      b2->addForceAtPos( -fN, gpos );

      // Add tangential force at contact point
      b1->addForceAtPos(  fT, gpos );
      b2->addForceAtPos( -fT, gpos );
   }

   return delta;
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
