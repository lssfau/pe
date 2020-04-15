//=================================================================================================
/*!
 *  \file pe/core/response/FFDSolver.h
 *  \brief Fast frictional dynamics solver
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

#ifndef _PE_CORE_RESPONSE_FFDSOLVER_H_
#define _PE_CORE_RESPONSE_FFDSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <cstring>
#include <iosfwd>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/MPISection.h>
#include <pe/math/Accuracy.h>
#include <pe/math/Constants.h>
#include <pe/math/Epsilon.h>
#include <pe/math/Infinity.h>
#include <pe/math/Twist.h>
#include <pe/math/Vector3.h>
#include <pe/system/FFDConfig.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/Null.h>
#include <pe/util/NullType.h>
#include <pe/util/Vector.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Fast frictional dynamics solver.
 * \ingroup collision_response
 *
 * TODO

   \f{eqnarray}
      \textnormal{minimize}   & \quad a^T \cdot x + \frac{1}{2} \cdot x^T \cdot G \cdot x & \nonumber \\
      \textnormal{subject to} & \quad A^T \cdot x - b \geq 0 & \nonumber
   \f}

 * TODO
 */
template< typename C              // Type of the configuration
        , typename U1=NullType    // First unused auxiliary template parameter
        , typename U2=NullType >  // Second unused auxiliary template parameter
class FFDSolver : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                                Config;          //!< Type of the configuration.
   typedef FFDSolver<C,U1,U2>               This;            //!< Type of this FFDSolver instance.
   typedef typename Config::BodyType        BodyType;        //!< Type of the rigid bodies.
   typedef typename Config::BodyID          BodyID;          //!< Handle for a rigid body.
   typedef typename Config::ConstBodyID     ConstBodyID;     //!< Handle for a constant rigid body.
   typedef typename Config::ContactType     ContactType;     //!< Type of the contacts.
   typedef typename Config::ContactID       ContactID;       //!< Handle for a contact.
   typedef typename Config::ConstContactID  ConstContactID;  //!< Handle for a constant contact.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit FFDSolver( const Domain& domain );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~FFDSolver();
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   template< typename Contacts > void checkContacts  ( Contacts& contacts );
                                 void resolveContacts( BodyID body );
   //@}
   //**********************************************************************************************

private:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
          void checkContact( ContactID contact );
          void collideBody ( BodyID body, ConstContactID contact, real d,
                             const Vec3& x, const Vec3& normal, const Twist& vel, Twist& ntwist );
          bool solveCollisionQP( ConstBodyID body, const Twist& vel, Twist& result );
          bool solveFrictionQP ( ConstBodyID body, const Twist& phi, const Twist& r, Twist& result );
   //@}
   //**********************************************************************************************

   //**Functions for the Goldfarb-Idnani QP solver*************************************************
   /*!\name Functions for the Goldfarb-Idnani QP solver */
   //@{
   inline void setProblemSize( int dimension, int constraints );
          bool solve();
          bool solveInverseG();
   inline void choleskyDecomposition( real* M );
   inline void invertLowerTriangleMatrix( real* L );
          void addConstraint   ( int index );
          void removeConstraint( int index );
   //@}
   //**********************************************************************************************

   //**Access functions****************************************************************************
   /*!\name Access functions */
   //@{
   inline real&       R( int i, int j );
   inline const real& R( int i, int j ) const;
   inline real&       G( int i, int j );
   inline const real& G( int i, int j ) const;
   inline real&       A( int i, int j );
   inline const real& A( int i, int j ) const;
   //@}
   //**********************************************************************************************

   //**Debugging functions*************************************************************************
   /*!\name Debugging functions */
   //@{
   void printVector( std::ostream& os, const real* v, int m );
   void printMatrix( std::ostream& os, const real* M, int m, int n );
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   int dimension_;          //!< The current dimension of the problem.
   int constraints_;        //!< The total number of constraints of the problem.
   int size_;               //!< The current number of allocated elements.

   real* R_;                //!< The matrix of the currently active constraints.
   real* G_;                //!< The generalized mass matrix.
   real* A_;                //!< The constraint normal matrix.
   real* b_;                //!< The constraint offset vector.
   real* s_;                //!< Auxiliary variable for the Goldfarb-Idnani QP solver.
   real* u_;                //!< Auxiliary variable for the Goldfarb-Idnani QP solver.
   real* uplus_;            //!< Auxiliary variable for the Goldfarb-Idnani QP solver.

   real a_[6];              //!< Vector a of the QP problem.
   real d_[6];              //!< Auxiliary variable for the Goldfarb-Idnani QP solver.
   real r_[6];              //!< Auxiliary variable for the Goldfarb-Idnani QP solver.
   real x_[6];              //!< Solution vector of the QP problem.
   real z_[6];              //!< Auxiliary variable for the Goldfarb-Idnani QP solver.

   int q_;                  //!< Size of the active set.
   Vector<int> activeSet_;  //!< The set of currently active constraints.

   const Domain& domain_;   //!< The local process domain.
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
/*!\brief The default constructor for the FFDSolver class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
FFDSolver<C,U1,U2>::FFDSolver( const Domain& domain )
   : dimension_  (0)   // The current dimension of the problem
   , constraints_(0)   // The total number of constraints of the problem
   , size_ (0)         // The current number of allocated elements
   , R_    (NULL)      // The matrix of the currently active constraints
   , G_    (NULL)      // The generalized mass matrix
   , A_    (NULL)      // The constraint normal matrix
   , b_    (NULL)      // The constraint offset vector
   , s_    (NULL)      // Auxiliary variable for the Goldfarb-Idnani QP solver
   , u_    (NULL)      // Auxiliary variable for the Goldfarb-Idnani QP solver
   , uplus_(NULL)      // Auxiliary variable for the Goldfarb-Idnani QP solver
   , q_    (0)         // Size of the active set
   , activeSet_()      // The set of currently active constraints
   , domain_(domain)
{
   for( int i=0; i<6; ++i ) {
      a_[i] = d_[i] = r_[i] = x_[i] = z_[i] = real(0);
   }

   setProblemSize( 6, 20 );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for the FFDSolver class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
FFDSolver<C,U1,U2>::~FFDSolver()
{
   delete [] R_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of the collision constraints for the colliding rigid bodies.
 *
 * \param contacts The contacts between the colliding rigid bodies.
 * \return void
 *
 * This function checks all contact contained in the contact container \a contacts whether
 * they are colliding or separating contacts. In case a constraint violation is detected,
 * the necessary constraints are added to the two attached rigid bodies.
 */
template< typename C           // Type of the configuration
        , typename U1          // First unused auxiliary template parameter
        , typename U2 >        // Second unused auxiliary template parameter
template< typename Contacts >  // Contact container type
void FFDSolver<C,U1,U2>::checkContacts( Contacts& contacts )
{
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Resolving the " << contacts.size() << " contact(s) via the fast frictional dynamics solver...\n"
          << "      Contacts: (";
      for( typename Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c )
         log << " " << c->getID();
      log << " )";
   }

   typedef typename Contacts::Iterator  Iterator;

   const Iterator cbegin( contacts.begin() );
   const Iterator cend  ( contacts.end()   );

   for( Iterator contact=cbegin; contact!=cend; ++contact ) {
      checkContact( *contact );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resolves all constraints for the given colliding rigid body.
 *
 * \param body The colliding rigid body.
 * \return void
 *
 * This function solves the acting constraints for the colliding rigid body \a body by first
 * solving a quadratic program (QP) solely depending on the normal constraints and second by
 * solving a QP for the frictional constraints. In case the collision QP cannot be solved
 * (i.e. is infeasible), the global velocity of the rigid body is reset to zero. Note that
 * this function additionally resets all constraints on the rigid body.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void FFDSolver<C,U1,U2>::resolveContacts( BodyID body )
{
   using namespace pe::response::ffd;

   pe_INTERNAL_ASSERT( !body->isFixed() , "Contact resolution for fixed rigid body"    );
   pe_INTERNAL_ASSERT( !body->isRemote(), "Contact resolution for a remote rigid body" );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "      Resolving the contact(s) on body " << body->getID()
          << " (" << body->countConstraints() << " constraints)...";
   }

   // Calculating a frictionless collision response
   const Twist vel( body->getRelAngularVel(), body->getRelLinearVel() );
   Twist r;

   // Handling a feasible collision QP
   if( solveCollisionQP( body, vel, r ) )
   {
      // Calculating the frictional collision response
      Twist delta;

      if( friction ) {
         const Twist phi( r.angular() + vel.angular(), r.linear() + vel.linear() );
         solveFrictionQP( body, phi, r, delta );
      }

      // Adaption of the coefficient of restitution
      real cor( 0 );
      if( body->getCV() != real(0) )
         cor = body->getWeightedCV() / body->getCV();

      pe_INTERNAL_ASSERT( cor >= real(0), "Negative coefficient of resitution detected"         );
      pe_INTERNAL_ASSERT( cor <= real(1), "Coefficient of restitution larger than one detected" );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "         Coefficient of restitution: cor=" << cor;
      }

      // Calculating the new global linear and angular velocity
      Vec3 v( body->getRotation() * ( vel.linear () + delta.linear () + ( real(1.0) + cor ) * r.linear () ) );
      Vec3 w( body->getRotation() * ( vel.angular() + delta.angular() + ( real(1.0) + cor ) * r.angular() ) );

      // Smoothing the resulting global velocities
      for( size_t i=0; i<3; ++i ) {
         if( std::fabs( v[i] ) < accuracy ) v[i] = real(0);
         if( std::fabs( w[i] ) < accuracy ) w[i] = real(0);
      }

      // Setting the global velocities
      pe_LOG_DEBUG_SECTION( log ) {
         log << "         Setting the global velocity: v=" << v << ", w=" << w;
      }
      body->setGlobalVelocity( v, w );
   }

   // Handling an infeasible collision QP
   else {
      // Setting the linear and angular velocity to zero
      pe_LOG_DEBUG_SECTION( log ) {
         log << "         Infeasible collision QP, setting the global velocity to 0";
      }
      body->resetGlobalVelocity();
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of the collision constraints for the rigid bodies attached to the given contact.
 *
 * \param contact The contact.
 * \return void
 *
 * This function sets up the collision constraints for the two rigid bodies attached to contact
 * \a contact.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void FFDSolver<C,U1,U2>::checkContact( ContactID contact )
{
   // Rejecting the contact in case it is not contained in the local domain
   pe_MPI_SECTION {
      if( !domain_.ownsPoint( contact->getPosition() ) )
         return;
   }

   BodyID body1( contact->getBody1() );
   BodyID body2( contact->getBody2() );

   const Vec3 pos   ( contact->getPosition() );  // The global position of the contact
   const Vec3 normal( contact->getNormal()   );  // The normal of the contact in global coordinates

   // Transforming the global contact positions to the body frames of the attached bodies
   const Vec3 x1( body1->pointFromWFtoBF( pos ) );
   const Vec3 x2( body2->pointFromWFtoBF( pos ) );

   // Transforming the global contact normal to the body frames of the attached bodies
   const Vec3 normal1( trans( body1->getRotation() ) * normal );
         Vec3 normal2( trans( body2->getRotation() ) * normal );

   // Calculating the normal twist induced by the contact
   Twist ntwist1( body1->getInvBodyInertia() * ( x1 % normal1 ), body1->getInvMass() * normal1 );
   Twist ntwist2( body2->getInvBodyInertia() * ( x2 % normal2 ), body2->getInvMass() * normal2 );

   // Calculating the relative velocities
   const Twist vel1( body1->getRelAngularVel(), body1->getRelLinearVel() );
   const Twist vel2( body2->getRelAngularVel(), body2->getRelLinearVel() );

   real d( 0 );

   if( !body1->isFixed() && !body2->isFixed() )
   {
      // Estimating the perceived mass of the two rigid bodies
      const real m1( real(1) / ( trans(normal1) * ( ( ntwist1.angular() % x1 ) + ntwist1.linear() ) ) );
      const real m2( real(1) / ( trans(normal2) * ( ( ntwist2.angular() % x2 ) + ntwist2.linear() ) ) );

      // Calculating the contact velocities
      const real v1( trans(normal1) * ( ( vel1.angular() % x1 ) + vel1.linear() ) );
      const real v2( trans(normal2) * ( ( vel2.angular() % x2 ) + vel2.linear() ) );

      // Calculating the constraint offset
      d = ( m1*v1 + m2*v2 ) / ( m1 + m2 );
   }

   // Setup of the constraints for body 1
   if( !body1->isFixed() ) {
      collideBody( body1, contact, d, x1, normal1, vel1, ntwist1 );
   }

   // Inverting the contact normal for the second rigid body
   // The checkBody() function expects the contact normal to point towards the body it checks
   normal2 = -normal2;
   ntwist2 = -ntwist2;
   d       = -d;

   // Setup of the constraints for body 2
   if( !body2->isFixed() ) {
      collideBody( body2, contact, d, x2, normal2, vel2, ntwist2 );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of the collision constraints for the rigid body \a body.
 *
 * \param body The colliding rigid body.
 * \param contact The connecting contact.
 * \param d The constraint offset.
 * \param x The contact location (in the body frame of body 1).
 * \param normal The contact normal (in the body frame of body 1).
 * \param vel The body's velocity (in the body frame of body 1).
 * \param ntwist The normal twist induced by the contact.
 * \return void
 *
 * This function sets up the collision and frictional constraints acting on the rigid body
 * \a body due to the collision represented by the contact point \a contact.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void FFDSolver<C,U1,U2>::collideBody( BodyID body, ConstContactID contact, real d,
                                      const Vec3& x, const Vec3& normal, const Twist& vel, Twist& ntwist )
{
   using namespace pe::response::ffd;

   pe_INTERNAL_ASSERT( !body->isFixed(), "Invalid call of 'collideBody' for fixed rigid body" );

   // Adjusting the normal twist induced by the contact
   ntwist.angular() = body->getBodyInertia() * ntwist.angular();
   ntwist.linear()  = body->getMass()        * ntwist.linear() ;

   for( size_t i=0; i<6; ++i ) {
      if( std::fabs( ntwist[i] ) < accuracy ) ntwist[i] = real(0);
   }

   // Calculating the constraint violation
   const real cv( ntwist * vel - d );

   // Early exit for separating contacts
   if( cv > contactTolerance ) return;

   // Estimating the coefficient of restitution
   body->addWeightedCV( -cv * contact->getRestitution() );
   body->addCV        ( -cv );

   // Adding the normal constraint and offset to the rigid body
   body->addConstraint( ntwist );
   body->addOffset    ( d - penetrationCorrection * contact->getDistance() / TimeStep::size() );

   // Early exit in case no friction is considered
   if( !friction ) return;

   // Adding the coefficient of friction of the contact to the rigid body
   // In case the coefficient of friction is zero a very small coefficient is used instead
   const real cof( contact->getFriction() );
   body->addCoF( ( cof == real(0) )?( real(accuracy) ):( cof ) );

   // Calculating a vector tangential to the contact velocity
   Vec3 tangent( -( vel.linear() + vel.angular() % x ) );
   tangent -= ( trans(normal)*tangent ) * normal;
   for( size_t i=0; i<3; ++i ) {
      if( std::fabs( tangent[i] ) < accuracy ) tangent[i] = real(0);
   }
   tangent.normalize();

   // Creating a quaternion for the sampling process
   Quat rot( normal, (real(2)*M_PI)/frictionSamples );
   Twist s;

   // Taking samples of possible friction contributions
   for( size_t m=0; m<frictionSamples; ++m )
   {
      // Creating a new friction sample
      s.angular() = x % tangent;
      s.linear()  = tangent;

      for( int i=0; i<6; ++i ) {
         if( std::fabs( s[i] ) < accuracy ) s[i] = real(0);
      }

      // Adding the friction bound to the rigid body
      // This possibly includes the extension of the friction base for the
      // calculation of the frictional collision response.
      body->addBound( s );

      // Rotating the tangent
      tangent = rot.rotate(tangent);
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Solving the collision QP for the given rigid body.
 *
 * \param body The colliding rigid body.
 * \param vel The body's velocity (in the body frame of body 1).
 * \param result The resulting twist.
 * \return void
 *
 * Solving the collision QP for the rigid body \a body depending on the currently acting
 * normal constraints.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
bool FFDSolver<C,U1,U2>::solveCollisionQP( ConstBodyID body, const Twist& vel, Twist& result )
{
   using namespace pe::response::ffd;

   // Checking the response restriction setting
   pe_INTERNAL_ASSERT( responseRestriction >= real(0), "Negative response restriction detected" );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "         Solving the collision QP for body " << body->getID();
   }

   // Calculating the number of currently acting normal constraints
   const int constraints( body->countConstraints() );

   // Preparing the memory for the collision QP
   setProblemSize( 6, constraints );

   // Initializing the G matrix
   for( int i=0; i<36; ++i )
      G_[i] = real(0);
   const real  mass( body->getMass() );
   const Mat3& I   ( body->getBodyInertia() );
   G(3,3) = G(4,4) = G(5,5) = mass;
   G(0,0) = I(0,0);  G(0,1) = I(0,1);  G(0,2) = I(0,2);
   G(1,0) = I(1,0);  G(1,1) = I(1,1);  G(1,2) = I(1,2);
   G(2,0) = I(2,0);  G(2,1) = I(2,1);  G(2,2) = I(2,2);

   // Initializating the a vector
   Twist tmp( vel );
   tmp.angular() = I    * tmp.angular();
   tmp.linear()  = mass * tmp.linear();
   a_[0] = ( std::fabs( tmp[0] ) < accuracy ) ? ( real(0) ) : ( -tmp[0] );
   a_[1] = ( std::fabs( tmp[1] ) < accuracy ) ? ( real(0) ) : ( -tmp[1] );
   a_[2] = ( std::fabs( tmp[2] ) < accuracy ) ? ( real(0) ) : ( -tmp[2] );
   a_[3] = ( std::fabs( tmp[3] ) < accuracy ) ? ( real(0) ) : ( -tmp[3] );
   a_[4] = ( std::fabs( tmp[4] ) < accuracy ) ? ( real(0) ) : ( -tmp[4] );
   a_[5] = ( std::fabs( tmp[5] ) < accuracy ) ? ( real(0) ) : ( -tmp[5] );

   // Initializing the A matrix and the b vector
   for( int i=0; i<constraints; ++i )
   {
      const real offset( body->getOffset( i ) );
      b_[i] = ( std::fabs( offset ) < accuracy ) ? ( real(0) ) : ( offset );

      const Twist& constraint( body->getConstraint( i ) );
      A(i,0) = ( std::fabs( constraint[0] ) < accuracy ) ? ( real(0) ) : ( constraint[0] );
      A(i,1) = ( std::fabs( constraint[1] ) < accuracy ) ? ( real(0) ) : ( constraint[1] );
      A(i,2) = ( std::fabs( constraint[2] ) < accuracy ) ? ( real(0) ) : ( constraint[2] );
      A(i,3) = ( std::fabs( constraint[3] ) < accuracy ) ? ( real(0) ) : ( constraint[3] );
      A(i,4) = ( std::fabs( constraint[4] ) < accuracy ) ? ( real(0) ) : ( constraint[4] );
      A(i,5) = ( std::fabs( constraint[5] ) < accuracy ) ? ( real(0) ) : ( constraint[5] );
   }

   // Estimating the maximum step size for a bounded collision response
   real maxStep( 0 ), constLen( 0 );

   if( responseRestriction != real(0) ) {
      for( int i=0; i<constraints; ++i ) {
         const Twist& constraint( body->getConstraint( i ) );
         constLen = std::fabs( b_[i] - constraint*vel ) / constraint.length();
         if( constLen > maxStep )
            maxStep = constLen;
      }
   }

   // Solving the collision QP
   if( solve() )
   {
      for( int i=0; i<6; ++i )
         result[i] = x_[i] - vel[i];

      if( responseRestriction != real(0) ) {
         real l( result.length() );
         if( l > responseRestriction*maxStep ) {
            result *= ( responseRestriction*maxStep ) / l;
         }
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Sucessfully solved the collision QP, result = " << result;
      }

      return true;
   }

   // Handling an infeasible collision QP
   else
   {
      for( int i=0; i<6; ++i )
         result[i] = -vel[i];

      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Could not solve the collision QP, result = " << result;
      }

      return false;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Solving the friction QP for the given rigid body.
 *
 * \param body The colliding rigid body.
 * \param phi The new velocity of the rigid body without frictional forces.
 * \param r The result of the collision QP.
 * \param result The resulting twist.
 * \return void
 *
 * Solving the friction QP for the rigid body \a body depending on the currently acting
 * frictional constraints.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
bool FFDSolver<C,U1,U2>::solveFrictionQP( ConstBodyID body, const Twist& phi, const Twist& r, Twist& result )
{
   using namespace pe::response::ffd;

   // Checking that the friction handling is turned on
   pe_INTERNAL_ASSERT( friction, "Friction treatment is turned off" );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "         Solving the friction QP for body " << body->getID();
   }

   const typename BodyType::FrictionBase& frictionBase( body->getFrictionBase() );

   // Calculating the dimension and the number of constraints for the friction QP
   const int    dimension  ( body->getDimension()     );
   const size_t constraints( body->countConstraints() );

   // Early exit
   if( dimension == 0 ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Size of the friction problem is 0";
      }
      return true;
   }

   // Setting the size of the friction problem
   setProblemSize( dimension, constraints*frictionSamples );

   // Setting up the matrix G = U^T*M*U
   Twist tmp;

   for( int i=0; i<dimension; ++i ) {
      for( int j=0; j<6; ++j )
         tmp[j] = frictionBase(j,i);

      tmp.angular() = body->getBodyInertia() * tmp.angular();
      tmp.linear()  = body->getMass()        * tmp.linear();

      a_[i]  = tmp * phi;

      G(i,i) = frictionBase(0,i) * tmp[0] + frictionBase(1,i) * tmp[1] +
               frictionBase(2,i) * tmp[2] + frictionBase(3,i) * tmp[3] +
               frictionBase(4,i) * tmp[4] + frictionBase(5,i) * tmp[5];

      for( int j=i+1; j<dimension; ++j ) {
         const real sp( frictionBase(0,j) * tmp[0] + frictionBase(1,j) * tmp[1] +
                        frictionBase(2,j) * tmp[2] + frictionBase(3,j) * tmp[3] +
                        frictionBase(4,j) * tmp[4] + frictionBase(5,j) * tmp[5] );
         G(i,j) = sp;
         G(j,i) = sp;
      }
   }

   // Setting up the constraint matrix A
   pe_INTERNAL_ASSERT( body->countBounds() == constraints*frictionSamples, "Invalid number of friction bounds" );
   pe_INTERNAL_ASSERT( body->countCoFs() == body->countConstraints(), "Invalid number of friction coefficients" );

   for( size_t i=0, row=0; i<constraints; ++i ) {
      const real offset( -( body->getConstraint( i ) * r ) );
      const real invCoF( real(1)/body->getCoF( i ) );

      for( size_t j=0; j<frictionSamples; ++j, ++row ) {
         const Twist& bound( body->getBound( row ) );

         for( int col=0; col<dimension; ++col ) {
            A(row,col) = -invCoF * ( frictionBase(0,col) * bound[0] + frictionBase(1,col) * bound[1] +
                                     frictionBase(2,col) * bound[2] + frictionBase(3,col) * bound[3] +
                                     frictionBase(4,col) * bound[4] + frictionBase(5,col) * bound[5] );
         }

         b_[row] = offset;
      }
   }

   // Solving the friction QP
   if( solve() ) {
      for( int i=0; i<6; ++i ) {
         result[i] = real(0);
         for( int j=0; j<dimension_; ++j )
            result[i] += x_[j] * frictionBase(i,j);
      }

      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Sucessfully solved the friction QP, result = " << result;
      }

      return true;
   }

   // Handling an infeasible friction QP
   else {
      for( int i=0; i<6; ++i )
         result[i] = real(0);

      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Could not solve the friction QP, result = " << result;
      }

      return false;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  FUNCTIONS FOR THE GOLDFARB-IDNANI QP SOLVER
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup of the memory requirements for the current QP.
 *
 * \param dimension The dimension of the QP.
 * \param constraints The number of active constraints for the QP.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline void FFDSolver<C,U1,U2>::setProblemSize( int dimension, int constraints )
{
   pe_INTERNAL_ASSERT( dimension   > 0, "Invalid problem dimension"     );
   pe_INTERNAL_ASSERT( constraints > 0, "Invalid number of constraints" );

   // Calculating the total memory requirement of the Goldfarb-Idnani QP solver
   //  - matrix R_    : dimension*dimension
   //  - matrix G_    : dimension*dimension
   //  - matrix A_    : dimension*constraints
   //  - vector b_    : constraints
   //  - vector s_    : constraints
   //  - vector u_    : constraints
   //  - vector uplus_: constraints+1
   const int size( dimension*dimension*2 + (dimension+4)*constraints + 1 );

   // Allocating new memory
   if( size > size_ ) {
      delete [] R_;
      R_    = new real[size];
      size_ = size;
   }

   // Setting the start points of the matrices and vectors
   G_     = R_ + dimension*dimension;
   A_     = G_ + dimension*dimension;
   b_     = A_ + dimension*constraints;
   s_     = b_ + constraints;
   u_     = s_ + constraints;
   uplus_ = u_ + constraints;

   dimension_   = dimension;
   constraints_ = constraints;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Solving the initialized QP.
 *
 * \return void
 *
 * This function applies the Goldfarb-Idnani QP solver on the initialized QP problem.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
bool FFDSolver<C,U1,U2>::solve()
{
   // Performing a cholesky decomposition of the generalized mass matrix
   choleskyDecomposition( G_ );

   // Inverting the lower triangle matrix
   invertLowerTriangleMatrix( G_ );

   // Clearing the lower left part of the matrix
   for( int i=1; i<dimension_; ++i ) {
      for( int j=0; j<i; ++j ) {
         G(i,j) = real(0);
      }
   }

   // Solving the QP problem
   return solveInverseG();
}
//*************************************************************************************************


//*************************************************************************************************
// TODO
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
bool FFDSolver<C,U1,U2>::solveInverseG()
{
   real f(0), t(0), t1(0), t2(0), min(0);

   // Reinitializating the set of active constraints
   q_ = 0;
   activeSet_.clear();

   // Performing a transpose vector-matrix multiplication
   for( int i=0; i<dimension_; ++i ) {
      d_[i] = a_[0] * G_[i];
      for( int j=1; j<dimension_; ++j ) {
         d_[i] += a_[j] * G_[j*dimension_+i];
      }
   }

   // Performing a matrix-vector multiplication
   for( int i=0; i<dimension_; ++i ) {
      x_[i] = G_[i*dimension_] * d_[0];
      for( int j=1; j<dimension_; ++j ) {
         x_[i] += G_[i*dimension_+j] * d_[j];
      }
   }

   // Inverting the initial result
   for( int i=0; i<dimension_; ++i )
      x_[i] = -x_[i];

   // Performing a scalar product
   f = a_[0] * x_[0];
   for( int i=1; i<dimension_; ++i )
      f += a_[i] * x_[i];
   f += real(0.5);

   int p, l;
   real tmp;

   pe_LOG_DEBUG_SECTION( log ) {
      log << "            Initial solution x =";
      printVector( log, x_, dimension_ );
   }

   // Jump label 'step1'
 step1:
   pe_LOG_DEBUG_SECTION( log ) {
      log << "            Jump label 'step1'";
   }

   // Choosing a violated constraint
   p   = -1;
   min = -accuracy;

   for( int i=0; i<constraints_; ++i ) {
      s_[i] = A(i,0) * x_[0];
      for( int j=1; j<dimension_; ++j )
         s_[i] += A(i,j) * x_[j];
      s_[i] -= b_[i];

      if( s_[i] < min ) {
         min = s_[i];
         bool inSet( false );
         for( int j=0; j<q_; ++j ) {
            if( activeSet_[j] == i )
               inSet = true;
         }
         if( !inSet ) p = i;
      }
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( p < 0 )
         log << "            No more violated constraints!";
      else
         log << "            [q = " << q_ << "] Strongest constraint violation on constraint "
             << p << " (" << s_[p] << ")";
   }

   // Terminating the solving process
   if( p < 0 ) {
      return true;
   }

   if( q_ == 0 ) std::memset( u_, 0, sizeof(real)*constraints_ );
   else          std::memcpy( uplus_, u_, sizeof(real)*q_ );
   uplus_[q_] = 0;

   // Performing a transpose vector-matrix multiplication
   for( int i=0; i<dimension_; ++i ) {
      d_[i] = A_[p*dimension_] * G_[i];
      for( int j=1; j<dimension_; ++j ) {
         d_[i] += A_[p*dimension_+j] * G_[j*dimension_+i];
      }
   }

   // Jump label 'step2'
 step2:
   pe_LOG_DEBUG_SECTION( log ) {
      log << "            Jump label 'step2'";
   }

   pe_INTERNAL_ASSERT( q_ <= dimension_, "Invalid q value detected" );

   // Updating z
   if( q_ == dimension_ ) std::memset( z_, 0, dimension_*sizeof(real) );
   else {
      for( int i=0; i<dimension_; ++i ) {
         z_[i] = G(i,q_) * d_[q_];
         for( int j=q_+1; j<dimension_; ++j )
            z_[i] += G(i,j) * d_[j];
      }
   }

   // Updating r (r = R^-1 x d) (i.e. performing a forward solve)
   for( int i=q_-1; i>=0; --i ) {
      tmp = d_[i];
      for( int j=q_-1; j>i; --j )
         tmp -= r_[j] * R(i,j);
      r_[i] = tmp/R(i,i);
   }

   // Checking the result
   pe_LOG_DEBUG_SECTION( log ) {
      for( int i=0; i<q_; ++i ) {
         tmp = real(0);
         for( int j=0; j<q_; ++j )
            tmp += R(i,j) * r_[j];
         if( std::fabs( tmp - d_[i] ) > accuracy ) {
            log << "            Apparently infeasible problem detected...";
//             log << " q = " << q_ << "\n";
//             log << " d_ =\n";
//             printVector( log, d_, dimension_ );
//             log << " r_ =\n";
//             printVector( log, r_, dimension_ );
//             log << " R_ =\n";
//             printMatrix( log, R_, dimension_, dimension_ );
         }
      }
   }

   // Performing a scalar product between the 'p'th row of C and x
   s_[p] = -b_[p];
   for( int i=0; i<dimension_; ++i )
      s_[p] += A(p,i) * x_[i];

   // Choosing a step direction
   l  = -1;
   t1 = inf;
   t2 = inf;

   for( int j=0; j<q_; ++j ) {
      if( r_[j] > epsilon ) {
         tmp = uplus_[j]/r_[j];
         if( tmp < t1 ) {
            t1 = tmp;
            l  = j;
         }
      }
   }

   // Performing a scalar product between z and the 'p'th row of C
   tmp = z_[0] * A(p,0);
   for( int i=1; i<dimension_; ++i )
      tmp += z_[i] * A(p,i);

   pe_LOG_DEBUG_SECTION( log ) {
      log << "            z * n = " << tmp;
   }

   if( std::fabs( tmp ) > epsilon )
      t2 = -s_[p]/tmp;

   pe_LOG_DEBUG_SECTION( log ) {
      log << "            t1 = " << t1 << "\n"
             "            t2 = " << t2;
   }

   if( t1 < t2 ) t = t1;
   else t = t2;

   // Termination of the calculation due to an infeasible problem
   if( t == inf ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Infeasible collision problem, solving did not succeed!";
      }
      return false;
   }

   if( t2 == inf ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Step in dual space (t = " << t << ")";
      }
      for( int i=0; i<q_; ++i ) uplus_[i] -= t*r_[i];
      uplus_[q_] += t;
      removeConstraint( l );  // Updating H and N^*
      goto step2;
   }

   for( int i=0; i<dimension_; ++i )
      x_[i] += t * z_[i];

   f += t * tmp * ( real(0.5)*t + uplus_[q_] );

   for( int i=0; i<q_; ++i )
      uplus_[i] -= t*r_[i];
   uplus_[q_] += t;

   // t == t2
   if( t == t2 ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Full step (t = " << t << ")\n"
             << "            Solution x = ";
         printVector( log, x_, dimension_ );
      }
      addConstraint( p );  // Updating H and N^*
      goto step1;
   }

   // t == t1
   else {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "            Partial step (t = " << t << ")";
      }
      removeConstraint( l );  // Updating H and N^*
      goto step2;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Cholesky decomposition of the given matrix \a M.
 *
 * \param M The matrix to be decomposed.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline void FFDSolver<C,U1,U2>::choleskyDecomposition( real* M )
{
   real tmp;

   for( int i=0; i<dimension_; ++i )
   {
      // Treatment of the non-diagonal elements
      for( int j=0; j<i; ++j ) {
         tmp = M[i*dimension_+j];
         for( int k=0; k<j; ++k )
            tmp -= M[i*dimension_+k]*M[j*dimension_+k];
         tmp = tmp/M[j*dimension_+j];
         M[i*dimension_+j] = tmp;
      }

      // Treatment of the diagonal element
      tmp = M[i*dimension_+i];
      for( int k=0; k<i; ++k )
         tmp -= M[i*dimension_+k]*M[i*dimension_+k];

      pe_INTERNAL_ASSERT( tmp >= real(0), "Invalid matrix for cholesky decomposition" );

      M[i*dimension_+i] = std::sqrt( tmp );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inverting the lower triangular matrix \a L.
 *
 * \param L The L-matrix to be inverted.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline void FFDSolver<C,U1,U2>::invertLowerTriangleMatrix( real* L )
{
   real tmp;

   for( int tr=0; tr<dimension_; ++tr )
   {
      L[tr*dimension_+tr] = real(1)/L[tr*dimension_+tr];
      for( int i=tr+1; i<dimension_; ++i ) {
         tmp = 0;
         for( int k=tr; k<i; ++k )
            tmp -= L[i*dimension_+k]*L[tr*dimension_+k];
         tmp = tmp/L[i*dimension_+i];
         L[tr*dimension_+i] = tmp;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Adding a constraint to the current QP.
 *
 * \param index The index of the new constraint.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void FFDSolver<C,U1,U2>::addConstraint( int index )
{
   pe_LOG_DEBUG_SECTION( log ) {
      log << "            Adding the constraint '" << index << "' at q = " << q_;
   }

   real w1(0), w2(0), mu(0), omega(0), tmp1(0), tmp2(0), c(0), s(0);

   // Copying d to the new column of R
   for( int i=0; i<dimension_; ++i )
      R(i,q_) = d_[i];

   // Eliminating all but the first element from "d in R"
   for( int i=dimension_-1; i>q_; --i ) {
      w1   = R(i-1,q_);
      mu   = std::fabs( w1 );
      w2   = R(i,q_);
      tmp1 = std::fabs( w2 );
      if( tmp1 > mu ) mu = tmp1;
      if( mu == real(0) ) continue;
      tmp1   = w1/mu;
      omega  = tmp1*tmp1;
      tmp1   = w2/mu;
      omega += tmp1*tmp1;
      omega  = mu * std::sqrt(omega);
      if( w1 < 0 ) omega = -omega;
      c = w1/omega;
      s = w2/omega;
      R(i-1,q_) = omega;
      R(i  ,q_) = 0;  // TODO: not necessary - lower left ignored anyway...

      // Applying the rotation to G as well
      real v( s/(1+c) );
      for( int j=0; j<dimension_; ++j ) {
         tmp1 = G(j,i-1);
         tmp2 = G(j,i);
         G(j,i-1) = c*tmp1 + s*tmp2;
         G(j,i  ) = v*( tmp1 + G(j,i-1) )-tmp2;
      }
   }

   activeSet_.pushBack( index );
   ++q_;

   std::memcpy( u_, uplus_, sizeof(real)*q_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a constraint from the current QP.
 *
 * \param index The index of the constraint to be removed.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void FFDSolver<C,U1,U2>::removeConstraint( int index )
{
   pe_LOG_DEBUG_SECTION( log ) {
      log << "            Removing the constraint '" << index << "' at q = " << q_;
   }

   pe_INTERNAL_ASSERT( q_ > 0, "Trying to remove a constraint from an empty active set" );

   int len = q_ - index - 1;

   // Deleting the 'index'th column from R
   if( len > 0 ) {
      // TODO: actually: manually copying might even be a better idea here - overlap is known...
      for( int i=0; i<q_; ++i )
         std::memmove( R_+i*dimension_+index, R_+i*dimension_+index+1, len*sizeof(real) );
      // TODO: copy all the trailing 0's in the lower part aswell - for now
   }

   real w1(0), w2(0), mu(0), omega(0), tmp1(0), tmp2(0), c(0), s(0);

   // Eliminating all elements under the diagonal of 'T'
   for( int i=index; i<q_-1; ++i ) {
      w1     = R(i,i);
      mu     = std::fabs( w1 );
      w2     = R(i+1,i);
      tmp1   = std::fabs( w2 );
      if( tmp1 > mu ) mu = tmp1;
      tmp1   = w1/mu;
      omega  = tmp1*tmp1;
      tmp1   = w2/mu;
      omega += tmp1*tmp1;
      omega  = mu * std::sqrt(omega);
      if( w1 < real(0) ) omega = -omega;
      c = w1/omega;
      s = w2/omega;
      R(i,i) = omega;
      R(i+1,i) = real(0);  // TODO: not necessary - lower left ignored anyway...

      // Applying the rotation to the rest of R
      for( int j=i+1; j<q_-1; ++j ) {
         tmp1 = R(i,j);
         tmp2 = R(i+1,j);
         R(i,j)   = c*tmp1+s*tmp2;
         R(i+1,j) = s*tmp1-c*tmp2;
      }

      // Applying the rotation to G as well
      real v( s/(1+c) );
      for( int j=0; j<dimension_; ++j ) {
         tmp1 = G(j,i);
         tmp2 = G(j,i+1);
         G(j,i)   = c*tmp1 + s*tmp2;
         G(j,i+1) = v*( tmp1+G(j,i) ) - tmp2;
      }

      // Apply the rotation to d as well
      tmp1    = d_[i];
      tmp2    = d_[i+1];
      d_[i]   = c*tmp1 + s*tmp2;
      d_[i+1] = v*( tmp1+d_[i] ) - tmp2;
   }

   activeSet_.erase( activeSet_.begin()+index );
   memmove( uplus_+index, uplus_+index+1, sizeof(real)*(q_-index) );  // TODO: the count could actually be decreased, as only q elements are used -right?
   --q_;
}
//*************************************************************************************************




//=================================================================================================
//
//  ACCESS FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief 2D-access function for the R matrix.
 *
 * \param i Access index for the row. The index has to be in the range [0..dimension].
 * \param j Access index for the column. The index has to be in the range [0..dimension].
 * \return Reference to the accessed value.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline real& FFDSolver<C,U1,U2>::R( int i, int j )
{
   pe_INTERNAL_ASSERT( i>=0 && i<dimension_ && j>=0 && j<dimension_, "Invalid R access index" );
   return R_[i*dimension_+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access function for the R matrix.
 *
 * \param i Access index for the row. The index has to be in the range [0..dimension].
 * \param j Access index for the column. The index has to be in the range [0..dimension].
 * \return Reference to the accessed value.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline const real& FFDSolver<C,U1,U2>::R( int i, int j ) const
{
   pe_INTERNAL_ASSERT( i>=0 && i<dimension_ && j>=0 && j<dimension_, "Invalid R access index" );
   return R_[i*dimension_+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access function for the G matrix.
 *
 * \param i Access index for the row. The index has to be in the range [0..dimension].
 * \param j Access index for the column. The index has to be in the range [0..dimension].
 * \return Reference to the accessed value.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline real& FFDSolver<C,U1,U2>::G( int i, int j )
{
   pe_INTERNAL_ASSERT( i>=0 && i<dimension_ && j>=0 && j<dimension_, "Invalid G access index" );
   return G_[i*dimension_+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access function for the G matrix.
 *
 * \param i Access index for the row. The index has to be in the range [0..dimension].
 * \param j Access index for the column. The index has to be in the range [0..dimension].
 * \return Reference to the accessed value.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline const real& FFDSolver<C,U1,U2>::G( int i, int j ) const
{
   pe_INTERNAL_ASSERT( i>=0 && i<dimension_ && j>=0 && j<dimension_, "Invalid G access index" );
   return G_[i*dimension_+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access function for the A matrix.
 *
 * \param i Access index for the row. The index has to be in the range [0..constraints].
 * \param j Access index for the column. The index has to be in the range [0..dimension].
 * \return Reference to the accessed value.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline real& FFDSolver<C,U1,U2>::A( int i, int j )
{
   pe_INTERNAL_ASSERT( i>=0 && i<constraints_ && j>=0 && j<dimension_, "Invalid C access index" );
   return A_[i*dimension_+j];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief 2D-access function for the A matrix.
 *
 * \param i Access index for the row. The index has to be in the range [0..constraints].
 * \param j Access index for the column. The index has to be in the range [0..dimension].
 * \return Reference to the accessed value.
 *
 * In case pe_INTERNAL_ASSERT() is active, this access function performs an index check.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline const real& FFDSolver<C,U1,U2>::A( int i, int j ) const
{
   pe_INTERNAL_ASSERT( i>=0 && i<constraints_ && j>=0 && j<dimension_, "Invalid C access index" );
   return A_[i*dimension_+j];
}
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the given vector \a v of size \a m.
 *
 * \param os Reference to the output stream.
 * \param v The vector to be printed.
 * \param m The number of elements of the vector to be printed.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void FFDSolver<C,U1,U2>::printVector( std::ostream& os, const real* v, int m )
{
   os << "(";
   for( int i=0; i<m; ++i )
      os << " " << v[i];
   os << " )\n";
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Output of the given matrix \a A of size \f$ m \times n \f$.
 *
 * \param os Reference to the output stream.
 * \param M The matrix to be printed.
 * \param m The number of rows of the matrix to be printed.
 * \param n The number of colums of the matrix to be printed.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void FFDSolver<C,U1,U2>::printMatrix( std::ostream& os, const real* M, int m, int n )
{
   for( int i=0; i<m; ++i ) {
      os << "(";
      for( int j=0; j<n; ++j )
         os << " " << M[i*dimension_+j];
      os << " )\n";
   }
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
