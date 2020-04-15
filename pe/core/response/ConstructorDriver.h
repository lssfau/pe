//=================================================================================================
/*!
 *  \file pe/core/response/ConstructorDriver.h
 *  \brief Constructs system matrices and right-hand-sides for contact problems.
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

#ifndef _PE_CORE_RESPONSE_CONSTRUCTORDRIVER_H_
#define _PE_CORE_RESPONSE_CONSTRUCTORDRIVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <set>
#include <vector>
#include <pe/core/TimeStep.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/NonCreatable.h>
#include <pe/util/Types.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 * \ingroup collision_response
 *
 * TODO
 */
template< typename C >  // Type of the configuration
class ConstructorDriver : private NonCreatable
{
public:
   //**Type definitions****************************************************************************
   typedef C                                Config;          //!< Type of the configuration.
   typedef ConstructorDriver<Config>        This;            //!< Type of this ConstructorDriver instance.
   typedef typename Config::BodyType        BodyType;        //!< Type of the rigid bodies.
   typedef typename Config::BodyID          BodyID;          //!< Handle to a rigid body.
   typedef typename Config::ConstBodyID     ConstBodyID;     //!< Handle for a constant rigid body.
   typedef typename Config::ContactType     ContactType;     //!< Type of the contacts.
   typedef typename Config::ContactID       ContactID;       //!< Handle to a contact.
   typedef typename Config::ConstContactID  ConstContactID;  //!< Handle to a contact.

   //! Contact container for neighboring contacts.
   typedef std::set<size_t>  Neighbors;
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename Contacts, typename Ctor >
   static void construct( const Contacts& contacts, Ctor& ctor );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 *
 * \param contacts TODO
 * \param ctor TODO
 *
 * TODO
 */
template< typename C >       // Type of the configuration
template< typename Contacts  // Contact container type
        , typename Ctor >    // Type of the system constructor
void ConstructorDriver<C>::construct( const Contacts& contacts, Ctor& ctor )
{
   const size_t n       ( contacts.size()  );
   const real   timestep( TimeStep::size() );

   // Allocating the system data
   {
      // (Over-)Estimating the number of non-zero entries in the system matrix
      std::vector<size_t> nonzeros( n );

      for( size_t i=0; i<n; ++i )
      {
         const BodyID b1( contacts[i]->getBody1() );
         const BodyID b2( contacts[i]->getBody2() );

         pe_INTERNAL_ASSERT( !b1->isFixed() || !b2->isFixed(), "Invalid contact between two fixed bodies detected" );

         if( b1->isFixed() )
            nonzeros[i] = b2->countContacts();
         else if( b2->isFixed() )
            nonzeros[i] = b1->countContacts();
         else
            nonzeros[i] = b1->countContacts() + b2->countContacts() - 1;
      }

      ctor.allocate( nonzeros );
   }

   // Allocating a temporary vector for neighboring contacts
   Neighbors neighbors;

   // Setup of the system matrix
   for( size_t i=0; i<n; ++i )
   {
      ContactID c_i( contacts[i] );
      const BodyID body_i1( c_i->getBody1() );
      const BodyID body_i2( c_i->getBody2() );

      // Collecting the neighboring contact IDs
      neighbors.clear();

      if( !body_i1->isFixed() ) {
         for( RigidBody::ContactIterator it=body_i1->beginContacts(); it!=body_i1->endContacts(); ++it )
            neighbors.insert( it->getIndex() );
      }
      if( !body_i2->isFixed() ) {
         for( RigidBody::ContactIterator it = body_i2->beginContacts(); it != body_i2->endContacts(); ++it )
            neighbors.insert( it->getIndex() );
      }

      // Constructing the i-th sparse row
      for( Neighbors::iterator nb=neighbors.find(i); nb!=neighbors.end(); ++nb )
      {
         const size_t j( *nb );
         ContactID c_j( contacts[j] );
         const BodyID body_j1( c_j->getBody1() );
         const BodyID body_j2( c_j->getBody2() );

         // TODO investigate why we get Valgrind errors when using the constructor
         //Mat3 JtMinvJ_ij( 0.0 );
         Mat3 JtMinvJ_ij;
         for( size_t p=0; p<9; ++p )
            JtMinvJ_ij[p] = 0;

         if( body_i1 == body_j1 ) {
            const Vec3 r1( c_i->getPosition() - body_i1->getPosition() );
            const Vec3 r2( c_j->getPosition() - body_j1->getPosition() );
            const real m ( body_j1->getInvMass() );
            JtMinvJ_ij -= r1 % body_j1->getInvInertia() % r2 - Mat3( m, m, m );
         }
         else if( body_i1 == body_j2 ) {
            const Vec3 r1( c_i->getPosition() - body_i1->getPosition() );
            const Vec3 r2( c_j->getPosition() - body_j2->getPosition() );
            const real m( body_j2->getInvMass() );
            JtMinvJ_ij += r1 % body_j2->getInvInertia() % r2 - Mat3( m, m, m );
         }

         if( body_i2 == body_j1 ) {
            const Vec3 r1( c_i->getPosition() - body_i2->getPosition() );
            const Vec3 r2( c_j->getPosition() - body_j1->getPosition() );
            const real m( body_j1->getInvMass() );
            JtMinvJ_ij += r1 % body_j1->getInvInertia() % r2 - Mat3( m, m, m );
         }
         else if( body_i2 == body_j2 ) {
            const Vec3 r1( c_i->getPosition() - body_i2->getPosition() );
            const Vec3 r2( c_j->getPosition() - body_j2->getPosition() );
            const real m( body_j2->getInvMass() );
            JtMinvJ_ij -= r1 % body_j2->getInvInertia() % r2 - Mat3( m, m, m );
         }

         if( i == j ) {
            ctor.insertMatrixDiag( i, c_i, JtMinvJ_ij );
         }
         else {
            pe_INTERNAL_ASSERT( j > i, "System matrix entry already inserted" );
            ctor.insertMatrixOffdiag( i, j, c_i, c_j, JtMinvJ_ij );
         }
      }
   }

   // Setup of the right-hand-side
   for( std::size_t i = 0; i < n; ++i )
   {
      ContactID c( contacts[i]   );
      const BodyID b1 ( c->getBody1() );
      const BodyID b2 ( c->getBody2() );
      Vec3 Jtb( 0 );

      if( !b1->isFixed() ) {
         const Vec3 v( b1->getLinearVel()  + timestep * ( b1->getInvMass() * b1->getForce() + Settings::gravity() ) );
         const Vec3 w( b1->getAngularVel() + timestep * ( b1->getInvInertia() * ( b1->getTorque() + ( b1->getInertia() * b1->getAngularVel() ) % b1->getAngularVel() ) ) );
         Jtb += v + w % ( c->getPosition() - b1->getPosition() );
      }
      else {
         const Vec3 v( b1->getLinearVel() );
         const Vec3 w( b1->getAngularVel() );
         Jtb += v + w % ( c->getPosition() - b1->getPosition() );
      }

      if( !b2->isFixed() ) {
         const Vec3 v( b2->getLinearVel()  + timestep * ( b2->getInvMass() * b2->getForce() + Settings::gravity() ) );
         const Vec3 w( b2->getAngularVel() + timestep * ( b2->getInvInertia() * ( b2->getTorque() + ( b2->getInertia() * b2->getAngularVel() ) % b2->getAngularVel() ) ) );
         Jtb -= v + w % ( c->getPosition() - b2->getPosition() );
      }
      else {
         const Vec3 v( b2->getLinearVel() );
         const Vec3 w( b2->getAngularVel() );
         Jtb -= v + w % ( c->getPosition() - b2->getPosition() );
      }

      ctor.insertVector( i, c, Jtb );
   }

   // System constructor post processing
   ctor.postprocess( contacts );
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
