//=================================================================================================
/*!
 *  \file pe/core/response/DEMSolver.h
 *  \brief Discrete element solver
 *
 *  Copyright (C) 2012 Tobias Preclik
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

#ifndef _PE_CORE_RESPONSE_DEMSOLVER_H_
#define _PE_CORE_RESPONSE_DEMSOLVER_H_


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
#include <pe/util/logging/WarningSection.h>
#include <pe/util/logging/InfoSection.h>
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
 * TODO
 */
template< typename C              // Type of the configuration
        , typename U1=NullType    // First unused auxiliary template parameter
        , typename U2=NullType >  // Second unused auxiliary template parameter
class DEMSolver : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                                Config;          //!< Type of the configuration.
   typedef DEMSolver<C,U1,U2>               This;            //!< Type of this DEMSolver instance.
   typedef typename Config::BodyType        BodyType;        //!< Type of the rigid bodies.
   typedef typename Config::BodyID          BodyID;          //!< Handle for a rigid body.
   typedef typename Config::ConstBodyID     ConstBodyID;     //!< Handle for a constant rigid body.
   typedef typename Config::ContactType     ContactType;     //!< Type of the contacts.
   typedef typename Config::ContactID       ContactID;       //!< Handle for a contact.
   typedef typename Config::ConstContactID  ConstContactID;	 //!< Handle for a constant contact.
   typedef typename ProcessStorage<Config>::Iterator ProcessIterator;
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit DEMSolver( const Domain& domain );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~DEMSolver();
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
/*!\brief The default constructor for the DEMSolver class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
DEMSolver<C,U1,U2>::DEMSolver( const Domain& domain )
   : domain_( domain )
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor for the DEMSolver class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
DEMSolver<C,U1,U2>::~DEMSolver()
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
 * TODO
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
inline real DEMSolver<C,U1,U2>::resolveContact( ContactID c ) const
{
   const int myRank( MPISettings::rank() );
   BodyID b1( c->getBody1() );
   BodyID b2( c->getBody2() );

   if( !c->isPenetrating() ) {
      // Skip separated contacts
      pe_LOG_DEBUG_SECTION( log ) {
         log << "Rejecting separated contact.\n";
      }
      return 0;
   }

   pe_INTERNAL_ASSERT( !b1->isFixed() || !b2->isFixed(), "Invalid contact between two fixed objects." );

   /* Contact filtering rules
    *
    * L: Local body
    * G: Global body
    * R: Remote body
    *
    * Option 1:              Option 2:
    * +---+---+---+---+      +---+---+---+---+
    * |   | L | G | R |      |   | L | G | R |
    * +---+---+---+---+      +---+---+---+---+
    * | L | + | + | * |      | L |§/+| + | § |
    * +---+---+---+---+      +---+---+---+---+
    * | G | + | ~ | - |      | G | + | ~ | - |
    * +---+---+---+---+      +---+---+---+---+
    * | R | * | - | # |      | R | § | - | § |
    * +---+---+---+---+      +---+---+---+---+
    *
    *  + Accept contact unconditionally
    *  - Reject contact unconditionally
    *  * Accept contact if we own the contact point
    *  # Accept contact if we own the contact point and the owners of the involved bodies are not the same
    *  ~ Accept contact only on root process
    *  § Accept contact if we are the owner with the smallest rank witnessing the contact or if none of the owners witness the contact and we are the process with the smallest rank witnessing the contact
    *
    * Note: Local-global contacts actually require a reduction of the contact reactions applied to the global body (unless it is fixed).
    * => MPI_Allreduce for all non-fixed global bodies before time-integration.
    */

   if( !b1->isRemote() && !b2->isRemote() ) {
      // local-local, local-global, global-global contacts

      if( b1->isGlobal() && b2->isGlobal() ) {
         // Resolve global-global contacts only on root process
         if( myRank != MPISettings::root() ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Rejecting global-global contact " << *c << " on non-root process.\n";
            }
            return 0;
         }
      }
      else {
         // Always resolve local-local and local-global contacts even if they are outside of our domain
      }
   }
   else {
      // local-remote, global-remote or remote-remote contact

      if( b1->isGlobal() || b2->isGlobal() ) {
         // Never resolve remote-global contacts
         pe_LOG_DEBUG_SECTION( log ) {
            log << "Rejecting global-remote contact " << *c << ".\n";
         }
         return 0;
      }
      else if( b1->isRemote() && b2->isRemote() && b1->getOwnerRank() == b2->getOwnerRank() ) {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "Rejecting remote-remote contact since it will be a local-local contact at the owner process: " << *c << ".\n";
         }
         return 0;
      }
      else {
#if 0
         // Option 1
         if( !domain_.ownsPoint( c->getPosition() ) ) {
            pe_LOG_DEBUG_SECTION( log ) {
               if( b1->isRemote() && b2->isRemote() )
                  log << "Rejecting remote-remote contact " << *c << " since we don't own it.\n";
               else
                  log << "Rejecting remote-local contact " << *c << " since we don't own it.\n";
            }
            return 0;
         }
#else
         // Option 2
         std::vector<int> bodyHoldersA, bodyHoldersB;

         bodyHoldersA.reserve( b1->sizeProcesses() + 2 );
         bodyHoldersA.push_back( b1->getOwnerRank() );
         if( b1->isRemote() )
            bodyHoldersA.push_back( myRank );
         for( ProcessIterator it = b1->beginProcesses(); it != b1->endProcesses(); ++it )
            bodyHoldersA.push_back( it->getRank() );
         std::sort( bodyHoldersA.begin(), bodyHoldersA.end() );

         bodyHoldersB.reserve( b2->sizeProcesses() + 2 );
         bodyHoldersB.push_back( b2->getOwnerRank() );
         if( b2->isRemote() )
            bodyHoldersB.push_back( myRank );
         for( ProcessIterator it = b2->beginProcesses(); it != b2->endProcesses(); ++it )
            bodyHoldersB.push_back( it->getRank() );
         std::sort( bodyHoldersB.begin(), bodyHoldersB.end() );

         int contactOwner( std::numeric_limits<int>::max() );

         // If one of the body owners is a contact witness then make it the contact owner, if both are contact witnesses then make the owner with the smaller process rank the contact owner.
         if( b1->getOwnerRank() < b2->getOwnerRank() ) {
            if( std::binary_search( bodyHoldersB.begin(), bodyHoldersB.end(), b1->getOwnerRank() ) )
               contactOwner = b1->getOwnerRank();
            else if( std::binary_search( bodyHoldersA.begin(), bodyHoldersA.end(), b2->getOwnerRank() ) )
               contactOwner = b2->getOwnerRank();
         }
         else {
            if( std::binary_search( bodyHoldersA.begin(), bodyHoldersA.end(), b2->getOwnerRank() ) )
               contactOwner = b2->getOwnerRank();
            else if( std::binary_search( bodyHoldersB.begin(), bodyHoldersB.end(), b1->getOwnerRank() ) )
               contactOwner = b1->getOwnerRank();
         }

         // If neither of the body owners is a contact witness then choose the contact witness with smallest process rank as the contact owner.
         if( contactOwner == std::numeric_limits<int>::max() ) {
            for( std::vector<int>::iterator itA( bodyHoldersA.begin() ), itB( bodyHoldersB.begin() ); itA != bodyHoldersA.end() && itB != bodyHoldersB.end(); ( *itA < *itB ? ++itA : ++itB ) ) {
               if( *itA == *itB ) {
                  // Smallest element in intersection found.
                  contactOwner = *itA;
                  break;
               }
            }
         }

         pe_INTERNAL_ASSERT( contactOwner != std::numeric_limits<int>::max(), "No contact owner could be found." );

         // Resolve contacts between local and remote object if and only if the contact point is owned by us.
         if( myRank != contactOwner ) {
            pe_LOG_DEBUG_SECTION( log ) {
               if( b1->isRemote() && b2->isRemote() )
                  log << "Rejecting remote-remote contact " << *c << " since contact witness with process rank " << contactOwner << " will treat it.\n";
               else
                  log << "Rejecting remote-local contact " << *c << " since contact witness with process rank " << contactOwner << " will treat it.\n";
            }
            return 0;
         }
#endif
      }
   }

   // Global position of contact
   const Vec3 gpos( c->getPosition() );

   // The absolute value of the penetration length
   real delta( -c->getDistance() );

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

   return delta;
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
