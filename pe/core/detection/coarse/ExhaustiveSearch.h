//=================================================================================================
/*!
 *  \file pe/core/detection/coarse/ExhaustiveSearch.h
 *  \brief Implementation of the 'Exhaustive Search' coarse collision detection algorithm
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

#ifndef _PE_CORE_DETECTION_COARSE_EXHAUSTIVESEARCH_H_
#define _PE_CORE_DETECTION_COARSE_EXHAUSTIVESEARCH_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NonCopyable.h>


namespace pe {

namespace detection {

namespace coarse {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of the 'Exhaustive Search' coarse collision detection algorithm.
 * \ingroup coarse_collision_detection
 *
 * The ExhaustiveSearch class represents the implementation of the 'Exhaustive Search' coarse
 * collision detection algorithm. This algorithm tests for an overlap of the axis-aligned
 * bounding boxes for each pair of rigid bodies and performs a fine collision detection in
 * case the bounding boxes overlap. Since every pair of rigid bodies is handled, the algorithm
 * has a complexity of \f$ O(n^2) \f$. Although the complexity is worse than the complexity of
 * other algorithms (compared to for example the SweepAndPrune algorithm), the ExhaustiveSearch
 * algorithm performs very good in situations where only a small number of rigid bodies is
 * required. This is because of no additional data management overhead.
 */
template< typename C >  // Type of the configuration
class ExhaustiveSearch : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                              Config;        //!< Type of the configuration.
   typedef ExhaustiveSearch<Config>       This;          //!< Type of this ExhaustiveSearch instance.
   typedef BodyStorage<Config>            BS;            //!< Type of the body storage.
   typedef typename Config::BodyID        BodyID;        //!< Handle for a rigid body.
   typedef typename Config::FineDetector  FineDetector;  //!< Type of the fine collision detector.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit ExhaustiveSearch( BS& bodystorage );
   explicit ExhaustiveSearch( BS& bodystorage, BS& bodystorageShadowCopies );
   //@}
   //**********************************************************************************************

   //**Add/remove functions************************************************************************
   /*!\name Add/remove functions */
   //@{
   inline void add   ( BodyID body );
   inline void remove( BodyID body );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename Contacts >
   void findContacts( Contacts& contacts );

   void clear();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   BS& bodystorage_;              //!< Reference to the central body storage.
   BS& bodystorageShadowCopies_;  //!< Reference to the body storage containing body shadow copies.
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( This, typename Config::CoarseDetector );
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
/*!\brief Constructor for exhaustive search collision detector.
 *
 * \param bodystorage Reference to the general body storage.
 *
 * Note that all (local, global and remote) must be contained in the body storage.
 */
template< typename C >  // Type of the configuration
ExhaustiveSearch<C>::ExhaustiveSearch( BS& bodystorage )
   : bodystorage_( bodystorage )
   , bodystorageShadowCopies_( bodystorage )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Constructor for exhaustive search collision detector.
 *
 * \param bodystorage Reference to the central body storage.
 * \param bodystorageShadowCopies Reference to the body storage containing all body shadow copies.
 */
template< typename C >  // Type of the configuration
ExhaustiveSearch<C>::ExhaustiveSearch( BS& bodystorage, BS& bodystorageShadowCopies )
   : bodystorage_( bodystorage )
     , bodystorageShadowCopies_( bodystorageShadowCopies )
{}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a rigid body with the coarse collision detector.
 *
 * \param body The body to be registered with the coarse collision detector.
 * \return void
 *
 * This function is a requirement for all coarse collision detectors. It is automatically
 * called every time a new rigid body is added to the simulation world.
 */
template< typename C >  // Type of the configuration
void ExhaustiveSearch<C>::add( BodyID /*body*/ )
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistering a rigid body from the coarse collision detector.
 *
 * \param body The body to be deregistered from the coarse collision detector.
 * \return void
 *
 * This function is a requirement for all coarse collision detectors. It is automatically
 * called every time a rigid body is removed from the simulation world.
 */
template< typename C >  // Type of the configuration
void ExhaustiveSearch<C>::remove( BodyID /*body*/ )
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact generation between colliding rigid bodies.
 *
 * \param contacts Contact container for the generated contacts.
 * \return void
 *
 * This function generates all contacts between the rigid bodies contained in the simulation
 * world. The contacts are added to the contact container \a contacts.
 */
template< typename C >         // Type of the configuration
template< typename Contacts >  // Contact container type
void ExhaustiveSearch<C>::findContacts( Contacts& contacts )
{
   typedef typename BS::Iterator             BodyIterator;
   typedef typename Contacts::ConstIterator  ConstContactIterator;

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Finding the contacts via the exhaustive search algorithm...";
   }

   // Colliding local rigid bodies
   {
      const BodyIterator begin( bodystorage_.begin() );
      const BodyIterator end  ( bodystorage_.end()   );
      for( BodyIterator b1=begin; b1!=end; ++b1 ) {
         for( BodyIterator b2=b1+1; b2!=end; ++b2 ) {
            if( ( !b1->isFixed() || !b2->isFixed()  ) &&       // Ignoring contacts between two fixed bodies
                ( b1->getAABB().overlaps( b2->getAABB() ) ) )  // Testing for overlapping bounding boxes
               FineDetector::collide( *b1, *b2, contacts );
         }
      }
   }

   if( &bodystorage_ != &bodystorageShadowCopies_ ) {
      {
         // Collide all remote rigid bodies
         const BodyIterator begin( bodystorageShadowCopies_.begin() );
         const BodyIterator end  ( bodystorageShadowCopies_.end()   );
         for( BodyIterator b1=begin; b1!=end; ++b1 ) {
            for( BodyIterator b2=b1+1; b2!=end; ++b2 ) {
               if( ( !b1->isFixed() || !b2->isFixed()  ) &&       // Ignoring contacts between two fixed bodies
                   ( b1->getAABB().overlaps( b2->getAABB() ) ) )  // Testing for overlapping bounding boxes
                  FineDetector::collide( *b1, *b2, contacts );
            }
         }
      }

      {
         // Collide local with remote rigid bodies
         const BodyIterator end1 ( bodystorage_.end()   );
         const BodyIterator end2 ( bodystorageShadowCopies_.end()   );
         for( BodyIterator b1=bodystorage_.begin(); b1!=end1; ++b1 ) {
            for( BodyIterator b2=bodystorageShadowCopies_.begin(); b2!=end2; ++b2 ) {
               if( ( !b1->isFixed() || !b2->isFixed()  ) &&       // Ignoring contacts between two fixed bodies
                   ( b1->getAABB().overlaps( b2->getAABB() ) ) )  // Testing for overlapping bounding boxes
                  FineDetector::collide( *b1, *b2, contacts );
            }
         }
      }
   }


   pe_LOG_DEBUG_SECTION( log ) {
      if( contacts.isEmpty() )
         log << "      no contacts found!";
      else {
         log << "      State of the contacts:\n";
         for( ConstContactIterator c=contacts.begin(); c!=contacts.end(); ++c ) {
            log << "         Contact " << c->getID() << " (" << c->getType() << "):"
                << " pos = " << c->getPosition()
                << " , n = " << c->getNormal()
                << " , dist = " << c->getDistance()
                << " , vrel = " << c->getNormalRelVel()
                << " , cor = " << c->getRestitution()
                << "\n";
         }
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clears the coarse collision detector.
 *
 * \return void
 */
template< typename C >  // Type of the configuration
void ExhaustiveSearch<C>::clear()
{}
//*************************************************************************************************

} // namespace coarse

} // namespace detection

} // namespace pe

#endif
