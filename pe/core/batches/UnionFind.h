//=================================================================================================
/*!
 *  \file pe/core/batches/UnionFind.h
 *  \brief Implementation of the 'Union Find' batch generation algorithm
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

#ifndef _PE_CORE_BATCHES_UNIONFIND_H_
#define _PE_CORE_BATCHES_UNIONFIND_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <set>
#include <pe/util/Assert.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NonCopyable.h>
#include <pe/util/Types.h>


namespace pe {

namespace batches {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of the 'Union Find' batch generation algorithm.
 * \ingroup batch_generation
 *
 * The UnionFind class represents the implementation of the 'Union Find' batch generation
 * algorithm. This algorithm is used to create independent batches of contacts that can
 * be solved independent/in parallel to each other.
 */
template< typename C >  // Type of the configuration
class UnionFind : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                               Config;       //!< Type of the configuration.
   typedef UnionFind<Config>               This;         //!< Type of this UnionFind instance.
   typedef typename Config::BodyType       BodyType;     //!< Type of the rigid bodies.
   typedef typename Config::BodyID         BodyID;       //!< Handle for a rigid body.
   typedef typename Config::ConstBodyID    ConstBodyID;  //!< Handle for a constant rigid body.
   typedef typename BodyType::ConstNodeID  ConstNodeID;  //!< Type of a constant node handle.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename Contacts, typename Batches >
   void generateBatches( Contacts& contacts, Batches& batches );
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( This, typename Config::BatchGenerator );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Generating independent batches of contacts.
 *
 * \param contacts Contact container holding all existing contacts.
 * \param batches Batch container to be filled with the contacts.
 * \return void
 *
 * This function generates independent batches of contacts from the contacts given in \a contacts.
 */
template< typename C >        // Type of the configuration
template< typename Contacts   // Contact container type
        , typename Batches >  // Batch container type
void UnionFind<C>::generateBatches( Contacts& contacts, Batches& batches )
{
   typedef typename Batches::Batch  Batch;

   static std::set<ConstNodeID> roots;

   // Initializations
   roots.clear();
   batches.clear();

   // Counting the number of independent contact graphs
   size_t batchCount( 0 );
   const typename Contacts::Iterator begin( contacts.begin() );
   const typename Contacts::Iterator end  ( contacts.end()   );

   for( typename Contacts::Iterator contact=begin; contact!=end; ++contact )
   {
      const ConstBodyID b1 = contact->getBody1();
      const ConstBodyID b2 = contact->getBody2();

      pe_INTERNAL_ASSERT( b1->isFixed() || b2->isFixed() || b1->getRoot() == b2->getRoot(),
                          "Conflicting root nodes in contacting bodies detected" );

      ConstNodeID root( b1->isFixed() ? b2->getRoot() : b1->getRoot() );

      if( roots.find( root ) == roots.end() ) {
         root->batch_ = batchCount++;
         roots.insert( root );
      }
   }

   // Sorting the contacts into batches
   batches.resize( batchCount );

   for( typename Contacts::Iterator contact=begin; contact!=end; ++contact )
   {
      const ConstBodyID b1 = contact->getBody1();
      const ConstBodyID b2 = contact->getBody2();

      ConstNodeID root( b1->isFixed() ? b2->getRoot() : b1->getRoot() );
      Batch& batch( batches[root->batch_] );

      contact->setIndex( batch.size() );           // Setting the batch index of the contact
      batches[root->batch_].pushBack( *contact );  // Adding the contact to the batch
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( !batches.isEmpty() ) {
         log << "   Batch generation...\n";
         for( size_t i=0; i<batches.size(); ++i ) {
            Batch& batch( batches[i] );
            log << "      Batch " << i+1 << " = (";
            for( typename Batch::SizeType j=0; j<batch.size(); ++j )
               log << " " << batch[j]->getID();
            log << " )\n";
         }
      }
   }
}
//*************************************************************************************************

} // namespace batches

} // namespace pe

#endif
