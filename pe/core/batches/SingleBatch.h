//=================================================================================================
/*!
 *  \file pe/core/batches/SingleBatch.h
 *  \brief Implementation of the 'Single Batch' batch generation algorithm
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

#ifndef _PE_CORE_BATCHES_SINGLEBATCH_H_
#define _PE_CORE_BATCHES_SINGLEBATCH_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

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
/*!\brief Implementation of the 'Single batch' batch generation algorithm.
 * \ingroup batch_generation
 *
 * The SingleBatch class represents the implementation of the 'Single batch' batch generation
 * algorithm. This algorithm generates a single batch of contacts without sorting them into
 * independent groups of contacts.
 */
template< typename C >  // Type of the configuration
class SingleBatch : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                    Config;  //!< Type of the configuration.
   typedef SingleBatch<Config>  This;    //!< Type of this SingleBatch instance.
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
/*!\brief Generating a single batch of contacts.
 *
 * \param contacts Contact container holding all existing contacts.
 * \param batches Batch container to be filled with the contacts.
 * \return void
 *
 * This function generates a single batch of contacts from the contacts given in \a contacts.
 */
template< typename C >        // Type of the configuration
template< typename Contacts   // Contact container type
        , typename Batches >  // Batch container type
void SingleBatch<C>::generateBatches( Contacts& contacts, Batches& batches )
{
   typedef typename Batches::Batch  Batch;

   batches.clear();  // Clearing the batch container

   // Early exit in case the contact container is empty
   if( contacts.isEmpty() ) return;

   // Setting the batch indices
   const size_t size( contacts.size() );
   for( size_t i=0; i<size; ++i ) {
      contacts[i]->setIndex( i );
   }

   // Creating a single batch and adding all contacts to this batch
   batches.resize( 1 );
   batches[0].assign( contacts.begin(), contacts.end() );

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
