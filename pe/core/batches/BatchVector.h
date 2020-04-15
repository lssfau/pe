//=================================================================================================
/*!
 *  \file pe/core/batches/BatchVector.h
 *  \brief Vector for batches of interacting contacts
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

#ifndef _PE_CORE_BATCHES_BATCHVECTOR_H_
#define _PE_CORE_BATCHES_BATCHVECTOR_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/contact/ContactVector.h>
#include <pe/math/VectorN.h>
#include <pe/util/Types.h>


namespace pe {

namespace batches {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Vector for batches of interacting contacts.
 * \ingroup batch_generation
 *
 * The BatchVector is a container for groups of interacting contacts. Each batch contains all
 * contacts of a single contact graph that potentially interact with each other. In order to
 * access a single batch, the subscript operator can be used.
 */
template< typename C                    // Type of the contact
        , typename D = NoDelete         // Deletion policy
        , typename G = OptimalGrowth >  // Growth policy
class BatchVector
{
public:
   //**Type definitions****************************************************************************
   typedef C                     Contact;   //!< Type of the contacts.
   typedef ContactVector<C,D,G>  Batch;     //!< Type of a single batch of contacts.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   // No explicitly declared constructor.
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************

   //**Copy assignment operator********************************************************************
   // No explicitly declared copy assignment operator.
   //**********************************************************************************************

   //**Operators***********************************************************************************
   /*!\name Operators */
   //@{
   inline Batch&       operator[]( size_t index );
   inline const Batch& operator[]( size_t index ) const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool   isEmpty() const;
   inline size_t size()    const;
   inline void   clear();
   inline void   resize( size_t n );
   inline void   extend( size_t n );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   VectorN< Batch > batches_;  //!< Vector of contact batches.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Subscript operator for the direct access to a single batch.
 *
 * \param index Access index. The index has to be in the range \f$[0..N]\f$.
 * \return Reference to the accessed batch.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline typename BatchVector<C,D,G>::Batch& BatchVector<C,D,G>::operator[]( size_t index )
{
   pe_USER_ASSERT( index < batches_.size(), "Invalid vector access index" );
   return batches_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Subscript operator for the direct access to a single batch.
 *
 * \param index Access index. The index has to be in the range \f$[0..N]\f$.
 * \return Reference to the accessed batch.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline const typename BatchVector<C,D,G>::Batch& BatchVector<C,D,G>::operator[]( size_t index ) const
{
   pe_USER_ASSERT( index < batches_.size(), "Invalid vector access index" );
   return batches_[index];
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns \a true if the batch vector has no batch.
 *
 * \return \a true if the batch vector is empty, \a false if it is not.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline bool BatchVector<C,D,G>::isEmpty() const
{
   return batches_.size() == 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current number of batches.
 *
 * \return The number of batches.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline size_t BatchVector<C,D,G>::size() const
{
   return batches_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing all batches.
 *
 * \return void
 *
 * This function clears all contained contact vectors and the batch vector itself. After the
 * clear() function, the size of the batch vector is 0.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline void BatchVector<C,D,G>::clear()
{
   const size_t num( batches_.size() );
   for( size_t i=0; i<num; ++i )
      batches_[i].clear();
   batches_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Changing the number of batches.
 *
 * \param n The new number of batches.
 * \return void
 *
 * This function resizes the batch vector using the given size to \a n. During this operation,
 * new dynamic memory may be allocated in case the capacity of the vector is too small. Therefore
 * this function potentially changes all batches.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline void BatchVector<C,D,G>::resize( size_t n )
{
   batches_.resize( n, false );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Extending the number of batches.
 *
 * \param n Number of additional batches.
 * \return void
 *
 * This function increases the size of the batch vector by \a n elements. During this operation,
 * new dynamic memory may be allocated in case the capacity of the vector is too small. Therefore
 * this function potentially changes all batches.
 */
template< typename C    // Type of the contact
        , typename D    // Deletion policy
        , typename G >  // Growth policy
inline void BatchVector<C,D,G>::extend( size_t n )
{
   batches_.extend( n, false );
}
//*************************************************************************************************

} // namespace batches

} // namespace pe

#endif
