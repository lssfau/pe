//=================================================================================================
/*!
 *  \file pe/core/rigidbody/BodyStorage.h
 *  \brief Body storage of the rigid body simulation world
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2011-2012 Tobias Preclik
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

#ifndef _PE_CORE_RIGIDBODY_BODYSTORAGE_H_
#define _PE_CORE_RIGIDBODY_BODYSTORAGE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <functional>
#include <map>
#include <vector>
#include <pe/util/Assert.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>


namespace pe {




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Body storage of the rigid body simulation world.
 * \ingroup core
 *
 * A BodyStorage is a data structure for storing rigid bodies. It supports efficient insertion and
 * deletion operations.
 */
template< typename C >  // Type of the configuration
class BodyStorage
{
private:
   //**Type definitions****************************************************************************
   typedef typename C::BodyType     BodyType;     //!< Type of the rigid bodies.
   typedef typename C::BodyID       BodyID;       //!< Handle to a rigid body.
   typedef typename C::ConstBodyID  ConstBodyID;  //!< Handle to a constant rigid body.

   //! Container for the bodies contained in the simulation world.
   typedef PtrVector<BodyType,NoDelete>  Bodies;
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename Bodies::SizeType       SizeType;       //!< Size type of the body storage.
   typedef typename Bodies::Iterator       Iterator;       //!< Iterator over non-const bodies.
   typedef typename Bodies::ConstIterator  ConstIterator;  //!< Iterator over constant bodies.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline BodyStorage();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~BodyStorage();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool          isEmpty () const;
   inline SizeType      size    () const;
   template< typename T >
   inline SizeType      size    () const;
   inline Iterator      begin   ();
   inline ConstIterator begin   () const;
   template< typename T >
   inline typename Bodies::template CastIterator<T> begin();
   template< typename T >
   inline typename Bodies::template ConstCastIterator<T> begin() const;
   inline Iterator      end     ();
   inline ConstIterator end     () const;
   template< typename T >
   inline typename Bodies::template CastIterator<T> end();
   template< typename T >
   inline typename Bodies::template ConstCastIterator<T> end() const;
   inline BodyID        at      ( SizeType index );
   inline ConstBodyID   at      ( SizeType index ) const;
   inline Iterator      find    ( id_t sid );
   inline ConstIterator find    ( id_t sid ) const;
   inline Iterator      find    ( ConstBodyID body );
   inline ConstIterator find    ( ConstBodyID body ) const;
   inline void          validate();
   //@}
   //**********************************************************************************************

   //**Add/Remove functions************************************************************************
   /*!\name Add/Remove functions */
   //@{
   inline void          add     ( BodyID body );
   inline void          remove  ( BodyID body );
   inline ConstIterator remove  ( ConstIterator pos );
   inline Iterator      remove  ( Iterator pos );
   inline void          clear   ();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Bodies bodies_;  //!< The rigid bodies contained in the simulation world.
   std::map<id_t, SizeType> bodyIDs_;   //!< The association of system IDs to rigid bodies.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class World;
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
/*!\brief The standard constructor.
 */
template< typename C >  // Type of the configuration
inline BodyStorage<C>::BodyStorage()
	: bodies_( 1000 )
	, bodyIDs_()
{
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief The destructor.
 *
 * The destructor clears all rigid bodies from the storage before destructing it.
 */
template< typename C >  // Type of the configuration
inline BodyStorage<C>::~BodyStorage()
{
	clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns \a true if the body storage contains no rigid bodies.
 *
 * \return \a true if the body storage is empty, \a false if it is not.
 */
template< typename C >  // Type of the configuration
inline bool BodyStorage<C>::isEmpty() const
{
   return bodies_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rigid bodies contained in the body storage.
 *
 * \return The number of rigid bodies.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::SizeType BodyStorage<C>::size() const
{
   return bodies_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rigid bodies of type \a T contained in the body storage.
 *
 * \return The number of rigid bodies of type \a T.
 *
 * \b Note: The total number of objects of type \a T is not cached but recalculated each time the
 * function is called. Using the templated version of size() to calculate the total number objects
 * of type \a T is therefore more expensive than using the non-template version of size() to get
 * the total number of pointers in the vector!
 */
template< typename C >  // Type of the configuration
template< typename T >  // Cast type
inline typename BodyStorage<C>::SizeType BodyStorage<C>::size() const
{
   return bodies_.template size<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first contained rigid body.
 *
 * \return Iterator to the first contained rigid body.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::Iterator BodyStorage<C>::begin()
{
   return bodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator to the first contained rigid body.
 *
 * \return Constant iterator to the first contained rigid body.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::ConstIterator BodyStorage<C>::begin() const
{
   return bodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first contained rigid body.
 *
 * \return Iterator to the first contained rigid body.
 */
template< typename C >  // Type of the configuration
template< typename T >  // Cast Type
inline typename BodyStorage<C>::Bodies::template CastIterator<T> BodyStorage<C>::begin()
{
   return bodies_.template begin<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator to the first contained rigid body.
 *
 * \return Constant iterator to the first contained rigid body.
 */
template< typename C >  // Type of the configuration
template< typename T >  // Cast Type
inline typename BodyStorage<C>::Bodies::template ConstCastIterator<T> BodyStorage<C>::begin() const
{
   return bodies_.template begin<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last contained rigid body.
 *
 * \return Iterator just past the last contained rigid body.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::Iterator BodyStorage<C>::end()
{
   return bodies_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator just past the last contained rigid body.
 *
 * \return Constant iterator just past the last contained rigid body.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::ConstIterator BodyStorage<C>::end() const
{
   return bodies_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last contained rigid body.
 *
 * \return Iterator just past the last contained rigid body.
 */
template< typename C >  // Type of the configuration
template< typename T >  // Cast Type
inline typename BodyStorage<C>::Bodies::template CastIterator<T> BodyStorage<C>::end()
{
   return bodies_.template end<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator just past the last contained rigid body.
 *
 * \return Constant iterator just past the last contained rigid body.
 */
template< typename C >  // Type of the configuration
template< typename T >  // Cast Type
inline typename BodyStorage<C>::Bodies::template ConstCastIterator<T> BodyStorage<C>::end() const
{
   return bodies_.template end<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the indexed rigid body.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed rigid body.
 *
 * \b Note: No runtime check is performed to ensure the validity of the access index.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::BodyID BodyStorage<C>::at( SizeType index )
{
   pe_USER_ASSERT( index < size(), "Invalid body index" );
   return bodies_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant handle to the indexed rigid body.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Constant handle to the accessed rigid body.
 *
 * \b Note: No runtime check is performed to ensure the validity of the access index.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::ConstBodyID BodyStorage<C>::at( SizeType index ) const
{
   pe_USER_ASSERT( index < size(), "Invalid body index" );
   return bodies_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a rigid body with a certain unique system-specific ID.
 *
 * \param sid The unique system-specific ID for the search.
 * \return Iterator to the body with system-specific ID \a sid or an iterator just past the end.
 *
 * This function finds the rigid body with the system-specific ID \a sid. In case the rigid body
 * is found, the function returns an iterator to the body. Otherwise, the function returns an
 * iterator just past the end of the last body contained in the body storage.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::Iterator BodyStorage<C>::find( id_t sid )
{
   typename std::map<id_t, SizeType>::const_iterator pos = bodyIDs_.find( sid );
   if( pos == bodyIDs_.end() )
      return bodies_.end();

   return bodies_.begin() + pos->second;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a rigid body with a certain unique system-specific ID.
 *
 * \param sid The unique system-specific ID for the search.
 * \return Constant iterator to the body with system-specific ID \a sid or a constant iterator just past the end.
 *
 * This function finds the rigid body with the system-specific ID \a sid. In case the rigid body
 * is found, the function returns a constant iterator to the body. Otherwise, the function returns
 * a constant iterator just past the end of the last body contained in the body storage.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::ConstIterator BodyStorage<C>::find( id_t sid ) const
{
   typename std::map<id_t, SizeType>::const_iterator pos = bodyIDs_.find( sid );
   if( pos == bodyIDs_.end() )
      return bodies_.end();

   return bodies_.begin() + pos->second;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a specific rigid body in the body storage.
 *
 * \param body The given rigid body for the search.
 * \return Iterator to the given rigid body or an iterator just past the end.
 *
 * This function finds the rigid body in the body storage. In case the rigid body is found,
 * the function returns an iterator to the body. Otherwise, the function returns an iterator
 * just past the end of the last body contained in the body storage.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::Iterator BodyStorage<C>::find( ConstBodyID body )
{
   return find( body->getSystemID() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a specific rigid body in the body storage.
 *
 * \param body The given rigid body for the search.
 * \return Constant iterator to the given rigid body or a constant iterator just past the end.
 *
 * This function finds the rigid body in the body storage. In case the rigid body is found,
 * the function returns a constant iterator to the body. Otherwise, the function returns a
 * constant iterator just past the end of the last body contained in the body storage.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::ConstIterator BodyStorage<C>::find( ConstBodyID body ) const
{
   return find( body->getSystemID() );
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a rigid body to the body storage.
 *
 * \param body The new rigid body to be added to the body storage.
 * \return void
 *
 * This function adds a rigid body to the body storage. Adding bodies with non-unique system ID or
 * adding the same body multiple times results in undefined behaviour. The time complexity is
 * logarithmic unless reallocation occurs.
 */
template< typename C >  // Type of the configuration
inline void BodyStorage<C>::add( BodyID body )
{
   pe_INTERNAL_ASSERT( bodyIDs_.find( body->getSystemID() ) == bodyIDs_.end(), "Body with same system ID already added." );
   bodyIDs_[ body->getSystemID() ] = bodies_.size();
   bodies_.pushBack( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the body storage.
 *
 * \param pos The position of the rigid body to be removed.
 * \return Iterator to the body after the erased rigid body.
 *
 * This function removes a body from the body storage. \a pos must be a valid iterator and the
 * rigid body pointer referred to must be valid. Invalidates all iterators pointing at or past
 * the element to be removed. The time complexity is logarithmic unless reallocation occurs.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::ConstIterator BodyStorage<C>::remove( ConstIterator pos )
{
   typename std::map<id_t, SizeType>::iterator it = bodyIDs_.find( (*pos)->getSystemID() );
   pe_INTERNAL_ASSERT( it != bodyIDs_.end(), "The body's system ID was not registered." );

   // Move last element to deleted place and update the system ID to index mapping.
   SizeType i = it->second;
   bodyIDs_[ bodies_.back()->getSystemID() ] = i;
   std::swap( bodies_[i], bodies_.back() );
   bodyIDs_.erase( it );
   bodies_.popBack();

   return pos;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the body storage.
 *
 * \param pos The position of the rigid body to be removed.
 * \return Iterator to the body after the erased rigid body.
 *
 * This function removes a body from the body storage. \a pos must be a valid iterator and the
 * rigid body pointer referred to must be valid. Invalidates all iterators pointing at or past
 * the element to be removed. The time complexity is logarithmic unless reallocation occurs.
 */
template< typename C >  // Type of the configuration
inline typename BodyStorage<C>::Iterator BodyStorage<C>::remove( Iterator pos )
{
   typename std::map<id_t, SizeType>::iterator it = bodyIDs_.find( (*pos)->getSystemID() );
   pe_INTERNAL_ASSERT( it != bodyIDs_.end(), "The body's system ID was not registered." );

   // Move last element to deleted place and update the system ID to index mapping.
   SizeType i = it->second;
   bodyIDs_[ bodies_.back()->getSystemID() ] = i;
   std::swap( bodies_[i], bodies_.back() );
   bodyIDs_.erase( it );
   bodies_.popBack();

   return pos;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the body storage.
 *
 * \param body A handle of the rigid body to be removed.
 * \return void
 *
 * This function removes a body from the body storage. \a body must be a valid rigid body poitner
 * and must be registered in the body storage. Invalidates all iterators pointing at or past
 * the element to be removed. The time complexity is logarithmic unless reallocation occurs.
 */
template< typename C >  // Type of the configuration
inline void BodyStorage<C>::remove( BodyID body )
{
   typename std::map<id_t, SizeType>::iterator it = bodyIDs_.find( body->getSystemID() );
   pe_INTERNAL_ASSERT( it != bodyIDs_.end(), "The body's system ID was not registered." );

   // Move last element to deleted place and update the system ID to index mapping.
   SizeType i = it->second;
   bodyIDs_[ bodies_.back()->getSystemID() ] = i;
   std::swap( bodies_[i], bodies_.back() );
   bodyIDs_.erase( it );
   bodies_.popBack();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all rigid bodies from the body storage.
 *
 * \return void
 *
 * This function removes all bodies from the body storage. The rigid bodies do not have to be
 * valid anymore that is they can already be deallocated. Invalidates all iterators of this
 * container.
 */
template< typename C >  // Type of the configuration
inline void BodyStorage<C>::clear()
{
   bodyIDs_.clear();
   bodies_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Validating the correctness of the body storage data structure.
 *
 * \return void
 *
 * This function validates the data structure in linear time and space. If validation fails
 * assertions are triggered unless the pe is compiled in release mode.
 */
template< typename C >  // Type of the configuration
inline void BodyStorage<C>::validate()
{
   std::vector<bool> tmp(bodies_.size());
   typename std::map<id_t, SizeType>::iterator it = bodyIDs_.begin();
   while( it != bodyIDs_.end() ) {
      pe_INTERNAL_ASSERT(tmp[it->second] == false, "Two system IDs point to the same storage index.");
      tmp[it->second] = true;
      pe_INTERNAL_ASSERT(bodies_[it->second]->getSystemID() == it->first, "The mapping of system ID to storage index is wrong since the system ID does not match with the stored body.");
      ++it;
   }

   pe_INTERNAL_ASSERT( bodyIDs_.size() == bodies_.size(), "The mapping is missing some system IDs." );
}
//*************************************************************************************************

} // namespace pe

#endif
