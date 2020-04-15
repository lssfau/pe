//=================================================================================================
/*!
 *  \file pe/core/attachable/AttachableStorage.h
 *  \brief Header file for the AttachableStorage class
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

#ifndef _PE_CORE_ATTACHABLE_ATTACHABLESTORAGE_H_
#define _PE_CORE_ATTACHABLE_ATTACHABLESTORAGE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <functional>
#include <pe/core/attachable/Attachable.h>
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
/*!\brief Attachable storage of the rigid body simulation world.
 * \ingroup core
 *
 * The AttachableStorage class stores all currently existing attachables in the simulation world
 * (see class World).
 */
template< typename C >  // Type of the configuration
class AttachableStorage
{
private:
   //**Type definitions****************************************************************************
   //! Container for the attachables contained in the simulation world.
   typedef PtrVector<Attachable,NoDelete>  Attachables;
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef Attachables::SizeType       SizeType;       //!< Size type of the attachable storage.
   typedef Attachables::Iterator       Iterator;       //!< Iterator over non-const attachables.
   typedef Attachables::ConstIterator  ConstIterator;  //!< Iterator over constant attachables.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline AttachableStorage( SizeType initCapacity = 100 );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool          isEmpty() const;
   inline SizeType      size   () const;
   inline Iterator      begin  ();
   inline ConstIterator begin  () const;
   inline Iterator      end    ();
   inline ConstIterator end    () const;
   inline Iterator      find   ( id_t sid );
   inline ConstIterator find   ( id_t sid ) const;
   inline Iterator      find   ( ConstAttachableID attachable );
   inline ConstIterator find   ( ConstAttachableID attachable ) const;
   //@}
   //**********************************************************************************************

   //**Add/Remove functions************************************************************************
   /*!\name Add/Remove functions */
   //@{
   inline void     add   ( AttachableID attachable );
   inline Iterator remove( Iterator pos );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Attachables attachables_;  //!< The currently existing attachables.
   //@}
   //**********************************************************************************************

   //**Private struct Compare**********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Helper class for the find() function. */
   struct Compare : public std::binary_function<AttachableID,id_t,bool>
   {
      //**Binary function call operator************************************************************
      /*!\name Binary function call operator */
      //@{
      inline bool operator()( ConstAttachableID attachable, id_t sid ) const {
         return attachable->getSystemID() < sid;
      }
      inline bool operator()( id_t sid, ConstAttachableID attachable ) const {
         return sid < attachable->getSystemID();
      }
      inline bool operator()( ConstAttachableID attachable1, ConstAttachableID attachable2 ) const {
         return attachable1->getSystemID() < attachable2->getSystemID();
      }
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class Attachable;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Standard constructor for the AttachableStorage.
 *
 * \param initCapacity The initial capacity of the attachable storage.
 */
template< typename C >  // Type of the configuration
inline AttachableStorage<C>::AttachableStorage( SizeType initCapacity )
   : attachables_( initCapacity )
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns \a true if the attachable storage contains no attachables.
 *
 * \return \a true if the attachable storage is empty, \a false if it is not.
 */
template< typename C >  // Type of the configuration
inline bool AttachableStorage<C>::isEmpty() const
{
   return attachables_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of attachables contained in the attachable storage.
 *
 * \return The number of attachables.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::SizeType AttachableStorage<C>::size() const
{
   return attachables_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first contained attachable.
 *
 * \return Iterator to the first contained attachable.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::Iterator AttachableStorage<C>::begin()
{
   return attachables_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator to the first contained attachable.
 *
 * \return Constant iterator to the first contained attachable.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::ConstIterator AttachableStorage<C>::begin() const
{
   return attachables_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last contained attachable.
 *
 * \return Iterator just past the last contained attachable.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::Iterator AttachableStorage<C>::end()
{
   return attachables_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator just past the last contained attachable.
 *
 * \return Constant iterator just past the last contained attachable.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::ConstIterator AttachableStorage<C>::end() const
{
   return attachables_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding an attachable with a certain unique system-specific ID.
 *
 * \param sid The unique system-specific ID for the search.
 * \return Iterator to the attachable with system-specific ID \a sid or an iterator just past the end.
 *
 * This function finds the attachable with the system-specific ID \a sid. In case the attachable
 * is found, the function returns an iterator to the attachable. Otherwise, the function returns
 * an iterator just past the end of last attachable contained in the attachable storage.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::Iterator AttachableStorage<C>::find( id_t sid )
{
   Iterator pos = std::lower_bound( begin(), end(), sid, Compare() );
   if( pos != end() && pos->getSystemID() == sid )
      return pos;
   else
      return end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding an attachable with a certain unique system-specific ID.
 *
 * \param sid The unique system-specific ID for the search.
 * \return Constant iterator to the attachable with system-specific ID \a sid or a constant iterator just past the end.
 *
 * This function finds the attachable with the system-specific ID \a sid. In case the attachable
 * is found, the function returns a constant iterator to the attachable. Otherwise, the function
 * returns a constant iterator just past the end of last attachable contained in the attachable
 * storage.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::ConstIterator AttachableStorage<C>::find( id_t sid ) const
{
   ConstIterator pos = std::lower_bound( begin(), end(), sid, Compare() );
   if( pos != end() && pos->getSystemID() == sid )
      return pos;
   else
      return end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a specific attachable in the attachable storage.
 *
 * \param attachable The given attachable for the search.
 * \return Iterator to the given attachable or an iterator just past the end.
 *
 * This function finds the attachable in the attachable storage. In case the attachable is found,
 * the function returns an iterator to the attachable. Otherwise, the function returns an iterator
 * just past the end of last attachable contained in the attachable storage.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::Iterator AttachableStorage<C>::find( ConstAttachableID attachable )
{
   return find( attachable->getSystemID() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a specific attachable in the attachable storage.
 *
 * \param attachable The given attachable for the search.
 * \return Iterator to the given attachable or an iterator just past the end.
 *
 * This function finds the attachable in the attachable storage. In case the attachable is found,
 * the function returns an iterator to the attachable. Otherwise, the function returns an iterator
 * just past the end of last attachable contained in the attachable storage.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::ConstIterator AttachableStorage<C>::find( ConstAttachableID attachable ) const
{
   return find( attachable->getSystemID() );
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding an attachable to the attachable storage.
 *
 * \param attachable The new attachable to be added to the attachable storage.
 * \return void
 *
 * This function adds an attachable to the attachable storage.
 */
template< typename C >  // Type of the configuration
inline void AttachableStorage<C>::add( AttachableID attachable )
{
   Iterator pos = std::lower_bound( begin(), end(), attachable->getSystemID(), Compare() );
   attachables_.insert( pos, attachable );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing an attachable from the attachable storage.
 *
 * \param pos The position of the attachable to be removed.
 * \return Iterator to the attachable after the erased attachable.
 *
 * This function removes an attachable from the attachable storage.
 */
template< typename C >  // Type of the configuration
inline typename AttachableStorage<C>::Iterator AttachableStorage<C>::remove( Iterator pos )
{
   pe_INTERNAL_ASSERT( pos != end(), "Attachable is not contained in the attachable storage" );

   return attachables_.erase( pos );
}
//*************************************************************************************************

} // namespace pe

#endif
