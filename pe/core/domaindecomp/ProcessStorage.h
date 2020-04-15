//=================================================================================================
/*!
 *  \file pe/core/domaindecomp/ProcessStorage.h
 *  \brief Process storage for remote MPI processes
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_DOMAINDECOMP_PROCESSSTORAGE_H_
#define _PE_CORE_DOMAINDECOMP_PROCESSSTORAGE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <functional>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Process storage for remote MPI processes.
 * \ingroup domaindecomp
 *
 * The ProcessStorage class stores all defined remote MPI processes (see class Process).
 */
template< typename C >  // Type of the configuration
class ProcessStorage
{
private:
   //**Type definitions****************************************************************************
   typedef typename C::ProcessType     ProcessType;     //!< Type of the remote processes.
   typedef typename C::ProcessID       ProcessID;       //!< Handle to a remote process.
   typedef typename C::ConstProcessID  ConstProcessID;  //!< Handle to a constant remote process.

   //! Container for the remote MPI processes.
   typedef PtrVector<ProcessType,NoDelete>  Processes;
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef typename Processes::SizeType       SizeType;       //!< Size type of the process storage.
   typedef typename Processes::Iterator       Iterator;       //!< Iterator over non-const processes.
   typedef typename Processes::ConstIterator  ConstIterator;  //!< Iterator over constant processes.
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool          isEmpty()           const;
   inline SizeType      size   ()           const;
   inline Iterator      begin  ();
   inline ConstIterator begin  ()           const;
   inline Iterator      end    ();
   inline ConstIterator end    ()           const;
   inline Iterator      find   ( int rank );
   inline ConstIterator find   ( int rank ) const;
   //@}
   //**********************************************************************************************

   //**Add/Remove functions************************************************************************
   /*!\name Add/Remove functions */
   //@{
   inline void          add    ( ProcessID body );
   inline void          remove ( ProcessID body );
   inline Iterator      remove ( Iterator pos );
   inline void          clear  ();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Processes processes_;  //!< The defined remote MPI processes.
   //@}
   //**********************************************************************************************

   //**Private struct Compare**********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Helper class for the find() function. */
   struct Compare : public std::binary_function<ConstProcessID,int,bool>
   {
      //**Binary function call operator************************************************************
      /*!\name Binary function call operator */
      //@{
      inline bool operator()( ConstProcessID process, int rank ) const {
         return process->getRank() == rank;
      }
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class MPISystem;
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
/*!\brief Returns \a true if no remote MPI processes have been connected.
 *
 * \return \a true if the process storage is empty, \a false if it is not.
 */
template< typename C >  // Type of the configuration
inline bool ProcessStorage<C>::isEmpty() const
{
   return processes_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of MPI processes contained in the process storage.
 *
 * \return The number of MPI processes.
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::SizeType ProcessStorage<C>::size() const
{
   return processes_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first connected remote MPI process.
 *
 * \return Iterator to the first MPI process.
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::Iterator ProcessStorage<C>::begin()
{
   return processes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator to the first connected remote MPI process.
 *
 * \return Constant iterator to the first MPI process.
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::ConstIterator ProcessStorage<C>::begin() const
{
   return processes_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last connected remote MPI process.
 *
 * \return Iterator just past the last MPI process.
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::Iterator ProcessStorage<C>::end()
{
   return processes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator just past the last connected remote MPI process.
 *
 * \return Constant iterator just past the last MPI process.
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::ConstIterator ProcessStorage<C>::end() const
{
   return processes_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a connected remote MPI process with a specific rank.
 *
 * \param rank The rank of the sought-after MPI process.
 * \return Iterator to the MPI process with rank \a rank or an iterator just past the last MPI process.
 *
 * This function finds the remote MPI processes with the given rank \a rank. In case the process
 * is found, the function returns an iterator to the process. Otherwise, the function returns an
 * iterator just past the last connected remote MPI process (see the end() function). This function
 * runs in linear time.
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::Iterator ProcessStorage<C>::find( int rank )
{
   const Iterator pbegin( processes_.begin() );
   const Iterator pend  ( processes_.end()   );

   return std::find_if( pbegin, pend, std::bind2nd( Compare(), rank ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a connected remote MPI process with a specific rank.
 *
 * \param rank The rank of the sought-after MPI process.
 * \return Constant iterator to the MPI process with rank \a rank or an iterator just past the last MPI process.
 *
 * This function finds the remote MPI processes with the given rank \a rank. In case the process
 * is found, the function returns an iterator to the process. Otherwise, the function returns an
 * iterator just past the last connected remote MPI process (see the end() function).
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::ConstIterator ProcessStorage<C>::find( int rank ) const
{
   const ConstIterator pbegin( processes_.begin() );
   const ConstIterator pend  ( processes_.end()   );

   return std::find_if( pbegin, pend, std::bind2nd( Compare(), rank ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a process to the process storage.
 *
 * \param process The new process to be added to the process storage.
 * \return void
 *
 * This function adds a proces to the process storage. Adding the same process multiple times
 * results in undefined behaviour. The time complexity is linear.
 */
template< typename C >  // Type of the configuration
inline void ProcessStorage<C>::add( ProcessID process )
{
   pe_INTERNAL_ASSERT( find( process->getRank() ) == processes_.end(), "Process with the same rank already added." );
   processes_.pushBack( process );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a process from the process storage.
 *
 * \param pos The iterator pointing to the process to be removed.
 * \return Iterator to the process after the erased process.
 *
 * This function removes a process from the process storage. \a pos must be a valid iterator.
 * Invalidates all iterators pointing at or past
 * the element to be removed. The time complexity is linear.
 */
template< typename C >  // Type of the configuration
inline typename ProcessStorage<C>::Iterator ProcessStorage<C>::remove( Iterator pos )
{
   pe_INTERNAL_ASSERT( pos != processes_.end(), "Process is not contained in the process storage." );
   return processes_.erase( pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a process from the process storage.
 *
 * \param process A handle of the process to be removed.
 * \return void
 *
 * This function removes a process from the process storage. \a process must be a valid process
 * handle and must be registered in the process storage. Invalidates all iterators pointing at or
 * past the element to be removed. The time complexity is linear.
 */
template< typename C >  // Type of the configuration
inline void ProcessStorage<C>::remove( ProcessID process )
{
   remove( find( process->getRank() ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing all processes from the process storage.
 *
 * \return void
 *
 * This function removes all processes from the process storage. The processes do not have to be
 * valid anymore that is they can already be deallocated. Invalidates all iterators of this
 * container.
 */
template< typename C >  // Type of the configuration
inline void ProcessStorage<C>::clear()
{
   processes_.clear();
}
//*************************************************************************************************

} // namespace pe

#endif
