//=================================================================================================
/*!
 *  \file pe/core/joint/JointStorage.h
 *  \brief Joint storage of the rigid body simulation world
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

#ifndef _PE_CORE_JOINT_JOINTSTORAGE_H_
#define _PE_CORE_JOINT_JOINTSTORAGE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/joint/Joint.h>
#include <pe/util/PtrVector.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Joint storage of the rigid body simulation world.
 * \ingroup core
 *
 * The JointStorage class stores all generated joints of the simulation world (see class
 * World).
 */
template< typename C >  // Type of the configuration
class JointStorage
{
private:
   //**Type definitions****************************************************************************
   //! Container for the joints contained in the simulation world.
   typedef PtrVector<Joint,NoDelete>  Joints;
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef Joints::SizeType       SizeType;       //!< Size type of the joint storage.
   typedef Joints::Iterator       Iterator;       //!< Iterator over non-const joints.
   typedef Joints::ConstIterator  ConstIterator;  //!< Iterator over constant joints.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit inline JointStorage( SizeType initCapacity = 100 );
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
   inline Iterator      find   ( ConstJointID joint );
   inline ConstIterator find   ( ConstJointID joint ) const;
   //@}
   //**********************************************************************************************

   //**Add/Remove functions************************************************************************
   /*!\name Add/Remove functions */
   //@{
   inline void     add   ( JointID joint );
   inline Iterator remove( Iterator pos );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Joints joints_;  //!< The joints of the simulation world.
   //@}
   //**********************************************************************************************

   //**Private struct Compare**********************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Helper class for the find() function. */
   struct Compare : public std::binary_function<JointID,id_t,bool>
   {
      //**Binary function call operator************************************************************
      /*!\name Binary function call operator */
      //@{
      inline bool operator()( ConstJointID joint, id_t sid ) const {
         return joint->getSystemID() < sid;
      }
      inline bool operator()( id_t sid, ConstJointID obj ) const {
         return sid < obj->getSystemID();
      }
      inline bool operator()( ConstJointID obj1, ConstJointID obj2 ) const {
         return obj1->getSystemID() < obj2->getSystemID();
      }
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   //template< typename C2 > friend class CollisionSystem;
   friend class Joint;
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
/*!\brief Standard constructor for the JointStorage.
 *
 * \param initCapacity The initial capacity of the joint storage.
 */
template< typename C >  // Type of the configuration
inline JointStorage<C>::JointStorage( SizeType initCapacity )
   : joints_( initCapacity )
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns \a true if the joint storage contains no joints.
 *
 * \return \a true if the joint storage is empty, \a false if it is not.
 */
template< typename C >  // Type of the configuration
inline bool JointStorage<C>::isEmpty() const
{
   return joints_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of joints contained in the joint storage.
 *
 * \return The number of joints.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::SizeType JointStorage<C>::size() const
{
   return joints_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first contained joint.
 *
 * \return Iterator to the first contained joint.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::Iterator JointStorage<C>::begin()
{
   return joints_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator to the first contained joint.
 *
 * \return Constant iterator to the first contained joint.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::ConstIterator JointStorage<C>::begin() const
{
   return joints_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last contained joint.
 *
 * \return Iterator just past the last contained joint.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::Iterator JointStorage<C>::end()
{
   return joints_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator just past the last contained joint.
 *
 * \return Constant iterator just past the last contained joint.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::ConstIterator JointStorage<C>::end() const
{
   return joints_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding an joint with a certain unique system-specific ID.
 *
 * \param sid The unique system-specific ID for the search.
 * \return Iterator to the joint with system-specific ID \a sid or an iterator just past the end.
 *
 * This function finds the joint with the system-specific ID \a sid. In case the joint
 * is found, the function returns an iterator to the joint. Otherwise, the function returns
 * an iterator just past the end of last joint contained in the joint storage.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::Iterator JointStorage<C>::find( id_t sid )
{
   Iterator pos = std::lower_bound( begin(), end(), sid, Compare() );
   if( pos != end() && pos->getSystemID() == sid )
      return pos;
   else
      return end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding an joint with a certain unique system-specific ID.
 *
 * \param sid The unique system-specific ID for the search.
 * \return Constant iterator to the joint with system-specific ID \a sid or a constant iterator just past the end.
 *
 * This function finds the joint with the system-specific ID \a sid. In case the joint
 * is found, the function returns a constant iterator to the joint. Otherwise, the function
 * returns a constant iterator just past the end of last joint contained in the joint
 * storage.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::ConstIterator JointStorage<C>::find( id_t sid ) const
{
   ConstIterator pos = std::lower_bound( begin(), end(), sid, Compare() );
   if( pos != end() && pos->getSystemID() == sid )
      return pos;
   else
      return end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a specific joint in the joint storage.
 *
 * \param joint The given joint for the search.
 * \return Iterator to the given joint or an iterator just past the end.
 *
 * This function finds the joint in the joint storage. In case the joint is found,
 * the function returns an iterator to the joint. Otherwise, the function returns an iterator
 * just past the end of last joint contained in the joint storage.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::Iterator JointStorage<C>::find( ConstJointID joint )
{
   return find( joint->getSystemID() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Finding a specific joint in the joint storage.
 *
 * \param joint The given joint for the search.
 * \return Iterator to the given joint or an iterator just past the end.
 *
 * This function finds the joint in the joint storage. In case the joint is found,
 * the function returns an iterator to the joint. Otherwise, the function returns an iterator
 * just past the end of last joint contained in the joint storage.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::ConstIterator JointStorage<C>::find( ConstJointID joint ) const
{
   return find( joint->getSystemID() );
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding an joint to the joint storage.
 *
 * \param joint The new joint to be added to the joint storage.
 * \return void
 *
 * This function adds an joint to the joint storage.
 */
template< typename C >  // Type of the configuration
inline void JointStorage<C>::add( JointID joint )
{
   Iterator pos = std::lower_bound( begin(), end(), joint->getSystemID(), Compare() );
   joints_.insert( pos, joint );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing an joint from the joint storage.
 *
 * \param pos The position of the joint to be removed.
 * \return Iterator to the joint after the erased joint.
 *
 * This function removes an joint from the joint storage.
 */
template< typename C >  // Type of the configuration
inline typename JointStorage<C>::Iterator JointStorage<C>::remove( Iterator pos )
{
   pe_INTERNAL_ASSERT( pos != end(), "Attachable is not contained in the joint storage" );

   return joints_.erase( pos );
}
//*************************************************************************************************

} // namespace pe

#endif
