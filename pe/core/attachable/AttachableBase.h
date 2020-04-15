//=================================================================================================
/*!
 *  \file pe/core/attachable/AttachableBase.h
 *  \brief Header file for the AttachableBase class
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

#ifndef _PE_CORE_ATTACHABLE_ATTACHABLEBASE_H_
#define _PE_CORE_ATTACHABLE_ATTACHABLEBASE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/attachable/AttachableType.h>
#include <pe/core/Types.h>
#include <pe/core/UniqueID.h>
#include <pe/util/NonCopyable.h>
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
/*!\brief Basic attachable properties.
 * \ingroup core
 *
 * The AttachableBase class represents the basic properties of attachables.
 */
class AttachableBase : private NonCopyable
{
protected:
   //**Type definitions****************************************************************************
   typedef PtrVector<RigidBody,NoDelete>  Bodies;  //!< Vector for attached rigid bodies.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef Bodies::Iterator       Iterator;       //!< Iterator over the currently attached bodies.
   typedef Bodies::ConstIterator  ConstIterator;  //!< ConstIterator over the currently attached bodies.
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit AttachableBase( AttachableType type, id_t sid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~AttachableBase();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline AttachableType getType()     const;
   inline id_t           getSystemID() const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline Iterator      begin();
   inline ConstIterator begin() const;
   inline Iterator      end  ();
   inline ConstIterator end  () const;
   inline size_t        size () const;
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   const AttachableType type_;  //!< The type code of the attachable.
   id_t sid_;                   //!< The unique system-specific attachable ID.
   Bodies bodies_;              //!< Vector of attached rigid bodies.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the type of the attachable.
 *
 * \return The type of the attachable.
 */
inline AttachableType AttachableBase::getType() const
{
   return type_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the unique system-specific ID of the attachable.
 *
 * \return The system-specific ID.
 */
inline id_t AttachableBase::getSystemID() const
{
   return sid_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns an iterator to the first attached rigid body.
 *
 * \return Iterator to the first attached rigid body.
 */
inline AttachableBase::Iterator AttachableBase::begin()
{
   return bodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first attached rigid body.
 *
 * \return Iterator to the first attached rigid body.
 */
inline AttachableBase::ConstIterator AttachableBase::begin() const
{
   return bodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last attached rigid body.
 *
 * \return Iterator just past the last attached rigid body.
 */
inline AttachableBase::Iterator AttachableBase::end()
{
   return bodies_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last attached rigid body.
 *
 * \return Iterator just past the last attached rigid body.
 */
inline AttachableBase::ConstIterator AttachableBase::end() const
{
   return bodies_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of attached rigid bodies.
 *
 * \return The number of attached rigid bodies
 */
inline size_t AttachableBase::size() const
{
   return bodies_.size();
}
//*************************************************************************************************

} // namespace pe

#endif
