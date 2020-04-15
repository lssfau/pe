//=================================================================================================
/*!
 *  \file pe/core/BodyManager.h
 *  \brief Header file for the BodyManager class
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

#ifndef _PE_CORE_BODYMANAGER_H_
#define _PE_CORE_BODYMANAGER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/Types.h>
#include <pe/system/Precision.h>
#include <pe/util/Null.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for rigid body managers.
 * \ingroup core
 *
 * The BodyManager class represents the interface for classes taking responsibility for rigid
 * bodies. Rigid bodies can be added to and removed from body managers and the body manager
 * takes full responsibility for the memory management of all contained rigid bodies.\n
 * The BodyManager class gives derived classes a certain number of additional rights for the
 * management of rigid bodies. This includes the registration as the current manager, the
 * movement and the complete destruction of rigid bodies.
 */
class BodyManager
{
private:
   //**World setup functions***********************************************************************
   /*! \cond PE_INTERNAL */
   friend ManagerID theDefaultManager();
   /*! \endcond */
   //**********************************************************************************************

public:
   //**Add/remove functions************************************************************************
   /*!\name Add/remove functions */
   //@{
   virtual void add   ( BodyID body ) = 0;
   virtual void remove( BodyID body ) = 0;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void setDefaultManager();
   //@}
   //**********************************************************************************************

protected:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BodyManager() = 0;
   //@}
   //**********************************************************************************************

   //**Rigid body manager functions****************************************************************
   /*!\name Rigid body manager functions */
   //@{
          inline void setManager  ( BodyID body );
          inline void resetManager( BodyID body );
   static inline void destroyBody ( BodyID body );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static ManagerID default_;  //!< The currently active default body manager.
                               /*!< This body manager handle refers to the currently active
                                    default body manager. Per default, this role is taken by
                                    the simulation world (see the World class). However, every
                                    body manager can be instated as the default body manager
                                    by using the setDefaultManager() member function. */
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Enstates this body manager as the default body manager.
 *
 * \return void
 */
inline void BodyManager::setDefaultManager()
{
   default_ = this;
}
//*************************************************************************************************




//=================================================================================================
//
//  RIGID BODY MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the supervising rigid body manager of a rigid body.
 *
 * \param body The supervised rigid body.
 * \return void
 */
inline void BodyManager::setManager( BodyID body )
{
   body->manager_ = this;  // Setting the rigid body manager
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the supervising rigid body manager of a rigid body.
 *
 * \param body The supervised rigid body.
 * \return void
 */
inline void BodyManager::resetManager( BodyID body )
{
   body->manager_ = NULL;  // Resetting the rigid body manager
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Destroying a rigid body.
 *
 * \param body The rigid body to be destroyed.
 * \return void
 *
 * This function allows the rigid body manager to completely destroy the given rigid body.
 */
inline void BodyManager::destroyBody( BodyID body )
{
   delete body;
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL BODY MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Global body manager functions */
//@{
ManagerID theDefaultManager();
//@}
//*************************************************************************************************

} // namespace pe

#endif
