//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyRemovalNotification.h
 *  \brief Header file for the RigidBodyRemovalNotification class
 *
 *  Copyright (C) 2012 Tobias Preclik
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYREMOVALNOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYREMOVALNOTIFICATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/rigidbody/RigidBody.h>



namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Wrapper class for rigid body removal notifications.
 * \ingroup rigid_body
 *
 * The RigidBodyRemovalNotification class is a wrapper class for marshaling and unmarshaling rigid body
 * removal notifications. When receiving a removal notification this indicates that a shadow copy
 * is no longer needed by the receiving process. A removal notification must only be sent if
 * the receiving process actually has a shadow copy of the concerned body and it must only be
 * sent by the owner and it must only be sent if the owner is sure that the receiving process
 * will no longer need the shadow copy.
 */
class RigidBodyRemovalNotification {
public:
   struct Parameters {
      id_t sid_;
   };

   inline RigidBodyRemovalNotification( const RigidBody& b ) : body_(b) {}
   const RigidBody& body_;
};
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Marshaling rigid body removal notifications.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshaled.
 * \return void
 *
 * The removal notifications just consists of the system ID of the body to remove.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyRemovalNotification& obj ) {
   buffer << obj.body_.getSystemID();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshaling rigid body removal notifications.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The removal notifications just consists of the system ID of the body to remove.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyRemovalNotification::Parameters& objparam ) {
   buffer >> objparam.sid_;
}
//*************************************************************************************************


} // namespace pe

#endif
