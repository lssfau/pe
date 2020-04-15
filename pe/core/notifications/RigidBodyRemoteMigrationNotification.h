//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyRemoteMigrationNotification.h
 *  \brief Header file for the RigidBodyRemoteMigrationNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYREMOTEMIGRATIONNOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYREMOTEMIGRATIONNOTIFICATION_H_


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
/*!\brief Wrapper class for rigid body remote migration notifications.
 * \ingroup rigid_body
 *
 * The RigidBodyRemoteMigrationNotification class is a wrapper class for marshaling and unmarshaling
 * rigid body remote migration notifications. When receiving a remote migration notification this
 * indicates that a body of which the local process has a shadow copy migrates from the sending
 * process to another one. Remote migration notices may only be sent by the previous owner of the
 * body.
 */
class RigidBodyRemoteMigrationNotification {
public:
   struct Parameters {
      id_t sid_;
      int to_;
   };

   inline RigidBodyRemoteMigrationNotification( const RigidBody& b, int to ) : body_(b), to_(to) {}
   const RigidBody& body_;
   const int to_;
};
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Marshaling rigid body remote migration notifications.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshaled.
 * \return void
 *
 * The remote migration notifications just consists of the system ID of the body migrating and the
 * rank of the process it is migrating to.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyRemoteMigrationNotification& obj ) {
   // TODO fixed sizes

   buffer << obj.body_.getSystemID();
   buffer << obj.to_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshaling rigid body remote migration notifications.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The remote migration notifications just consists of the system ID of the body migrating and the
 * rank of the process it is migrating to.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyRemoteMigrationNotification::Parameters& objparam ) {
   // TODO fixed sizes

   buffer >> objparam.sid_;
   buffer >> objparam.to_;
}
//*************************************************************************************************


} // namespace pe

#endif
