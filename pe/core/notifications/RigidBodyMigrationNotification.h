//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyMigrationNotification.h
 *  \brief Header file for the RigidBodyMigrationNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYMIGRATIONNOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYMIGRATIONNOTIFICATION_H_


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
/*!\brief Wrapper class for rigid body migration notifications.
 * \ingroup rigid_body
 *
 * The RigidBodyMigrationNotification class is a wrapper class for marshaling and unmarshaling
 * rigid body migration notifications. When receiving a migration notification this indicates that
 * a body migrates from the sending neighbor to the local process and the local process takes over
 * ownership of the body. Migration notices may only be sent if the new owner already obtained a
 * shadow copy previously. They may also only be sent by a neighboring process.
 */
class RigidBodyMigrationNotification {
public:
   struct Parameters {
      id_t sid_;
   };

   inline RigidBodyMigrationNotification( const RigidBody& b ) : body_(b) {}
   const RigidBody& body_;
};
//*************************************************************************************************



//*************************************************************************************************
/*!\brief Marshaling rigid body migration notifications.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshaled.
 * \return void
 *
 * The migration notifications just consists of the system ID of the body migrating.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyMigrationNotification& obj ) {
   buffer << obj.body_.getSystemID();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshaling rigid body migration notifications.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The migration notifications just consists of the system ID of the body migrating.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyMigrationNotification::Parameters& objparam ) {
   buffer >> objparam.sid_;
}
//*************************************************************************************************


} // namespace pe

#endif
