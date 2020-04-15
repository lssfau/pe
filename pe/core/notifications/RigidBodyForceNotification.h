//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyForceNotification.h
 *  \brief Header file for the RigidBodyForceNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYFORCENOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYFORCENOTIFICATION_H_


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
/*!\brief Wrapper class for rigid body force and torque contribution notifications.
 * \ingroup rigid_body
 *
 * The RigidBodyForceNotification class is a wrapper class for marshaling and unmarshaling rigid body
 * force and torque contribution notifications. They may only be sent by processes registered
 * to have a shadow copy. They may only be sent to the owner of the rigid body.
 */
class RigidBodyForceNotification {
public:
   struct Parameters {
      id_t sid_;
      Vec3 f_, tau_;
   };

   inline RigidBodyForceNotification( const RigidBody& b ) : body_(b) {}
   const RigidBody& body_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshaling rigid body force notifications.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshaled.
 * \return void
 *
 * The force notification consists of the force and torque contribution to a body.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyForceNotification& obj ) {
   buffer << obj.body_.getSystemID();
   marshal( buffer, obj.body_.getForce() );
   marshal( buffer, obj.body_.getTorque() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshaling rigid body updates.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The update consists of the position, orientation and linear and angular velocities.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyForceNotification::Parameters& objparam ) {
   buffer >> objparam.sid_;
   unmarshal( buffer, objparam.f_ );
   unmarshal( buffer, objparam.tau_ );
}
//*************************************************************************************************

} // namespace pe

#endif
