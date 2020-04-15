//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyVelocityUpdateNotification.h
 *  \brief Header file for the RigidBodyVelocityUpdateNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYVELOCITYUPDATENOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYVELOCITYUPDATENOTIFICATION_H_


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
/*!\brief Wrapper class for rigid body velocity updates.
 * \ingroup rigid_body
 *
 * The RigidBodyVelocityUpdateNotification class is a wrapper class for marshalling and unmarshalling rigid body
 * velocity updates. It includes linear and angular velocities.
 */
class RigidBodyVelocityUpdateNotification {
public:
   struct Parameters {
      id_t sid_;
      Vec3 v_, w_;
   };

   inline RigidBodyVelocityUpdateNotification( const RigidBody& b ) : body_(b) {}
   const RigidBody& body_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling rigid body updates.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 *
 * The update consists of the linear and angular velocities.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyVelocityUpdateNotification& obj ) {
   buffer << obj.body_.getSystemID();
   marshal( buffer, obj.body_.getLinearVel() );
   marshal( buffer, obj.body_.getAngularVel() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling rigid body updates.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The update consists of the linear and angular velocities.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyVelocityUpdateNotification::Parameters& objparam ) {
   buffer >> objparam.sid_;
   unmarshal( buffer, objparam.v_ );
   unmarshal( buffer, objparam.w_ );
}
//*************************************************************************************************

} // namespace pe

#endif
