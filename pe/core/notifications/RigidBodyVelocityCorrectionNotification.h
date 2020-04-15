//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyVelocityCorrectionNotification.h
 *  \brief Header file for the RigidBodyVelocityCorrectionNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYVELOCITYCORRECTIONNOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYVELOCITYCORRECTIONNOTIFICATION_H_


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
/*!\brief Wrapper class for rigid body velocity corrections.
 * \ingroup rigid_body
 *
 * The RigidBodyVelocityCorrectionNotification class is a wrapper class for marshalling and unmarshalling rigid body
 * velocity correction. It includes the system ID of the body the correction applies to and the linear and angular
 * velocity corrections.
 */
class RigidBodyVelocityCorrectionNotification {
public:
   struct Parameters {
      id_t sid_;
      Vec3 dv_, dw_;
   };

   inline RigidBodyVelocityCorrectionNotification( const RigidBody& b, const Vec3& dv, const Vec3& dw ) : body_(b), dv_(dv), dw_(dw) {}
   const RigidBody& body_;
   const Vec3& dv_, dw_;
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling rigid body updates.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 *
 * The update consists of the linear and angular velocity corrections.
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBodyVelocityCorrectionNotification& obj ) {
   buffer << obj.body_.getSystemID();
   marshal( buffer, obj.dv_ );
   marshal( buffer, obj.dw_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling rigid body updates.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The update consists of the linear and angular velocity corrections.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBodyVelocityCorrectionNotification::Parameters& objparam ) {
   buffer >> objparam.sid_;
   unmarshal( buffer, objparam.dv_ );
   unmarshal( buffer, objparam.dw_ );
}
//*************************************************************************************************

} // namespace pe

#endif
