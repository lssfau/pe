//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBodyUpdateNotification.h
 *  \brief Header file for the RigidBodyUpdateNotification class
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

#ifndef _PE_CORE_NOTIFICATIONS_RIGIDBODYUPDATENOTIFICATION_H_
#define _PE_CORE_NOTIFICATIONS_RIGIDBODYUPDATENOTIFICATION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/rigidbody/RigidBody.h>



namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Wrapper class for rigid body updates.
 * \ingroup rigid_body
 *
 * The RigidBodyUpdateNotification class is a wrapper class for marshalling and unmarshalling rigid
 * body updates. The class template should be the
 * collision system configuration and only serves the purpose to prevent compiler errors for
 * specific collision system choices. The marshalling of the list of shadow copy holders can only
 * be performed if the MPIRigidBodyTrait class is part of the rigid body class hierarchy, which is
 * not the case for all collision systems. However, if it is not part of the class hierarchy then
 * the function is also never called and the compiler need not compile the code and the template
 * parameter prevents it.
 */
template<typename C>
class RigidBodyUpdateNotification {
public:
   struct Parameters {
      id_t sid_;
      Vec3 gpos_, v_, w_;
      Quat q_;
      std::vector<int> reglist_;
   };

   inline RigidBodyUpdateNotification( const RigidBody& b ) : body_(b) {}
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
 * The update consists of the position, orientation and linear and angular velocities and the
 * list of shadow copy holders.
 */
template< typename Buffer, typename C >
inline void marshal( Buffer& buffer, const RigidBodyUpdateNotification<C>& obj ) {
   typedef typename ProcessStorage<Config>::ConstIterator ProcessConstIterator;

   buffer << obj.body_.getSystemID();
   marshal( buffer, obj.body_.getPosition() );
   marshal( buffer, obj.body_.getQuaternion() );
   marshal( buffer, obj.body_.getLinearVel() );
   marshal( buffer, obj.body_.getAngularVel() );

   buffer << obj.body_.sizeProcesses();
   for( ProcessConstIterator it = obj.body_.beginProcesses(); it != obj.body_.endProcesses(); ++it )
      marshal( buffer, it->getRank() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling rigid body updates.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \return void
 *
 * The update consists of the position, orientation and linear and angular velocities and the
 * list of shadow copy holders.
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, typename RigidBodyUpdateNotification<Config>::Parameters& objparam ) {
   buffer >> objparam.sid_;
   unmarshal( buffer, objparam.gpos_ );
   unmarshal( buffer, objparam.q_ );
   unmarshal( buffer, objparam.v_ );
   unmarshal( buffer, objparam.w_ );

   size_t n;
   buffer >> n;
   objparam.reglist_.reserve( n );
   for( size_t i = 0; i < n; ++i ) {
      int p;
      buffer >> p;
      objparam.reglist_.push_back( p );
   }
}
//*************************************************************************************************

} // namespace pe

#endif
