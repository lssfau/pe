//=================================================================================================
/*!
*  \file pe/core/notifications/NotificationType.h
 *  \brief Header file for the notification types
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

#ifndef _PE_CORE_NOTIFICATIONS_NOTIFICATIONTYPE_H_
#define _PE_CORE_NOTIFICATIONS_NOTIFICATIONTYPE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/util/Byte.h>


namespace pe {

//=================================================================================================
//
//  NOTIFICATION TYPES
//
//=================================================================================================

//*************************************************************************************************
//! Associate a unique number to notifications for identifying/tagging them.
enum NotificationType {
   rigidBodyDeletionNotification = 1,
   rigidBodyRemovalNotification,
   rigidBodyCopyNotification,
   rigidBodyForceNotification,
   rigidBodyUpdateNotification,
   rigidBodyMigrationNotification,
   rigidBodyRemoteMigrationNotification,
   rigidBodyVelocityUpdateNotification,
   rigidBodyVelocityCorrectionNotification,
};
//*************************************************************************************************




//=================================================================================================
//
//  NOTIFICATION UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Notification utility functions */
//@{

/*!\brief Returns the notification type of the template parameter.
 * \return The notification type of the template parameter.
 */
template<typename T>
NotificationType notificationType();
//@}
//*************************************************************************************************







//=================================================================================================
//
//  NOTIFICATION TYPE MARSHALLING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Notification type marshalling functions */
//@{
template< typename Buffer > inline void marshal( Buffer& buffer, const NotificationType& type );
template< typename Buffer > inline void unmarshal( Buffer& buffer, NotificationType& type );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const NotificationType& type ) {
   buffer << static_cast<byte>( type );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief TODO
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, NotificationType& type ) {
   byte tmp;
   buffer >> tmp;
   type = static_cast<NotificationType>( tmp );
}
//*************************************************************************************************


} // namespace pe

#endif
