//=================================================================================================
/*!
 *  \file src/core/notifications/NotificationType.cpp
 *  \brief Source file for the notification types
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/notifications/NotificationType.h>
#include <pe/core/notifications/RigidBodyCopyNotification.h>
#include <pe/core/notifications/RigidBodyDeletionNotification.h>
#include <pe/core/notifications/RigidBodyForceNotification.h>
#include <pe/core/notifications/RigidBodyMigrationNotification.h>
#include <pe/core/notifications/RigidBodyRemoteMigrationNotification.h>
#include <pe/core/notifications/RigidBodyRemovalNotification.h>
#include <pe/core/notifications/RigidBodyUpdateNotification.h>
#include <pe/core/notifications/RigidBodyVelocityUpdateNotification.h>
#include <pe/core/notifications/RigidBodyVelocityCorrectionNotification.h>


namespace pe {

//!\cond pe_INTERNAL

//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body update.
 * \return The notification type of a rigid body update.
 */
template<>
NotificationType notificationType< RigidBodyUpdateNotification<Config> >() {
   return rigidBodyUpdateNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body copy.
 * \return The notification type of a rigid body copy.
 */
template<>
NotificationType notificationType< RigidBodyCopyNotification<Config> >() {
   return rigidBodyCopyNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body deletion.
 * \return The notification type of a rigid body deletion.
 */
template<>
NotificationType notificationType<RigidBodyDeletionNotification>() {
   return rigidBodyDeletionNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body force notification.
 * \return The notification type of a rigid body notification.
 */
template<>
NotificationType notificationType<RigidBodyForceNotification>() {
   return rigidBodyForceNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body removal.
 * \return The notification type of a rigid body removal.
 */
template<>
NotificationType notificationType<RigidBodyRemovalNotification>() {
   return rigidBodyRemovalNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body migration.
 * \return The notification type of a rigid body migration.
 */
template<>
NotificationType notificationType<RigidBodyMigrationNotification>() {
   return rigidBodyMigrationNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body migration.
 * \return The notification type of a rigid body migration.
 */
template<>
NotificationType notificationType<RigidBodyRemoteMigrationNotification>() {
   return rigidBodyRemoteMigrationNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body migration.
 * \return The notification type of a rigid body velocity update.
 */
template<>
NotificationType notificationType<RigidBodyVelocityUpdateNotification>() {
   return rigidBodyVelocityUpdateNotification;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the notification type of a rigid body migration.
 * \return The notification type of a rigid body velocity correction.
 */
template<>
NotificationType notificationType<RigidBodyVelocityCorrectionNotification>() {
   return rigidBodyVelocityCorrectionNotification;
}
//*************************************************************************************************

//!\endcond

} // namespace pe
