//=================================================================================================
/*!
 *  \file pe/core/rigidbody/SuperBody.h
 *  \brief Header file for the SuperBody class
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

#ifndef _PE_CORE_RIGIDBODY_SUPERBODY_H_
#define _PE_CORE_RIGIDBODY_SUPERBODY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/GeomType.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/Types.h>
#include <pe/math/Quaternion.h>
#include <pe/math/Vector3.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for superordinate rigid bodies.
 * \ingroup core
 *
 * The SuperBody class is the abstract basis for all superordinate rigid bodies. This base class
 * provides the necessary privileges to manage subordinate rigid bodies for all bodies consisting
 * of several other rigid bodies.
 */
class SuperBody : public RigidBody
{
protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit SuperBody( GeomType type, bool finite, bool visible, id_t sid, id_t uid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~SuperBody() = 0;
   //@}
   //**********************************************************************************************

   //**Superbody functions*************************************************************************
   /*!\name Superbody functions */
   //@{
          inline void setSuperBody       ( BodyID body );
          inline void resetSuperBody     ( BodyID body );
          inline void registerSuperBody  ( BodyID body );
          inline void deregisterSuperBody( BodyID body );

   static inline void updateRelPosition( BodyID body );
   static inline void updateBody       ( BodyID body, const Vec3& dp );
   static inline void updateBody       ( BodyID body, const Quat& dq );
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template< typename C > friend class CollisionSystem;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  SUPERBODY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the superordinate body of a rigid body.
 *
 * \param body The subordinate rigid body.
 *
 * This function registers this SuperBody as the superordinate body of the given rigid body.
 */
inline void SuperBody::setSuperBody( BodyID body )
{
   body->sb_ = this;  // Setting the superordinate body
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting the superordinate body of a rigid body.
 *
 * \param body The subordinate rigid body.
 *
 * This function deregisteres this SuperBody as the superordinate body of the given rigid body
 * and makes the rigid body his own superordinate body.
 */
inline void SuperBody::resetSuperBody( BodyID body )
{
   body->sb_ = body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Registering a superordinate body.
 *
 * \param body The subordinate rigid body.
 *
 * This function registers this SuperBody as a superordinate body with the given rigid body.
 */
inline void SuperBody::registerSuperBody( BodyID body )
{
   body->superBodies_.pushBack( this );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregistering a superordinate body.
 *
 * \param body The subordinate rigid body.
 *
 * This function deregisters this SuperBody as a superordinate body.
 */
inline void SuperBody::deregisterSuperBody( BodyID body )
{
   RigidBody::Bodies& bodies( body->superBodies_ );
   const RigidBody::Bodies::Iterator pos( std::find( bodies.begin(), bodies.end(), this ) );
   pe_INTERNAL_ASSERT( pos != bodies.end(), "Super body is not registered" );
   bodies.erase( pos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the relative position of a rigid body contained in a superordinate body.
 *
 * \param body The subordinate rigid body.
 *
 * This function triggers the update of the relative position of a subordinate rigid body
 * contained in this SuperBody.
 */
inline void SuperBody::updateRelPosition( BodyID body )
{
   body->calcRelPosition();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation update of a subordinate rigid body.
 *
 * \param body The subordinate rigid body.
 * \param dp Change in the global position of the superordinate rigid body.
 *
 * This function triggers an update of a subordinate rigid body contained in this SuperBody.
 */
inline void SuperBody::updateBody( BodyID body, const Vec3& dp )
{
   body->update( dp );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation update of a subordinate rigid body.
 *
 * \param body The subordinate rigid body.
 * \param dq Change in the orientation of the superordinate rigid body.
 *
 * This function triggers an update of a subordinate rigid body contained in this SuperBody. 
 */
inline void SuperBody::updateBody( BodyID body, const Quat& dq )
{
   body->update( dq );
}
//*************************************************************************************************

} // namespace pe

#endif
