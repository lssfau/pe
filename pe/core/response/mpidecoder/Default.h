//=================================================================================================
/*!
 *  \file pe/core/response/mpidecoder/Default.h
 *  \brief Default implementation of the response MPIDecoder class template
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2011 Tobias Preclik
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

#ifndef _PE_CORE_RESPONSE_MPIDECODER_DEFAULT_H_
#define _PE_CORE_RESPONSE_MPIDECODER_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <boost/tuple/tuple.hpp>
#include <pe/core/attachable/AttachableStorage.h>
#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/attachable/Gravity.h>
#include <pe/core/Marshalling.h>
#include <pe/core/MPIEntity.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/rigidbody/UnionSection.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/logging/DetailSection.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Decoder for the MPI communication of rigid bodies and attachables.
 * \ingroup mpi
 *
 * The MPIDecoder class template represents a decoder for rigid bodies and attachables in MPI
 * messages. It provides the functionality to decode MPI messages from remote processes encoded
 * with the MPIEncoder class.
 */
template< typename C >  // Type of the configuration
class MPIDecoder
{
public:
   //**Type definitions****************************************************************************
   typedef C                                   Config;             //!< Type of the configuration.

   typedef BodyStorage<Config>                 BS;                 //!< Type of the body storage.
   typedef typename Config::BodyID             BodyID;             //!< Handle to a rigid body.
   typedef typename Config::ConstBodyID        ConstBodyID;        //!< Handle to a constant rigid body.

   typedef typename Config::SphereID           SphereID;           //!< Handle to a sphere primitive.
   typedef typename Config::ConstSphereID      ConstSphereID;      //!< Handle to a constant sphere primitive.

   typedef typename Config::BoxID              BoxID;              //!< Handle to a box primitive.
   typedef typename Config::ConstBoxID         ConstBoxID;         //!< Handle to a constant box primitive.

   typedef typename Config::CapsuleID          CapsuleID;          //!< Handle to a capsule primitive.
   typedef typename Config::ConstCapsuleID     ConstCapsuleID;     //!< Handle to a constant capsule primitive.

   typedef typename Config::CylinderID         CylinderID;         //!< Handle to a cylinder primitive.
   typedef typename Config::ConstCylinderID    ConstCylinderID;    //!< Handle to a constant cylinder primitive.

   typedef typename Config::UnionID            UnionID;            //!< Handle to a union.
   typedef typename Config::ConstUnionID       ConstUnionID;       //!< Handle to a constant union.

   typedef AttachableStorage<Config>           AS;                 //!< Type of the attachable storage.
   typedef typename Config::AttachableID       AttachableID;       //!< Handle to an attachable.
   typedef typename Config::ConstAttachableID  ConstAttachableID;  //!< Handle to a constant attachable.

   typedef typename Config::GravityID          GravityID;          //!< Handle to a gravity force generator.
   typedef typename Config::ConstGravityID     ConstGravityID;     //!< Handle to a constant gravity force generator.

   typedef typename Config::SpringID           SpringID;           //!< Handle to a spring force generator.
   typedef typename Config::ConstSpringID      ConstSpringID;      //!< Handle to a constant spring force generator.

   //! Receive data for attachables.
   /*! In contrast to the decoding of rigid bodies, which can never fail, the decoding of
       attachables might fail in case an attached rigid body is not yet available. Therefore,
       instead of returning a handle to an attachable, the attachable decode functions
       return a data structure that contains the following information:
        - the type of attachable
        - the system-specific ID of the attachable
        - a flag that indicates whether the instantiation succeded */
   typedef boost::tuple<AttachableType,id_t,bool>  AttachableData;
   //**********************************************************************************************

private:
   //**Message tags********************************************************************************
   /*! Message tags. */
   enum {
      sphereTag   = 0,  //!< Message tag for sphere primitives.
      boxTag      = 1,  //!< Message tag for box primitives.
      capsuleTag  = 2,  //!< Message tag for capsule primitives.
      cylinderTag = 3,  //!< Message tag for cylinder primitives.
      unionTag    = 4,  //!< Message tag for unions.
      updateTag   = 5,  //!< Message tag for rigid body updates.
      gravityTag  = 6,  //!< Message tag for gravity force generators.
      springTag   = 7   //!< Message tag for spring force generators.
   };
   //**********************************************************************************************

public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit MPIDecoder( BS& bodystorage, AS& attachablestorage );
   //@}
   //**********************************************************************************************

   //**Decode functions****************************************************************************
   /*!\name Decode functions */
   //@{
   template< typename RB > inline BodyID         decodeBody      ( RB& buffer ) const;
   template< typename RB >        AttachableData decodeAttachable( RB& buffer ) const;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   template< typename RB > inline MPIEntity peek( const RB& buffer ) const;
   //@}
   //**********************************************************************************************

private:
   //**Decode functions****************************************************************************
   /*!\name Decode functions */
   //@{
   template< typename RB > BodyID         decodeSuperBody( RB& buffer ) const;
   template< typename RB > BodyID         decodeSubBody  ( RB& buffer ) const;
   template< typename RB > BodyID         decodeSphere   ( RB& buffer, bool superordinate ) const;
   template< typename RB > BodyID         decodeBox      ( RB& buffer, bool superordinate ) const;
   template< typename RB > BodyID         decodeCapsule  ( RB& buffer, bool superordinate ) const;
   template< typename RB > BodyID         decodeCylinder ( RB& buffer, bool superordinate ) const;
   template< typename RB > BodyID         decodeUnion    ( RB& buffer, bool superordinate ) const;
   template< typename RB > BodyID         decodeUpdate   ( RB& buffer ) const;
   template< typename RB > AttachableData decodeGravity  ( RB& buffer ) const;
   template< typename RB > AttachableData decodeSpring   ( RB& buffer ) const;
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   BS& bodystorage_;
   AS& attachablestorage_;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the default MPIDecoder implementation.
 *
 * \param bodystorage Reference to the central bodystorage.
 */
template< typename C >
inline MPIDecoder<C>::MPIDecoder( BS& bodystorage, AS& attachablestorage )
   : bodystorage_( bodystorage )
   , attachablestorage_( attachablestorage )
{
}
//*************************************************************************************************




//=================================================================================================
//
//  DECODE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Decoding a rigid body from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded body.
 * \return The received rigid body.
 * \exception std::runtime_error Received invalid message tag.
 *
 * This function decodes a rigid body received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
inline typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeBody( RB& buffer ) const
{
   pe_INTERNAL_ASSERT( !buffer.isEmpty()          , "Empty receive buffer detected" );
   pe_INTERNAL_ASSERT( peek( buffer ) == rigidbody, "Non-body entity detected"      );

   return decodeSuperBody( buffer );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding an attachable from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded attachable.
 * \return The received attachable.
 * \exception std::runtime_error Invalid encoded attachable
 * \exception std::runtime_error Received invalid message tag.
 *
 * This function decodes an attachable received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::AttachableData MPIDecoder<C>::decodeAttachable( RB& buffer ) const
{
   pe_INTERNAL_ASSERT( !buffer.isEmpty()           , "Empty receive buffer detected"  );
   pe_INTERNAL_ASSERT( peek( buffer ) == attachable, "Non-attachable entity detected" );

   byte tag;

   // Decoding the message tag of the next entity
   buffer >> tag;

   // Decoding the received data from the receive buffer
   switch( tag ) {
      case gravityTag:
         return decodeGravity( buffer );
         break;
      case springTag:
         return decodeSpring( buffer );
         break;
      default:
         throw std::runtime_error( "Received invalid message tag" );
         break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a superordinate rigid body from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded body.
 * \return The received superordinate rigid body.
 * \exception std::runtime_error Received invalid message tag.
 *
 * This function decodes a superordinate rigid body received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeSuperBody( RB& buffer ) const
{
   byte tag;

   // Decoding the message tag of the next message content
   buffer >> tag;

   // Decoding the received data from the receive buffer
   switch( tag ) {
      case sphereTag:
         return decodeSphere( buffer, true );
         break;
      case boxTag:
         return decodeBox( buffer, true );
         break;
      case capsuleTag:
         return decodeCapsule( buffer, true );
         break;
      case cylinderTag:
         return decodeCylinder( buffer, true );
         break;
      case unionTag:
         return decodeUnion( buffer, true );
         break;
      case updateTag:
         return decodeUpdate( buffer );
         break;
      default:
         throw std::runtime_error( "Received invalid message tag" );
         break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a subordinate rigid body from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded body.
 * \return The received subordinate rigid body.
 * \exception std::runtime_error Received invalid message tag.
 *
 * This function decodes a subordinate rigid body received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeSubBody( RB& buffer ) const
{
   byte tag;

   // Decoding the message tag of the next message content
   buffer >> tag;

   // Decoding the received data from the receive buffer
   switch( tag ) {
      case sphereTag:
         return decodeSphere( buffer, false );
         break;
      case boxTag:
         return decodeBox( buffer, false );
         break;
      case capsuleTag:
         return decodeCapsule( buffer, false );
         break;
      case cylinderTag:
         return decodeCylinder( buffer, false );
         break;
      case unionTag:
         return decodeUnion( buffer, false );
         break;
      case updateTag:
         throw std::runtime_error( "Received invalid update tag" );
         break;
      default:
         throw std::runtime_error( "Received invalid message tag" );
         break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a sphere primitive from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded sphere.
 * \param superordinate \a true if the sphere is not contained in another body, \a false if it is.
 * \return void
 *
 * This function decodes a sphere primitive received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeSphere( RB& buffer, bool superordinate ) const
{
   bool visible, fixed;
   id_t sid, uid;
   MaterialID material;
   real radius;
   Vec3 gpos, rpos, v, w;
   Quat q;

   // Decoding the received sphere data
   buffer >> sid;       // Decoding the unique system-specific ID
   buffer >> uid;       // Decoding the user-specific ID
   buffer >> radius;    // Decoding the radius of the sphere
   buffer >> material;  // Decoding the material of the sphere
   buffer >> visible;   // Decoding the visibility flag
   buffer >> fixed;     // Decoding the fixation flag
   unmarshal( buffer, gpos );      // Decoding the global position

   if( !superordinate ) {
      unmarshal( buffer, rpos );   // Decoding the relative position
   }

   unmarshal( buffer, q );         // Decoding the orientation of the sphere

   if( superordinate ) {
      unmarshal( buffer, v );      // Decoding the linear velocity
      unmarshal( buffer, w );      // Decoding the angular velocity
   }

   // Checking that the sphere primitive is unknown on this process
   pe_INTERNAL_ASSERT( bodystorage_.find( sid ) == bodystorage_.end(), "Sphere primitive already known" );

   // Instantiating a remote sphere primitive
   BodyID body = instantiateSphere( sid, uid, gpos, rpos, q, radius, material, visible, fixed );
   body->setUpdated( true );

   if( superordinate ) {
      body->setLinearVel ( v );
      body->setAngularVel( w );
   }

   return body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a box primitive from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded box.
 * \param superordinate \a true if the box is not contained in another body, \a false if it is.
 * \return void
 *
 * This function decodes a box primitive received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeBox( RB& buffer, bool superordinate ) const
{
   bool visible, fixed;
   id_t sid, uid;
   MaterialID material;
   Vec3 lengths, gpos, rpos, v, w;
   Quat q;
   BodyID body;

   // Decoding the received box data
   buffer >> sid;       // Decoding the unique system-specific ID
   buffer >> uid;       // Decoding the user-specific ID
   unmarshal( buffer, lengths );   // Decoding the side lengths of the box
   buffer >> material;  // Decoding the material of the box
   buffer >> visible;   // Decoding the visibility flag
   buffer >> fixed;     // Decoding the fixation flag
   unmarshal( buffer, gpos );      // Decoding the global position

   if( !superordinate ) {
      unmarshal( buffer, rpos );   // Decoding the relative position
   }

   unmarshal( buffer, q );         // Decoding the orientation of the box

   if( superordinate ) {
      unmarshal( buffer, v );      // Decoding the linear velocity
      unmarshal( buffer, w );      // Decoding the angular velocity
   }

   // Checking that the box primitive is unknown on this process
   pe_INTERNAL_ASSERT( bodystorage_.find( sid ) == bodystorage_.end(), "Box primitive already known" );

   // Instantiating a remote box primitive
   body = instantiateBox( sid, uid, gpos, rpos, q, lengths, material, visible, fixed );
   body->setUpdated( true );

   if( superordinate ) {
      body->setLinearVel ( v );
      body->setAngularVel( w );
   }

   return body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a capsule primitive from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded capsule.
 * \param superordinate \a true if the capsule is not contained in another body, \a false if it is.
 * \return void
 *
 * This function decodes a capsule primitive received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeCapsule( RB& buffer, bool superordinate ) const
{
   bool visible, fixed;
   id_t sid, uid;
   MaterialID material;
   real radius, length;
   Vec3 gpos, rpos, v, w;
   Quat q;

   // Decoding the received capsule data
   buffer >> sid;       // Decoding the unique system-specific ID
   buffer >> uid;       // Decoding the user-specific ID
   buffer >> radius;    // Decoding the radius of the capsule
   buffer >> length;    // Decoding the length of the capsule
   buffer >> material;  // Decoding the material of the capsule
   buffer >> visible;   // Decoding the visibility flag
   buffer >> fixed;     // Decoding the fixation flag
   unmarshal( buffer, gpos );      // Decoding the global position

   if( !superordinate ) {
      unmarshal( buffer, rpos );   // Decoding the relative position
   }

   unmarshal( buffer, q );         // Decoding the orientation of the capsule

   if( superordinate ) {
      unmarshal( buffer, v );      // Decoding the linear velocity
      unmarshal( buffer, w );      // Decoding the angular velocity
   }

   // Checking that the capsule primitive is unknown on this process
   pe_INTERNAL_ASSERT( bodystorage_.find( sid ) == bodystorage_.end(), "Capsule primitive already known" );

   // Instantiating a remote capsule primitive
   BodyID body = instantiateCapsule( sid, uid, gpos, rpos, q, radius, length, material, visible, fixed );
   body->setUpdated( true );

   if( superordinate ) {
      body->setLinearVel ( v );
      body->setAngularVel( w );
   }

   return body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a cylinder primitive from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded cylinder.
 * \param superordinate \a true if the cylinder is not contained in another body, \a false if it is.
 * \return void
 *
 * This function decodes a cylinder primitive received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeCylinder( RB& buffer, bool superordinate ) const
{
   bool visible, fixed;
   id_t sid, uid;
   MaterialID material;
   real radius, length;
   Vec3 gpos, rpos, v, w;
   Quat q;

   // Decoding the received cylinder data
   buffer >> sid;       // Decoding the unique system-specific ID
   buffer >> uid;       // Decoding the user-specific ID
   buffer >> radius;    // Decoding the radius of the cylinder
   buffer >> length;    // Decoding the length of the cylinder
   buffer >> material;  // Decoding the material of the cylinder
   buffer >> visible;   // Decoding the visibility flag
   buffer >> fixed;     // Decoding the fixation flag
   unmarshal( buffer, gpos );      // Decoding the global position

   if( !superordinate ) {
      unmarshal( buffer, rpos );   // Decoding the relative position
   }

   unmarshal( buffer, q );         // Decoding the orientation of the cylinder

   if( superordinate ) {
      unmarshal( buffer, v );      // Decoding the linear velocity
      unmarshal( buffer, w );      // Decoding the angular velocity
   }

   // Checking that the cylinder primitive is unknown on this process
   pe_INTERNAL_ASSERT( bodystorage_.find( sid ) == bodystorage_.end(), "Cylinder primitive already known" );

   // Instantiating a remote cylinder primitive
   BodyID body = instantiateCylinder( sid, uid, gpos, rpos, q, radius, length, material, visible, fixed );
   body->setUpdated( true );

   if( superordinate ) {
      body->setLinearVel ( v );
      body->setAngularVel( w );
   }

   return body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a union compound geometry from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded union.
 * \param superordinate \a true if the union is not contained in another body, \a false if it is.
 * \return void
 *
 * This function decodes a union received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeUnion( RB& buffer, bool superordinate ) const
{
   bool visible, fixed;
   id_t sid, uid;
   size_t size;
   real mass;
   Vec3 gpos, rpos, v, w;
   Vec6 aabb;
   Mat3 I;
   Quat q;
   UnionID u( 0 );

   // Decoding the received union data
   buffer >> sid;      // Decoding the unique system-specific ID
   buffer >> uid;      // Decoding the user-specific ID
   buffer >> visible;  // Decoding the visibility flag
   buffer >> fixed;    // Decoding the fixation flag
   unmarshal( buffer, gpos );     // Decoding the global position

   if( !superordinate ) {
      unmarshal( buffer, rpos );  // Decoding the relative position
   }

   buffer >> mass;     // Decoding the total mass
   unmarshal( buffer, I );        // Decoding the moment of inertia
   unmarshal( buffer, q );        // Decoding the orientation of the sphere
   unmarshal( buffer, aabb );     // Decoding the axis-aligned bounding box

   if( superordinate ) {
      unmarshal( buffer, v );     // Decoding the linear velocity
      unmarshal( buffer, w );     // Decoding the angular velocity
   }

   buffer >> size;     // Decoding the number of contained primitives

   // Checking that the union is unknown on this process
   pe_INTERNAL_ASSERT( bodystorage_.find( sid ) == bodystorage_.end(), "Union already known" );

   // Instantiating a remote union
   pe_INSTANTIATE_UNION( handle, sid, uid, gpos, rpos, mass, I, q, aabb, visible, fixed, true )
   {
      u = handle;
      u->setUpdated( true );

      if( superordinate ) {
         u->setLinearVel ( v );
         u->setAngularVel( w );
      }

      for( size_t i=0; i<size; ++i ) {
         u->add( decodeSubBody( buffer ) );
      }
   }

   return u;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a rigid body update from the data received from the remote MPI process.
 *
 * \param buffer The receive buffer containing the rigid body update.
 * \return void
 *
 * This function decodes a rigid body update received from the connected remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::BodyID MPIDecoder<C>::decodeUpdate( RB& buffer ) const
{
   bool visible;
   id_t sid;
   Vec3 gpos, v, w;
   Quat q;

   // Decoding the received rigid body data
   buffer >> sid;      // Decoding the unique system-specific ID
   buffer >> visible;  // Decoding the visibility flag
   unmarshal( buffer, gpos );     // Decoding the global position
   unmarshal( buffer, q );        // Decoding the orientation of the sphere
   unmarshal( buffer, v );        // Decoding the linear velocity
   unmarshal( buffer, w );        // Decoding the angular velocity

   // Searching for the rigid body
   typedef typename BS::Iterator  Iterator;
   const Iterator begin( bodystorage_.begin() );
   const Iterator end  ( bodystorage_.end()   );
   Iterator pos( bodystorage_.find( sid ) );

   // Checking that the rigid body is known on this process
   pe_INTERNAL_ASSERT( pos != end, "Update of an unknown sphere primitive" );
   BodyID body = *pos;

   // Checking the finiteness, mobility, locality, and the update flag of the rigid body
   pe_INTERNAL_ASSERT( body->isFinite()  , "Invalid infinite rigid body detected" );
   pe_INTERNAL_ASSERT( !body->isFixed()  , "Invalid fixed rigid body detected"    );
   pe_INTERNAL_ASSERT( body->isRemote()  , "Invalid local rigid body detected"    );
   pe_INTERNAL_ASSERT( !body->isUpdated(), "Invalid update flag detected"         );

   // Updating the rigid body
   body->setUpdated    ( true );
   body->setPosition   ( gpos );
   body->setLinearVel  ( v );
   body->setAngularVel ( w );
   body->setOrientation( q );

   return body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a gravity force generator from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded gravity force generator.
 * \return void
 *
 * This function decodes a gravity force generator received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::AttachableData MPIDecoder<C>::decodeGravity( RB& buffer ) const
{
   id_t sid, body;
   Vec3 gravity;

   // Decoding the received gravity data
   buffer >> sid;
   buffer >> body;
   unmarshal( buffer, gravity );

   // Searching the attached rigid body
   typename BS::Iterator pos( bodystorage_.find( body ) );
   pe_INTERNAL_ASSERT( pos != bodystorage_.end(), "Unknown attached rigid body" );

   // Instantiating a gravity force generator
   instantiateGravity( sid, *pos, gravity );

   return boost::make_tuple( gravityType, sid, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding a spring force generator from the data received from a remote MPI process.
 *
 * \param buffer The receive buffer containing the encoded spring force generator.
 * \return void
 *
 * This function decodes a spring force generator received from a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
typename MPIDecoder<C>::AttachableData MPIDecoder<C>::decodeSpring( RB& buffer ) const
{
   bool visible;
   id_t sid, body1, body2;
   real stiffness, damping, length;
   Vec3 anchor1, anchor2;

   // Decoding the received spring data
   buffer >> sid;        // Decoding the unique system-specific ID
   buffer >> body1;      // Decoding the unique system-specific ID of the first attached rigid body
   buffer >> body2;      // Decoding the unique system-specific ID of the second attached rigid body
   unmarshal( buffer, anchor1 );    // Decoding the anchor point of the first attached rigid body
   unmarshal( buffer, anchor2 );    // Decoding the anchor point of the second attached rigid body
   buffer >> stiffness;  // Decoding the spring stiffness
   buffer >> damping;    // Decoding the spring damping
   buffer >> length;     // Decoding the spring rest length
   buffer >> visible;    // Decoding the visibility flag

   // Checking that the spring force generator is unknown on this process
   pe_INTERNAL_ASSERT( attachablestorage_.find( sid ) == attachablestorage_.end(), "Spring force generator already known" );

   // Searching the first attached rigid body
   typename BS::Iterator pos1( bodystorage_.find( body1 ) );
   if( pos1 == bodystorage_.end() ) return boost::make_tuple( springType, sid, false );

   // Searching the second attached rigid body
   typename BS::Iterator pos2( bodystorage_.find( body2 ) );
   if( pos2 == bodystorage_.end() ) return boost::make_tuple( springType, sid, false );

   // Instantiating a spring force generator
   instantiateSpring( sid, *pos1, anchor1, *pos2, anchor2, stiffness, damping, length, visible );

   return boost::make_tuple( springType, sid, true );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Reading the type of the next entity in the receive buffer.
 *
 * \param buffer The non-empty receive buffer containing the next entity.
 * \return \a rigidbody if the next entity is a rigid body, \a attachable if it is an attachable.
 *
 * This function reads the type of the next entity in the receive buffer without extracting it
 * from the buffer. In case the next entity is a rigid body, the function returns \a rigidbody,
 * in case it is an attachable it returns \a attachable. The attempt to peek into an empty
 * receive buffer results in a \a std::invalid_argument exception.
 */
template< typename C >   // Type of the configuration
template< typename RB >  // Type of the receive buffer
inline MPIEntity MPIDecoder<C>::peek( const RB& buffer ) const
{
   pe_INTERNAL_ASSERT( !buffer.isEmpty(), "Empty receive buffer detected" );

   byte tag;
   buffer.peek( tag );

   if( tag < 6 ) return rigidbody;
   else return attachable;
}
//*************************************************************************************************

} // namespace pe

#endif
