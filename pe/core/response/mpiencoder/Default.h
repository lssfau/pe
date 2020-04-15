//=================================================================================================
/*!
 *  \file pe/core/response/mpiencoder/Default.h
 *  \brief Default implementation of the response MPIEncoder class template
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

#ifndef _PE_CORE_RESPONSE_MPIENCODER_DEFAULT_H_
#define _PE_CORE_RESPONSE_MPIENCODER_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/core/attachable/AttachableCast.h>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/attachable/Gravity.h>
#include <pe/core/response/Types.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/core/attachable/Spring.h>
#include <pe/math/Vector3.h>
#include <pe/util/Assert.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Encoder for the MPI communication of rigid bodies and attachables.
 * \ingroup mpi
 *
 * The MPIEncoder class template represents an encoder for rigid bodies and attachables to
 * MPI messages. It provides the functionality to encode all primitive rigid body types,
 * union compounds and attachables such that they can be sent via MPI to remote processes,
 * where they can be decoded with the MPIDecoder class.
 */
template< typename C >  // Type of the configuration
class MPIEncoder
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

   typedef typename Config::AttachableID       AttachableID;       //!< Handle to an attachable.
   typedef typename Config::ConstAttachableID  ConstAttachableID;  //!< Handle to a constant attachable.

   typedef typename Config::GravityID          GravityID;          //!< Handle to a gravity force generator.
   typedef typename Config::ConstGravityID     ConstGravityID;     //!< Handle to a constant gravity force generator.

   typedef typename Config::SpringID           SpringID;           //!< Handle to a spring force generator.
   typedef typename Config::ConstSpringID      ConstSpringID;      //!< Handle to a constant spring force generator.
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
   //**Encode functions****************************************************************************
   /*!\name Encode functions */
   //@{
   template< typename SB >
   inline static void encodeBody( SB& buffer, ConstBodyID b, const Vec3& offset );

   template< typename SB >
   inline static void encodeSphere( SB& buffer, ConstSphereID s, const Vec3& offset );

   template< typename SB >
   inline static void encodeBox( SB& buffer, ConstBoxID b, const Vec3& offset );

   template< typename SB >
   inline static void encodeCapsule( SB& buffer, ConstCapsuleID c, const Vec3& offset );

   template< typename SB >
   inline static void encodeCylinder( SB& buffer, ConstCylinderID c, const Vec3& offset );

   template< typename SB >
   inline static void encodeUnion( SB& buffer, ConstUnionID u, const Vec3& offset );

   template< typename SB >
   static void encodeUpdate( SB& buffer, ConstBodyID b, const Vec3& offset );

   template< typename SB >
   static void encodeAttachable( SB& buffer, ConstAttachableID a );

   template< typename SB >
   static void encodeGravity( SB& buffer, ConstGravityID g );

   template< typename SB >
   static void encodeSpring( SB& buffer, ConstSpringID s );
   //@}
   //**********************************************************************************************

private:
   //**Encode functions****************************************************************************
   /*!\name Encode functions */
   //@{
   template< typename SB >
   static void encodeBody( SB& buffer, ConstBodyID b, const Vec3& offset, bool superordinate );

   template< typename SB >
   static void encodeSphere( SB& buffer, ConstSphereID s, const Vec3& offset, bool superordinate );

   template< typename SB >
   static void encodeBox( SB& buffer, ConstBoxID b, const Vec3& offset, bool superordinate );

   template< typename SB >
   static void encodeCapsule( SB& buffer, ConstCapsuleID c, const Vec3& offset, bool superordinate );

   template< typename SB >
   static void encodeCylinder( SB& buffer, ConstCylinderID c, const Vec3& offset, bool superordinate );

   template< typename SB >
   static void encodeUnion( SB& buffer, ConstUnionID u, const Vec3& offset, bool superordinate );
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  ENCODE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Encoding a rigid body for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param b The rigid body to be encoded.
 * \param offset The positional offset of the rigid body.
 * \return void
 * \exception std::runtime_error Unknown geometry type.
 *
 * This function encodes the given rigid body for the send operation to a remote MPI process.
 * In case pe_INTERNAL_ASSERT() is active, the finite flag and the remote flag of the rigid
 * body are tested.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
inline void MPIEncoder<C>::encodeBody( SB& buffer, ConstBodyID b, const Vec3& offset )
{
   // Checking whether the body is contained in another rigid body
   pe_INTERNAL_ASSERT( !b->hasSuperBody(), "Invalid subordinate rigid body detected" );

   // Checking the global flag and the locality of the rigid body
   pe_INTERNAL_ASSERT( !b->isGlobal(), "Invalid global rigid body detected" );
   pe_INTERNAL_ASSERT( !b->isRemote(), "Invalid remote rigid body detected" );

   // Encoding the rigid body
   encodeBody( buffer, b, offset, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a rigid body for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param b The rigid body to be encoded.
 * \param offset The positional offset of the rigid body.
 * \param superordinate \a true if the body is not contained in another body, \a false if it is.
 * \return void
 * \exception std::runtime_error Unknown geometry type.
 *
 * This function encodes the given rigid body for the send operation to a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeBody( SB& buffer, ConstBodyID b, const Vec3& offset, bool superordinate )
{
   switch( b->getType() ) {
      case sphereType:
         encodeSphere( buffer, static_body_cast<const Sphere>( b ), offset, superordinate );
         break;
      case boxType:
         encodeBox( buffer, static_body_cast<const Box>( b ), offset, superordinate );
         break;
      case capsuleType:
         encodeCapsule( buffer, static_body_cast<const Capsule>( b ), offset, superordinate );
         break;
      case cylinderType:
         encodeCylinder( buffer, static_body_cast<const Cylinder>( b ), offset, superordinate );
         break;
      case unionType:
         encodeUnion( buffer, static_body_cast<const Union>( b ), offset, superordinate );
         break;
      default:
         throw std::runtime_error( "Unknown geometry type" );
         break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a sphere primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param s The sphere to be encoded.
 * \param offset The positional offset of the sphere.
 * \return void
 *
 * This function encodes the given sphere primitive for the send operation to a remote MPI
 * process. In case pe_INTERNAL_ASSERT() is active, it is tested whether the sphere is not
 * contained in another rigid body. Additionally, the finite flag and the remote flag of the
 * sphere primitive are tested.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
inline void MPIEncoder<C>::encodeSphere( SB& buffer, ConstSphereID s, const Vec3& offset )
{
   // Checking whether the sphere is contained in another rigid body
   pe_INTERNAL_ASSERT( !s->hasSuperBody(), "Invalid subordinate sphere primitive detected" );

   // Checking the global flag and the locality of the sphere primitive
   pe_INTERNAL_ASSERT( !s->isGlobal(), "Invalid global sphere primitive detected" );
   pe_INTERNAL_ASSERT( !s->isRemote(), "Invalid remote sphere primitive detected" );

   // Encoding the sphere primitive
   encodeSphere( buffer, s, offset, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a sphere primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param s The sphere to be encoded.
 * \param offset The positional offset of the sphere.
 * \param superordinate \a true if the sphere is not contained in another body, \a false if it is.
 * \return void
 *
 * This function encodes the given sphere primitive for the send operation to a remote MPI
 * process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeSphere( SB& buffer, ConstSphereID s, const Vec3& offset, bool superordinate )
{
   // Encoding the sphere primitive
   buffer << static_cast<byte>( sphereTag );  // Encoding the message tag of the sphere
   buffer << s->getSystemID();                // Encoding the unique system-specific ID
   buffer << s->getID();                      // Encoding the user-specific ID
   buffer << s->getRadius();                  // Encoding the radius
   buffer << s->getMaterial();                // Encoding the material
   buffer << s->isVisible();                  // Encoding the visibility flag
   buffer << s->isFixed();                    // Encoding the fixation flag
   marshal( buffer, s->getPosition() + offset );       // Encoding the global position

   if( !superordinate ) {
      marshal( buffer, s->getRelPosition() );          // Encoding the relative position
   }

   marshal( buffer, s->getQuaternion() );              // Encoding the orientation of the sphere

   if( superordinate ) {
      marshal( buffer, s->getLinearVel() );            // Encoding the linear velocity
      marshal( buffer, s->getAngularVel() );           // Encoding the angular velocity
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a box primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param b The box to be encoded.
 * \param offset The positional offset of the box.
 * \return void
 *
 * This function encodes the given box primitive for the send operation to a remote MPI
 * process. In case pe_INTERNAL_ASSERT() is active, it is tested whether the box is not
 * contained in another rigid body. Additionally, the finite flag and the remote flag of
 * the box primitive are tested.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
inline void MPIEncoder<C>::encodeBox( SB& buffer, ConstBoxID b, const Vec3& offset )
{
   // Checking whether the box is contained in another rigid body
   pe_INTERNAL_ASSERT( !b->hasSuperBody(), "Invalid subordinate box primitive detected" );

   // Checking the global flag and the locality of the box primitive
   pe_INTERNAL_ASSERT( !b->isGlobal(), "Invalid global box primitive detected" );
   pe_INTERNAL_ASSERT( !b->isRemote(), "Invalid remote box primitive detected" );

   // Encoding the box primitive
   encodeBox( buffer, b, offset, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a box primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param b The box to be encoded.
 * \param offset The positional offset of the box.
 * \param superordinate \a true if the box is not contained in another body, \a false if it is.
 * \return void
 *
 * This function encodes the given box primitive for the send operation to a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeBox( SB& buffer, ConstBoxID b, const Vec3& offset, bool superordinate )
{
   // Encoding the box primitive
   buffer << static_cast<byte>( boxTag );  // Encoding the message tag of the box
   buffer << b->getSystemID();             // Encoding the unique system-specific ID
   buffer << b->getID();                   // Encoding the user-specific ID
   marshal( buffer, b->getLengths() );              // Encoding the box side lengths
   buffer << b->getMaterial();             // Encoding the material
   buffer << b->isVisible();               // Encoding the visibility flag
   buffer << b->isFixed();                 // Encoding the fixation flag
   marshal( buffer, b->getPosition() + offset );    // Encoding the global position

   if( !superordinate ) {
      marshal( buffer, b->getRelPosition() );       // Encoding the relative position
   }

   marshal( buffer, b->getQuaternion() );           // Encoding the orientation of the box

   if( superordinate ) {
      marshal( buffer, b->getLinearVel() );         // Encoding the linear velocity
      marshal( buffer, b->getAngularVel() );        // Encoding the angular velocity
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a capsule primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param c The capsule to be encoded.
 * \param offset The positional offset of the capsule.
 * \return void
 *
 * This function encodes the given capsule primitive for the send operation to a remote MPI
 * process. In case pe_INTERNAL_ASSERT() is active, it is tested whether the capsule is not
 * contained in another rigid body. Additionally, the finite flag and the remote flag of
 * the capsule primitive are tested.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
inline void MPIEncoder<C>::encodeCapsule( SB& buffer, ConstCapsuleID c, const Vec3& offset )
{
   // Checking whether the capsule is contained in another rigid body
   pe_INTERNAL_ASSERT( !c->hasSuperBody(), "Invalid subordinate capsule primitive detected" );

   // Checking the global flag and the locality of the capsule primitive
   pe_INTERNAL_ASSERT( !c->isGlobal(), "Invalid global capsule primitive detected" );
   pe_INTERNAL_ASSERT( !c->isRemote(), "Invalid remote capsule primitive detected" );

   // Encoding the capsule primitive
   encodeCapsule( buffer, c, offset, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a capsule primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param c The capsule to be encoded.
 * \param offset The positional offset of the capsule.
 * \param superordinate \a true if the capsule is not contained in another body, \a false if it is.
 * \return void
 *
 * This function encodes the given capsule primitive for the send operation to a remote MPI
 * process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeCapsule( SB& buffer, ConstCapsuleID c, const Vec3& offset, bool superordinate )
{
   // Encoding the capsule primitive
   buffer << static_cast<byte>( capsuleTag );  // Encoding the message tag of the capsule
   buffer << c->getSystemID();                 // Encoding the unique system-specific ID
   buffer << c->getID();                       // Encoding the user-specific ID
   buffer << c->getRadius();                   // Encoding the radius of the capsule
   buffer << c->getLength();                   // Encoding the length of the capsule
   buffer << c->getMaterial();                 // Encoding the material
   buffer << c->isVisible();                   // Encoding the visibility flag
   buffer << c->isFixed();                     // Encoding the fixation flag
   marshal( buffer, c->getPosition() + offset );        // Encoding the global position

   if( !superordinate ) {
      marshal( buffer, c->getRelPosition() );           // Encoding the relative position
   }

   marshal( buffer, c->getQuaternion() );               // Encoding the orientation of the capsule

   if( superordinate ) {
      marshal( buffer, c->getLinearVel() );             // Encoding the linear velocity
      marshal( buffer, c->getAngularVel() );            // Encoding the angular velocity
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a cylinder primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param c The cylinder to be encoded.
 * \param offset The positional offset of the cylinder.
 * \return void
 *
 * This function encodes the given cylinder primitive for the send operation to a remote MPI
 * process. In case pe_INTERNAL_ASSERT() is active, it is tested whether the cylinder is not
 * contained in another rigid body. Additionally, the finite flag and the remote flag of
 * the cylinder primitive are tested.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
inline void MPIEncoder<C>::encodeCylinder( SB& buffer, ConstCylinderID c, const Vec3& offset )
{
   // Checking whether the cylinder is contained in another rigid body
   pe_INTERNAL_ASSERT( !c->hasSuperBody(), "Invalid subordinate cylinder primitive detected" );

   // Checking the global flag and the locality of the cylinder primitive
   pe_INTERNAL_ASSERT( !c->isGlobal(), "Invalid global cylinder primitive detected" );
   pe_INTERNAL_ASSERT( !c->isRemote(), "Invalid remote cylinder primitive detected" );

   // Encoding the cylinder primitive
   encodeCylinder( buffer, c, offset, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a cylinder primitive for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param c The cylinder to be encoded.
 * \param offset The positional offset of the cylinder.
 * \param superordinate \a true if the cylinder is not contained in another body, \a false if it is.
 * \return void
 *
 * This function encodes the given cylinder primitive for the send operation to a remote MPI
 * process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeCylinder( SB& buffer, ConstCylinderID c, const Vec3& offset, bool superordinate )
{
   // Encoding the cylinder primitive
   buffer << static_cast<byte>( cylinderTag );  // Encoding the message tag of the cylinder
   buffer << c->getSystemID();                  // Encoding the unique system-specific ID
   buffer << c->getID();                        // Encoding the user-specific ID
   buffer << c->getRadius();                    // Encoding the radius of the cylinder
   buffer << c->getLength();                    // Encoding the length of the cylinder
   buffer << c->getMaterial();                  // Encoding the material
   buffer << c->isVisible();                    // Encoding the visibility flag
   buffer << c->isFixed();                      // Encoding the fixation flag
   marshal( buffer, c->getPosition() + offset );         // Encoding the global position

   if( !superordinate ) {
      marshal( buffer, c->getRelPosition() );            // Encoding the relative position
   }

   marshal( buffer, c->getQuaternion() );                // Encoding the orientation of the cylinder

   if( superordinate ) {
      marshal( buffer, c->getLinearVel() );              // Encoding the linear velocity
      marshal( buffer, c->getAngularVel() );             // Encoding the angular velocity
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a union compound geometry for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param u The union to be encoded.
 * \param offset The positional offset of the union.
 * \return void
 * \exception std::runtime_error Unknown geometry type.
 *
 * This function encodes the given union for the send operation to a remote MPI process.
 * In case pe_INTERNAL_ASSERT() is active, it is tested whether the union is not contained
 * in another rigid body. Additionally, the finite flag and the remote flag of the union
 * are tested.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
inline void MPIEncoder<C>::encodeUnion( SB& buffer, ConstUnionID u, const Vec3& offset )
{
   // Checking whether the cylinder is contained in another rigid body
   pe_INTERNAL_ASSERT( !u->hasSuperBody(), "Invalid subordinate union detected" );

   // Checking the global flag and the locality of the union
   pe_INTERNAL_ASSERT( !u->isGlobal(), "Invalid global union detected" );
   pe_INTERNAL_ASSERT( !u->isRemote(), "Invalid remote union detected" );

   // Encoding the union
   encodeUnion( buffer, u, offset, true );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a union compound geometry for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param u The union to be encoded.
 * \param offset The positional offset of the union.
 * \param superordinate \a true if the union is not contained in another body, \a false if it is.
 * \return void
 * \exception std::runtime_error Unknown geometry type.
 *
 * This function encodes the given union for the send operation to a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeUnion( SB& buffer, ConstUnionID u, const Vec3& offset, bool superordinate )
{
   // Preparing the axis-aligned bounding box
   const Union::AABB& tmp( u->getAABB() );
   Vec6 aabb( tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5] );

   // Encoding the union
   buffer << static_cast<byte>( unionTag );  // Encoding the message tag of the union
   buffer << u->getSystemID();               // Encoding the unique system-specific ID
   buffer << u->getID();                     // Encoding the user-specific ID
   buffer << u->isVisible();                 // Encoding the visibility flag
   buffer << u->isFixed();                   // Encoding the fixation flag
   marshal( buffer, u->getPosition() + offset );      // Encoding the global position

   if( !superordinate ) {
      marshal( buffer, u->getRelPosition() );         // Encoding the relative position
   }

   buffer << u->getMass();                   // Encoding the total mass
   marshal( buffer, u->getBodyInertia() );            // Encoding the moment of inertia
   marshal( buffer, u->getQuaternion() );             // Encoding the orientation of the union
   marshal( buffer, aabb );                           // Encoding the axis-aligned bounding box

   if( superordinate ) {
      marshal( buffer, u->getLinearVel() );           // Encoding the linear velocity
      marshal( buffer, u->getAngularVel() );          // Encoding the angular velocity
   }

   marshal( buffer, u->size() );                      // Encoding the number of contained bodies

   // Encoding the contained primitives
   const Union::ConstIterator begin( u->begin() );
   const Union::ConstIterator end  ( u->end()   );
   for( Union::ConstIterator body=begin; body!=end; ++body ) {
      encodeBody( buffer, *body, offset, false );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Updating a local rigid body on a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param b The rigid body to be updated on a remote MPI process.
 * \param offset The positional offset of the rigid body.
 * \return void
 *
 * This function encodes the given rigid body for the send operation to a remote MPI
 * process. Note that it is invalid to send an infinite, fixed, or remote rigid body!
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeUpdate( SB& buffer, ConstBodyID b, const Vec3& offset )
{
   // Checking the finiteness, the mobility, and the locality of the rigid body
   pe_INTERNAL_ASSERT( b->isFinite() , "Invalid infinite rigid body detected" );
   pe_INTERNAL_ASSERT( !b->isFixed() , "Invalid fixed rigid body detected"    );
   pe_INTERNAL_ASSERT( !b->isRemote(), "Invalid remote rigid body detected"   );

   // Encoding the body update
   buffer << static_cast<byte>( updateTag );  // Encoding the message tag of a rigid body update
   buffer << b->getSystemID();                // Encoding the unique system-specific ID
   buffer << b->isVisible();                  // Encoding the visibility flag
   marshal( buffer, b->getPosition() + offset );       // Encoding the global position
   marshal( buffer, b->getQuaternion() );              // Encoding the orientation of the rigid body
   marshal( buffer, b->getLinearVel() );               // Encoding the linear velocity
   marshal( buffer, b->getAngularVel() );              // Encoding the angular velocity
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding an attachable for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param a The attachable to be encoded.
 * \return void
 * \exception std::runtime_error Unknown attachable type.
 *
 * This function encodes the given attachable for the send operation to a remote MPI process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeAttachable( SB& buffer, ConstAttachableID a )
{
   switch( a->getType() ) {
      case gravityType:
         encodeGravity( buffer, static_attachable_cast<const Gravity>( a ) );
         break;
      case springType:
         encodeSpring( buffer, static_attachable_cast<const Spring>( a ) );
         break;
      default:
         throw std::runtime_error( "Unknown attachable type" );
         break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a gravity force generator for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param g The gravity force generator to be encoded.
 * \return void
 *
 * This function encodes the given gravity force generator for the send operation to a remote
 * MPI process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeGravity( SB& buffer, ConstGravityID g )
{
   // Encoding the gravity force generator
   buffer << static_cast<byte>( gravityTag );  // Encoding the message tag of the gravity force generator
   buffer << g->getSystemID();                 // Encoding the unique system-specific ID
   buffer << g->getBody()->getSystemID();      // Encoding the unique system-specific ID of the attached rigid body
   marshal( buffer, g->getGravity() );                  // Encoding the exerted gravity.
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a spring force generator for the send operation to a remote MPI process.
 *
 * \param buffer The send buffer to be filled.
 * \param s The spring force generator to be encoded.
 * \return void
 *
 * This function encodes the given spring force generator for the send operation to a remote
 * MPI process.
 */
template< typename C >   // Type of the configuration
template< typename SB >  // Type of the send buffer
void MPIEncoder<C>::encodeSpring( SB& buffer, ConstSpringID s )
{
   // Encoding the spring force generator
   buffer << static_cast<byte>( springTag );  // Encoding the message tag of the spring force generator
   buffer << s->getSystemID();                // Encoding the unique system-specific ID
   buffer << s->getBody1()->getSystemID();    // Encoding the unique system-specific ID of the first attached rigid body
   buffer << s->getBody2()->getSystemID();    // Encoding the unique system-specific ID of the second attached rigid body
   marshal( buffer, s->getAnchor1BF() );               // Encoding the anchor point of the first attached rigid body
   marshal( buffer, s->getAnchor2BF() );               // Encoding the anchor point of the second attached rigid body
   buffer << s->getStiffness();               // Encoding the spring stiffness
   buffer << s->getDamping();                 // Encoding the spring damping
   buffer << s->getRestLength();              // Encoding the spring rest length
   buffer << s->isVisible();                  // Encoding the visibility flag
}
//*************************************************************************************************

} // namespace pe

#endif
