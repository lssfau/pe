//=================================================================================================
/*!
 *  \file pe/core/Marshalling.h
 *  \brief Marshalling of objects for data transmission or storage.
 *
 *  Copyright (C) 2011 Tobias Preclik
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

#ifndef _PE_CORE_MARSHALLING_H_
#define _PE_CORE_MARSHALLING_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/core/attachable/AttachableCast.h>
#include <pe/core/rigidbody/BodyCast.h>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/rigidbody/GeomPrimitive.h>
#include <pe/core/attachable/Gravity.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/response/Types.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/attachable/Spring.h>
#include <pe/core/rigidbody/Union.h>
#include <pe/math/Vector3.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/Builtin.h>


namespace pe {


/*!\defgroup marshalling Object marshalling
 * \ingroup core
 * @{
 */


//*************************************************************************************************
template< typename Buffer, typename T >
inline void marshal( Buffer& buffer, const T& obj ) {
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( T );
   buffer << obj;
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void unmarshal( Buffer& buffer, T& obj ) {
   pe_CONSTRAINT_MUST_BE_BUILTIN_TYPE( T );
   buffer >> obj;
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void marshal( Buffer& buffer, const bool& val ) {
   buffer << static_cast<byte>( val );
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void unmarshal( Buffer& buffer, bool& val ) {
   byte tmp;
   buffer >> tmp;
   val = static_cast<bool>( tmp );
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void marshal( Buffer& buffer, const Vector3<T>& obj ) {
   buffer << obj[0] << obj[1] << obj[2];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void unmarshal( Buffer& buffer, Vector3<T>& obj ) {
   buffer >> obj[0] >> obj[1] >> obj[2];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void marshal( Buffer& buffer, const Vector6<T>& obj ) {
   buffer << obj[0] << obj[1] << obj[2] << obj[3] << obj[4] << obj[5];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void unmarshal( Buffer& buffer, Vector6<T>& obj ) {
   buffer >> obj[0] >> obj[1] >> obj[2] >> obj[3] >> obj[4] >> obj[5];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void marshal( Buffer& buffer, const Matrix3x3<T>& obj ) {
   buffer << obj[0] << obj[1] << obj[2] << obj[3] << obj[4] << obj[5] << obj[6] << obj[7] << obj[8];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void unmarshal( Buffer& buffer, Matrix3x3<T>& obj ) {
   buffer >> obj[0] >> obj[1] >> obj[2] >> obj[3] >> obj[4] >> obj[5] >> obj[6] >> obj[7] >> obj[8];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void marshal( Buffer& buffer, const Quaternion<T>& obj ) {
   buffer << obj[0] << obj[1] << obj[2] << obj[3];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer, typename T >
inline void unmarshal( Buffer& buffer, Quaternion<T>& obj ) {
   T r = T(), i = T(), j = T(), k = T();
   buffer >> r >> i >> j >> k;
   obj.set(r, i, j, k);
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void marshal( Buffer& buffer, const Twist& obj ) {
   buffer << obj[0] << obj[1] << obj[2] << obj[3] << obj[4] << obj[5];
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void unmarshal( Buffer& buffer, Twist& obj ) {
   buffer >> obj[0] >> obj[1] >> obj[2] >> obj[3] >> obj[4] >> obj[5];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling rigid body parameters.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const RigidBody& obj ) {
   buffer << obj.getSystemID();
   buffer << obj.getID();
   buffer << obj.isVisible();
   buffer << obj.isFixed();
   marshal( buffer, obj.getPosition() );
   if( obj.hasSuperBody() )
      marshal( buffer, obj.getRelPosition() );
   marshal( buffer, obj.getQuaternion() );
   if( !obj.hasSuperBody() ) {
      marshal( buffer, obj.getLinearVel() );
      marshal( buffer, obj.getAngularVel() );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling rigid body parameters dynamically.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled dynamically.
 * \return void
 *
 * The rigid body is casted dynamically to its original type and then marshalled. For recognition
 * an identifying tag is prepended.
 */
template< typename Buffer >
inline void marshalDynamically( Buffer& buffer, const RigidBody& obj ) {
   const GeomType geomType( obj.getType() );
   marshal( buffer, geomType );

   switch( geomType ) {
   case sphereType:
      marshal( buffer, static_cast<const Sphere&>( obj ) );
      break;
   case boxType:
      marshal( buffer, static_cast<const Box&>( obj ) );
      break;
   case capsuleType:
      marshal( buffer, static_cast<const Capsule&>( obj ) );
      break;
   case cylinderType:
      marshal( buffer, static_cast<const Cylinder&>( obj ) );
      break;
   case planeType:
      marshal( buffer, static_cast<const Plane&>( obj ) );
      break;
   case triangleMeshType:
      marshal( buffer, static_cast<const TriangleMesh&>( obj ) );
      break;
   case unionType:
      marshal( buffer, static_cast<const Union&>( obj ) );
      break;
   default:
      throw std::runtime_error( "Unknown geometry type" );
      break;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling rigid body parameters.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \param hasSuperBody False if body is not part of a union. Subordinate bodies in unions do not encode velocities but encode relative positions.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, RigidBody::Parameters& objparam, bool hasSuperBody ) {
   buffer >> objparam.sid_;
   buffer >> objparam.uid_;
   buffer >> objparam.visible_;
   buffer >> objparam.fixed_;
   unmarshal( buffer, objparam.gpos_ );
   /*
   std::cout << "hasSuperBody: " << hasSuperBody << std::endl;
   uint32_t tmp;
   buffer.peek( tmp );
   std::cout << "32bit peek: 0x" << std::hex << std::setw(8) << std::setfill('0') << tmp << std::dec << std::endl;
   */
   if( hasSuperBody )
      unmarshal( buffer, objparam.rpos_ );
   /*
   buffer.peek( tmp );
   std::cout << "32bit peek: 0x" << std::hex << std::setw(8) << std::setfill('0') << tmp << std::dec << std::endl;
   */
   unmarshal( buffer, objparam.q_ );
   if( !hasSuperBody ) {
      unmarshal( buffer, objparam.v_ );
      unmarshal( buffer, objparam.w_ );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling parameters of a geometric primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const GeomPrimitive& obj ) {
   marshal( buffer, static_cast<const RigidBody&>( obj ) );
   buffer << obj.getMaterial();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling parameters of a geometric primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \param hasSuperBody False if body is not part of a union. Passed on to rigid body unmarshalling.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, GeomPrimitive::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<RigidBody::Parameters&>( objparam ), hasSuperBody );
   buffer >> objparam.material_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling a box primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const Box& obj ) {
   marshal( buffer, static_cast<const GeomPrimitive&>( obj ) );
   marshal( buffer, obj.getLengths() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling a box primitive.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \param hasSuperBody False if body is not part of a union. Passed on to rigid body unmarshalling.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, Box::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<GeomPrimitive::Parameters&>( objparam ), hasSuperBody );
   unmarshal( buffer, objparam.lengths_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling a capsule primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const Capsule& obj ) {
   marshal( buffer, static_cast<const GeomPrimitive&>( obj ) );
   buffer << obj.getRadius();
   buffer << obj.getLength();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling a capsule primitive.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \param hasSuperBody False if body is not part of a union. Passed on to rigid body unmarshalling.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, Capsule::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<GeomPrimitive::Parameters&>( objparam ), hasSuperBody );
   buffer >> objparam.radius_ >> objparam.length_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling a cylinder primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const Cylinder& obj ) {
   marshal( buffer, static_cast<const GeomPrimitive&>( obj ) );
   buffer << obj.getRadius();
   buffer << obj.getLength();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling a cylinder primitive.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \param hasSuperBody False if body is not part of a union. Passed on to rigid body unmarshalling.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, Cylinder::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<GeomPrimitive::Parameters&>( objparam ), hasSuperBody );
   buffer >> objparam.radius_ >> objparam.length_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling a plane primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const Plane& obj ) {
   marshal( buffer, static_cast<const GeomPrimitive&>( obj ) );

   // Normal and displacement are reconstructed from rotation quaternion.
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling a plane primitive.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \param hasSuperBody False if body is not part of a union. Passed on to rigid body unmarshalling.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, Plane::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<GeomPrimitive::Parameters&>( objparam ), hasSuperBody );

   // Normal and displacement are reconstructed from rotation quaternion.
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling a triangle mesh primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const TriangleMesh& obj ) {
   marshal( buffer, static_cast<const GeomPrimitive&>( obj ) );
   buffer << obj.getBFVertices().size();
   for( size_t i = 0; i < obj.getBFVertices().size(); ++i )
      marshal( buffer, obj.getBFVertices()[i] );
   buffer << obj.getFaceIndices().size();
   for( size_t i = 0; i < obj.getFaceIndices().size(); ++i )
      marshal( buffer, obj.getFaceIndices()[i] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling a triangle mesh primitive.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \param hasSuperBody False if body is not part of a union. Passed on to rigid body unmarshalling.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, TriangleMesh::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<GeomPrimitive::Parameters&>( objparam ), hasSuperBody );

   size_t numVertices;
   buffer >> numVertices;
   objparam.vertices_.resize( numVertices );

   for( size_t i = 0; i < objparam.vertices_.size(); ++i )
      unmarshal( buffer, objparam.vertices_[i] );

   size_t numFaces;
   buffer >> numFaces;
   objparam.faceIndices_.resize( numFaces );

   for( size_t i = 0; i < objparam.faceIndices_.size(); ++i )
      unmarshal( buffer, objparam.faceIndices_[i] );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Marshalling a sphere primitive.
 *
 * \param buffer The buffer to be filled.
 * \param obj The object to be marshalled.
 * \return void
 */
template< typename Buffer >
inline void marshal( Buffer& buffer, const Sphere& obj ) {
   marshal( buffer, static_cast<const GeomPrimitive&>( obj ) );
   buffer << obj.getRadius();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Unmarshalling a sphere primitive.
 *
 * \param buffer The buffer from where to read.
 * \param objparam The object to be reconstructed.
 * \param hasSuperBody False if body is not part of a union. Passed on to rigid body unmarshalling.
 * \return void
 */
template< typename Buffer >
inline void unmarshal( Buffer& buffer, Sphere::Parameters& objparam, bool hasSuperBody ) {
   unmarshal( buffer, static_cast<GeomPrimitive::Parameters&>( objparam ), hasSuperBody );
   buffer >> objparam.radius_;
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void marshal( Buffer& buffer, const Union& obj ) {
   // Material of union is not relevant for reconstruction thus marshal RigidBody instead of GeomPrimitive.
   marshal( buffer, static_cast<const RigidBody&>( obj ) );

   buffer << obj.getMass();               // Encoding the total mass
   marshal( buffer, obj.getBodyInertia() );        // Encoding the moment of inertia

   // Preparing the axis-aligned bounding box
   const Union::AABB& tmp( obj.getAABB() );
   Vec6 aabb( tmp[0], tmp[1], tmp[2], tmp[3], tmp[4], tmp[5] );
   marshal( buffer, aabb );                        // Encoding the axis-aligned bounding box

   buffer << obj.size();                  // Encoding the number of contained bodies

   // Encoding the contained primitives
   const Union::ConstIterator begin( obj.begin() );
   const Union::ConstIterator end  ( obj.end()   );
   for( Union::ConstIterator body=begin; body!=end; ++body ) {
      GeomType type = (*body)->getType();
      marshal( buffer, type );
      switch( type ) {
         case sphereType:
            marshal( buffer, static_cast<const Sphere&>( *( *body ) ) );
            break;
         case boxType:
            marshal( buffer, static_cast<const Box&>( *( *body ) ) );
            break;
         case capsuleType:
            marshal( buffer, static_cast<const Capsule&>( *( *body ) ) );
            break;
         case cylinderType:
            marshal( buffer, static_cast<const Cylinder&>( *( *body ) ) );
            break;
         case planeType:
            marshal( buffer, static_cast<const Plane&>( *( *body ) ) );
            break;
         case unionType:
            marshal( buffer, static_cast<const Union&>( *( *body ) ) );
            break;
         default:
            throw std::runtime_error( "Unknown geometry type" );
            break;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void unmarshal( Buffer& buffer, Union::Parameters& objparam, bool hasSuperBody ) {
   // Material of union is not relevant for reconstruction thus marshal RigidBody instead of GeomPrimitive.
   unmarshal( buffer, static_cast<RigidBody::Parameters&>( objparam ), hasSuperBody );

   /*
   uint32_t tmp;
   buffer.peek( tmp );
   std::cout << "32bit peek: 0x" << std::hex << tmp << std::dec << std::endl;
   */
   buffer >> objparam.m_;
   unmarshal( buffer, objparam.I_ );
   unmarshal( buffer, objparam.aabb_ );
   //std::cout << "mass of union: " << objparam.m_ << std::endl;

   // Decode the number of contained bodies
   Union::SizeType size;
   buffer >> size;
   //std::cout << "size of union: " << size << std::endl;

   // Decoding the contained primitives
   for( Union::SizeType i = 0; i < size; ++i ) {
      GeomType type;
      unmarshal( buffer, type );
      //std::cout << "union subbody geomType: " << (int)type << std::endl;
      switch( type ) {
         case sphereType: {
            Sphere::Parameters subobjparam;
            unmarshal( buffer, subobjparam, true );
            objparam.spheres_.push_back( subobjparam );
            break;
         }
         case boxType: {
            Box::Parameters subobjparam;
            unmarshal( buffer, subobjparam, true );
            objparam.boxes_.push_back( subobjparam );
            break;
         }
         case capsuleType: {
            Capsule::Parameters subobjparam;
            unmarshal( buffer, subobjparam, true );
            objparam.capsules_.push_back( subobjparam );
            break;
         }
         case cylinderType: {
            Cylinder::Parameters subobjparam;
            unmarshal( buffer, subobjparam, true );
            objparam.cylinders_.push_back( subobjparam );
            break;
         }
         case planeType: {
            Plane::Parameters subobjparam;
            unmarshal( buffer, subobjparam, true );
            objparam.planes_.push_back( subobjparam );
            break;
         }
         case unionType: {
            Union::Parameters subobjparam;
            unmarshal( buffer, subobjparam, true );
            objparam.unions_.push_back( subobjparam );
            break;
         }
         default:
            std::cout << "failed union subbody geomType: " << (int)type << std::endl;
            throw std::runtime_error( "Unknown geometry type" );
            break;
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void marshal( Buffer& buffer, const Attachable& obj ) {
   buffer << obj.getSystemID();               // Encoding the unique system-specific ID
   buffer << obj.size();                      // Encoding the number of attached bodies
   for( Attachable::Iterator it = obj.begin(); it != obj.end(); ++it )
      buffer << (*it)->getSystemID();         // Encoding the unique system-specific ID of the attached rigid bodies
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void marshal( Buffer& buffer, const Gravity& obj ) {
   marshal( buffer, static_cast<const Attachable&>( obj ) );
   buffer << obj.getGravity();                 // Encoding the exerted gravity.
}
//*************************************************************************************************


//*************************************************************************************************
template< typename Buffer >
inline void marshal( Buffer& buffer, const Spring& obj ) {
   marshal( buffer, static_cast<const Attachable&>( obj ) );
   buffer << obj.getAnchor1BF();              // Encoding the anchor point of the first attached rigid body
   buffer << obj.getAnchor2BF();              // Encoding the anchor point of the second attached rigid body
   buffer << obj.getStiffness();              // Encoding the spring stiffness
   buffer << obj.getDamping();                // Encoding the spring damping
   buffer << obj.getRestLength();             // Encoding the spring rest length
   buffer << obj.isVisible();                 // Encoding the visibility flag
}
//*************************************************************************************************

//!@}

} // namespace pe

#endif
