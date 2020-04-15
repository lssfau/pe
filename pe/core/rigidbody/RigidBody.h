//=================================================================================================
/*!
 *  \file pe/core/rigidbody/RigidBody.h
 *  \brief Header file for the RigidBody class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2013-2014 Tobias Scharpff
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

#ifndef _PE_CORE_RIGIDBODY_RIGIDBODY_H_
#define _PE_CORE_RIGIDBODY_RIGIDBODY_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <iosfwd>
#include <pe/core/attachable/Attachable.h>
#include <pe/core/Configuration.h>
#include <pe/core/GamesSection.h>
#include <pe/core/GeomType.h>
#include <pe/core/ScientificSection.h>
#include <pe/core/Types.h>
#include <pe/core/UniqueID.h>
#include <pe/math/Matrix3x3.h>
#include <pe/math/Quaternion.h>
#include <pe/core/rigidbody/RigidBodyTrait.h>
#include <pe/math/RotationMatrix.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/policies/NoDelete.h>
#include <pe/util/PtrVector.h>
#include <pe/util/StaticAssert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  ::pe NAMESPACE FORWARD DECLARATIONS
//
//=================================================================================================

template< typename C > class CollisionSystem;




//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup rigid_body Rigid bodies
 * \ingroup core
 */
/*!\brief Base class for all rigid bodies.
 * \ingroup rigid_body
 *
 * The RigidBody class is the abstract basis for all rigid bodies of the physics engine. The
 * class provides the common data members and the common functionality for all rigid bodies.
 */
class RigidBody : public RigidBodyTrait<Config>
{
private:
   //**Type definitions****************************************************************************
   typedef pe::RigidBodyTrait<Config>      Parent;       //!< The type of the parent class of the RigidBody class.
   typedef PtrVector<RigidBody,NoDelete>   Bodies;       //!< Vector for superordinate bodies containing this rigid body.
   typedef PtrVector<Contact,NoDelete>     Contacts;     //!< Vector for attached contacts.
   typedef PtrVector<Attachable,NoDelete>  Attachables;  //!< Vector for attachables.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef Contacts::Iterator          ContactIterator;            //!< Iterator over the currently attached contacts.
   typedef Contacts::ConstIterator     ConstContactIterator;       //!< ConstIterator over the currently attached contacts.
   typedef Attachables::Iterator       AttachableIterator;         //!< Iterator over the currently attached attachables.
   typedef Attachables::ConstIterator  ConstAttachableIterator;    //!< ConstIterator over the currently attached attachables.
   typedef Bodies::Iterator            AttachedBodyIterator;       //!< Iterator over the currently attached rigid bodies.
   typedef Bodies::ConstIterator       ConstAttachedBodyIterator;  //!< ConstIterator over the currently attached rigid bodies.
   typedef CCDBT::AABB                 AABB;                       //!< Bounding volume of the rigid body.
   struct Parameters {
      bool visible_, fixed_;
      id_t sid_, uid_;
      Vec3 gpos_, rpos_, v_, w_;
      Quat q_;
   };
   //**********************************************************************************************

protected:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit RigidBody( GeomType type, bool finite, bool visible, id_t sid, id_t uid );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~RigidBody() = 0;
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline bool           hasManager()        const;
   inline ManagerID      getManager();
   inline ConstManagerID getManager()        const;
   inline bool           hasSuperBody()      const;
   inline BodyID         getSuperBody();
   inline ConstBodyID    getSuperBody()      const;
   inline BodyID         getTopSuperBody();
   inline ConstBodyID    getTopSuperBody()   const;
   inline bool           isFinite()          const;
   inline bool           isActive()          const;
   inline bool           isAwake()           const;
   inline bool           isFixed()           const;
   inline bool           isVisible()         const;
   inline GeomType       getType()           const;
   inline id_t           getSystemID()       const;
   inline id_t           getID()             const;
   inline const Vec3&    getRelPosition()    const;
   inline const Vec3&    getPosition()       const;
   inline const Vec3     getRelLinearVel()   const;
   inline const Vec3&    getLinearVel()      const;
   inline const Vec3     getRelAngularVel()  const;
   inline const Vec3&    getAngularVel()     const;
   inline const Quat&    getQuaternion()     const;
   inline const Rot3&    getRotation()       const;
   inline real           getMass()           const;
   inline real           getInvMass()        const;
   inline const Mat3&    getBodyInertia()    const;
   inline const Mat3     getInertia()        const;
   inline const Mat3&    getInvBodyInertia() const;
   inline const Mat3     getInvInertia()     const;
   inline const AABB&    getAABB()           const;

   inline const Vec3     vectorFromBFtoWF( real vx, real vy, real vz ) const;
   inline const Vec3     vectorFromBFtoWF( const Vec3& v )             const;
   inline const Vec3     pointFromBFtoWF ( real px, real py, real pz ) const;
   inline const Vec3     pointFromBFtoWF ( const Vec3& rpos )          const;
   inline const Vec3     velFromBF       ( real px, real py, real pz ) const;
   inline const Vec3     velFromBF       ( const Vec3& rpos )          const;
   inline const Vec3     vectorFromWFtoBF( real vx, real vy, real vz ) const;
   inline const Vec3     vectorFromWFtoBF( const Vec3& v )             const;
   inline const Vec3     pointFromWFtoBF ( real px, real py, real pz ) const;
   inline const Vec3     pointFromWFtoBF ( const Vec3& gpos )          const;
   inline const Vec3     velFromWF       ( real px, real py, real pz ) const;
   inline const Vec3     velFromWF       ( const Vec3& gpos )          const;
   inline const Vec3     accFromWF       ( real px, real py, real pz ) const;
          const Vec3     accFromWF       ( const Vec3& gpos )          const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
           void setFixed      ( bool fixed );
   virtual void setVisible    ( bool visible ) = 0;
   virtual void setPosition   ( real px, real py, real pz ) = 0;
   virtual void setPosition   ( const Vec3& gpos ) = 0;
   virtual void setOrientation( real r, real i, real j, real k ) = 0;
   virtual void setOrientation( const Quat& q ) = 0;

   inline void setRelLinearVel ( real vx, real vy, real vz );
   inline void setRelLinearVel ( const Vec3& lvel );
   inline void setLinearVel    ( real vx, real vy, real vz );
   inline void setLinearVel    ( const Vec3& lvel );
   inline void setRelAngularVel( real ax, real ay, real az );
   inline void setRelAngularVel( const Vec3& avel );
   inline void setAngularVel   ( real ax, real ay, real az );
   inline void setAngularVel   ( const Vec3& avel );
   //@}
   //**********************************************************************************************

   //**Fixation functions**************************************************************************
   /*!\name Fixation functions */
   //@{
   virtual bool isAlwaysFixed() const;
   //@}
   //**********************************************************************************************

   //**Translation functions***********************************************************************
   /*!\name Translation functions */
   //@{
   virtual void translate( real dx, real dy, real dz ) = 0;
   virtual void translate( const Vec3& dp ) = 0;
   //@}
   //**********************************************************************************************

   //**Rotation functions**************************************************************************
   /*!\name Rotation functions */
   //@{
   virtual void rotate( real x, real y, real z, real angle ) = 0;
   virtual void rotate( const Vec3& axis, real angle ) = 0;
   virtual void rotate( real xangle, real yangle, real zangle ) = 0;
   virtual void rotate( const Vec3& euler ) = 0;
   virtual void rotate( const Quat& dq ) = 0;

   virtual void rotateAroundOrigin( real x, real y, real z, real angle ) = 0;
   virtual void rotateAroundOrigin( const Vec3& axis, real angle ) = 0;
   virtual void rotateAroundOrigin( real xangle, real yangle, real zangle ) = 0;
   virtual void rotateAroundOrigin( const Vec3& euler ) = 0;

   virtual void rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle ) = 0;
   virtual void rotateAroundPoint( const Vec3& point, const Vec3& euler ) = 0;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool containsRelPoint       ( real px, real py, real pz ) const = 0;
   virtual bool containsRelPoint       ( const Vec3& rpos )          const = 0;
   virtual bool containsPoint          ( real px, real py, real pz ) const = 0;
   virtual bool containsPoint          ( const Vec3& gpos )          const = 0;
   virtual bool isSurfaceRelPoint      ( real px, real py, real pz ) const = 0;
   virtual bool isSurfaceRelPoint      ( const Vec3& rpos )          const = 0;
   virtual bool isSurfacePoint         ( real px, real py, real pz ) const = 0;
   virtual bool isSurfacePoint         ( const Vec3& gpos )          const = 0;
   virtual Vec3 support                ( const Vec3& d )             const;
   virtual Vec3 supportContactThreshold( const Vec3& d )             const;
   //@}
   //**********************************************************************************************

   //**Force functions*****************************************************************************
   /*!\name Force functions */
   //@{
           inline bool        hasForce () const;

           inline const Vec3& getForce () const;
           inline const Vec3& getTorque() const;

           inline void        setForce           ( const Vec3& f );
           inline void        setTorque          ( const Vec3& t );

           inline void        addRelForce        ( real fx, real fy, real fz );
           inline void        addRelForce        ( const Vec3& f );
           inline void        addForce           ( real fx, real fy, real fz );
           inline void        addForce           ( const Vec3& f );
           inline void        addRelForceAtRelPos( real fx, real fy, real fz, real px, real py, real pz );
           inline void        addRelForceAtRelPos( const Vec3& f, const Vec3& rpos );
           inline void        addRelForceAtPos   ( real fx, real fy, real fz, real px, real py, real pz );
           inline void        addRelForceAtPos   ( const Vec3& f, const Vec3& gpos );
           inline void        addForceAtRelPos   ( real fx, real fy, real fz, real px, real py, real pz );
           inline void        addForceAtRelPos   ( const Vec3& f, const Vec3& rpos );
           inline void        addForceAtPos      ( real fx, real fy, real fz, real px, real py, real pz );
           inline void        addForceAtPos      ( const Vec3& f, const Vec3& gpos );

           inline void        addTorque( real tx, real ty, real tz );
           inline void        addTorque( const Vec3& t );

   virtual inline void        resetForce();
   //@}
   //**********************************************************************************************

   //**Impulse functions***************************************************************************
   /*!\name Impulse functions */
   //@{
   inline void addImpulse( real jx, real jy, real jz );
   inline void addImpulse( const Vec3& j );
   inline void addImpulseAtPos( real jx, real jy, real jz, real px, real py, real pz );
   inline void addImpulseAtPos( const Vec3& j, const Vec3& p );
   //@}
   //**********************************************************************************************

   //**Contact functions***************************************************************************
   /*!\name Contact functions */
   //@{
   inline void                 registerContact( ContactID contact );
   inline bool                 hasContacts  () const;
   inline void                 clearContacts();
   inline size_t               countContacts() const;
   inline ContactIterator      beginContacts();
   inline ConstContactIterator beginContacts() const;
   inline ContactIterator      endContacts  ();
   inline ConstContactIterator endContacts  () const;
   //@}
   //**********************************************************************************************

   //**Attachable functions************************************************************************
   /*!\name Attachable functions */
   //@{
          void                      registerAttachable  ( AttachableID attachable );
          void                      deregisterAttachable( AttachableID attachable );
   inline bool                      isAttached          () const;
   inline size_t                    countAttachables    () const;
   inline AttachableIterator        beginAttachables    ();
   inline ConstAttachableIterator   beginAttachables    () const;
   inline AttachableIterator        endAttachables      ();
   inline ConstAttachableIterator   endAttachables      () const;
   inline AttachedBodyIterator      beginAttachedBodies ();
   inline ConstAttachedBodyIterator beginAttachedBodies () const;
   inline AttachedBodyIterator      endAttachedBodies   ();
   inline ConstAttachedBodyIterator endAttachedBodies   () const;
   //@}
   //**********************************************************************************************

   //**MPI functions*******************************************************************************
   /*!\name MPI functions */
   //@{
           inline bool isRemote () const;
   virtual inline void setRemote( bool remote );
           inline bool isGlobal () const;
           inline void setGlobal();
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os, const char* tab ) const = 0;
   //@}
   //**********************************************************************************************

protected:
   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void calcRelPosition();
   //@}
   //**********************************************************************************************

   //**Utility functions*******************************************************************************
   /*!\name Utility functions */
   //@{
   virtual void calcBoundingBox();
   //@}
   //**********************************************************************************************

   //**Fixation functions**************************************************************************
   /*!\name Fixation functions */
   //@{
   virtual void fix  ();
   virtual void unfix();
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void update( const Vec3& dp ) = 0;  // Translation update of a subordinate rigid body
   virtual void update( const Quat& dq ) = 0;  // Rotation update of a subordinate rigid body
   //@}
   //**********************************************************************************************

   //**Functions for internal changes in compound geometries***************************************
   /*!\name Functions for internal changes in compound geometries */
   //@{
   inline void signalModification();
   inline void signalTranslation();
   inline void signalRotation();
   inline void signalFixation();

   virtual void handleModification();
   virtual void handleTranslation();
   virtual void handleRotation();
   virtual void handleFixation();
   //@}
   //**********************************************************************************************

   //**Debugging functions*************************************************************************
   /*!\name Debugging functions */
   //@{
   virtual bool checkInvariants();
   //@}
   //**********************************************************************************************

protected:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   ManagerID manager_;        //!< The rigid body manager responsible for the rigid body.
   BodyID sb_;                //!< The superordinate rigid body.
                              /*!< This data member is the connection to the superordinate body,
                                   which is either the enclosing Union or the rigid body itself. */
   const GeomType type_;      //!< The geometry type of the rigid body.
   bool finite_;              //!< Finiteness flag.
                              /*!< The flag value indicates if the rigid body is finite (\a true)
                                   or infinite (\a false). */
   bool visible_;             //!< Visibility flag.
                              /*!< If the body is visible, it is automatically registered for all
                                   active visualizations. */
   bool remote_;              //!< Remote flag.
                              /*!< This flag indicates whether the rigid body belongs to a remote
                                   process (\a true) or to the local process (\a false). */
   bool global_;              //!< Global flag.
                              /*!< The global flag indicates whether the rigid body is global in
                                   an MPI parallel simulation. Since global rigid bodies are always
                                   fixed and known on all MPI processes they are not participating
                                   in any communication process. */
   id_t sid_;                 //!< The unique system-specific body ID.
   id_t uid_;                 //!< The user-specific body ID.
   Contacts contacts_;        //!< Vector for the currently attached contacts.
   Attachables attachables_;  //!< Vector for the currently attached attachables.
   Bodies attachedBodies_;    //!< Vector for the currently attached rigid bodies.
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Bodies superBodies_;   //!< All superordinate bodies containing this rigid body.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class BodyManager;
   friend class SuperBody;
   template< typename C > friend class CollisionSystem;
   /*! \endcond */
   //**********************************************************************************************

   //**Rigid body setup functions******************************************************************
   /*! \cond PE_INTERNAL */
   friend void destroy( BodyID body );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the rigid body currently has a supervising rigid body manager.
 *
 * \return \a true if the rigid body has a supervising manager, \a false if not.
 */
inline bool RigidBody::hasManager() const
{
   return manager_ != 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the supervising rigid body manager of the rigid body.
 *
 * \return The supervising rigid body manager.
 *
 * If the body is currently not supervised by an rigid body manager, the returned ManagerID is 0.
 */
inline ManagerID RigidBody::getManager()
{
   return manager_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the supervising rigid body manager of the rigid body.
 *
 * \return The supervising rigid body manager.
 *
 * If the body is currently not supervised by an rigid body manager, the returned ManagerID is 0.
 */
inline ConstManagerID RigidBody::getManager() const
{
   return manager_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body is contained in a superordinate body.
 *
 * \return \a true if the rigid body is contained in a superordinate body, \a false if not.
 */
inline bool RigidBody::hasSuperBody() const
{
   return sb_ != this;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the superordinate body in which the rigid body is contained.
 *
 * \return The superordinate body.
 *
 * This function returns the direct superordinate body in which the rigid body is contained.
 * If the rigid body is not contained in another body, the returned BodyID is the body itself.
 */
inline BodyID RigidBody::getSuperBody()
{
   return sb_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the superordinate body in which the rigid body is contained.
 *
 * \return The superordinate body.
 *
 * This function returns the direct superordinate body in which the rigid body is contained.
 * If the rigid body is not contained in another body, the returned BodyID is the body itself.
 */
inline ConstBodyID RigidBody::getSuperBody() const
{
   return sb_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the top level superordinate body in which the rigid body is contained.
 *
 * \return The top level superordinate body.
 *
 * This function returns the top level superordinate body in which the rigid body is contained.
 * If the rigid body is not contained in another body, the returned BodyID is the body itself.
 */
inline BodyID RigidBody::getTopSuperBody()
{
   if( !hasSuperBody() )
      return sb_;
   else return sb_->getTopSuperBody();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the top level superordinate body in which the rigid body is contained.
 *
 * \return The superordinate body.
 *
 * This function returns the top level superordinate body in which the rigid body is contained.
 * If the rigid body is not contained in another body, the returned BodyID is the body itself.
 */
inline ConstBodyID RigidBody::getTopSuperBody() const
{
   if( !hasSuperBody() )
      return sb_;
   else return sb_->getTopSuperBody();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body is finite or not.
 *
 * \return \a true in case of a finite rigid body, \a false in case of a infinite rigid body.
 */
inline bool RigidBody::isFinite() const
{
   return finite_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body is currently active or not.
 *
 * \return \a true in case the rigid body is active, \a false if it is either asleep or fixed.
 */
inline bool RigidBody::isActive() const
{
   if( !hasSuperBody() )
#if MOBILE_INFINITE
      return awake_;
#else
      return awake_ && !fixed_;
#endif
   else return sb_->isActive();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body is awake or not.
 *
 * \return \a true in case the rigid body is awake, \a false if is not.
 */
inline bool RigidBody::isAwake() const
{
   if( !hasSuperBody() )
      return awake_;
   else return sb_->isAwake();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body's position is fixed or not.
 *
 * \return \a true in case of a fixed/stationary rigid body, \a false in case of a free body.
 */
inline bool RigidBody::isFixed() const
{
   return fixed_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body is visible or not.
 *
 * \return \a true in case of a visible rigid body, \a false in case of an invisible body.
 */
inline bool RigidBody::isVisible() const
{
   return visible_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the geometry type of the rigid body.
 *
 * \return The geometry type of the rigid body.
 */
inline GeomType RigidBody::getType() const
{
   return type_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the unique system-specific ID of the rigid body.
 *
 * \return The system-specific ID.
 */
inline id_t RigidBody::getSystemID() const
{
   return sid_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the user-specific ID of the rigid body.
 *
 * \return The user-specific ID.
 */
inline id_t RigidBody::getID() const
{
   return uid_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the relative position of the rigid body within the superordinate body.
 *
 * \return The relative position of the rigid body.
 *
 * If the rigid body is not contained in a superordinate body, the returned relative position will
 * be \f$ \left(\begin{array}{*{3}{c}} 0 & 0 & 0 \end{array}\right) \f$.
 */
inline const Vec3& RigidBody::getRelPosition() const
{
   return rpos_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the global position of the center of mass of the rigid body.
 *
 * \return The global position of the center of mass.
 */
inline const Vec3& RigidBody::getPosition() const
{
   return gpos_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the relative linear velocity of the rigid body.
 *
 * \return The relative linear velocity.
 *
 * This function returns the linear velocity of the center of mass of the rigid body in reference
 * to the body's own frame of reference.
 */
inline const Vec3 RigidBody::getRelLinearVel() const
{
   if( hasSuperBody() )
      v_ = sb_->velFromWF( gpos_ );
   return trans(R_) * v_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the global linear velocity of the rigid body.
 *
 * \return The global linear velocity.
 *
 * This function returns the linear velocity of the center of mass of the rigid body in reference
 * to the global world frame.
 */
inline const Vec3& RigidBody::getLinearVel() const
{
   if( hasSuperBody() )
      v_ = sb_->velFromWF( gpos_ );
   return v_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the relative angular velocity.
 *
 * \return The relative angular velocity.
 *
 * This function returns the angluar velocity of the center of mass in reference to the body's
 * own frame of reference.
 */
inline const Vec3 RigidBody::getRelAngularVel() const
{
   if( hasSuperBody() )
      w_ = sb_->getAngularVel();
   return trans(R_) * w_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the global angular velocity.
 *
 * \return The global angular velocity.
 *
 * This function returns the angluar velocity of the center of mass in reference to the global
 * world frame.
 */
inline const Vec3& RigidBody::getAngularVel() const
{
   if( hasSuperBody() )
      w_ = sb_->getAngularVel();
   return w_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the orientation of the rigid body.
 *
 * \return The orientation of the rigid body.
 *
 * This function returns the quaternion of the rigid body, which represents the orientation of
 * the body in reference to the global world frame.
 */
inline const Quat& RigidBody::getQuaternion() const
{
   return q_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the rotation of the rigid body.
 *
 * \return The rotation of the rigid body.
 *
 * This function returns the rotation matrix of the rigid body, which represents the rotation of
 * the body in reference to the global world frame.
 */
inline const Rot3& RigidBody::getRotation() const
{
   return R_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the total mass of the rigid body.
 *
 * \return The total mass of the rigid body.
 */
inline real RigidBody::getMass() const
{
   return mass_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the inverse total mass of the rigid body.
 *
 * \return The inverse total mass of the rigid body.
 */
inline real RigidBody::getInvMass() const
{
   return invMass_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the moment of inertia in reference to the body frame of the rigid body.
 *
 * \return The body relative moment of inertia.
 */
inline const Mat3& RigidBody::getBodyInertia() const
{
   return I_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the moment of inertia in reference to the global world frame.
 *
 * \return The global moment of inertia.
 */
inline const Mat3 RigidBody::getInertia() const
{
   return R_ * I_ * trans(R_);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the inverse moment of inertia in reference to the body frame of the rigid body.
 *
 * \return The inverse body relative moment of inertia.
 */
inline const Mat3& RigidBody::getInvBodyInertia() const
{
   return Iinv_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the inverse moment of inertia in reference to the global world frame.
 *
 * \return The inverse global moment of inertia.
 */
inline const Mat3 RigidBody::getInvInertia() const
{
   return R_ * Iinv_ * trans(R_);
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the axis-aligned bounding box of the rigid body.
 *
 * \return The axis-aligned bounding box of the rigid body.
 */
inline const RigidBody::AABB& RigidBody::getAABB() const
{
   return aabb_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a relative to a global vector.
 *
 * \param vx The x-component of the relative vector.
 * \param vy The y-component of the relative vector.
 * \param vz The z-component of the relative vector.
 * \return The transformed global vector.
 *
 * The function calculates the transformation of a vector in body frame to a vector in global
 * world frame.
 */
inline const Vec3 RigidBody::vectorFromBFtoWF( real vx, real vy, real vz ) const
{
   return R_ * Vec3( vx, vy, vz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a relative to a global vector.
 *
 * \param v the relative vector.
 * \return The transformed global vector.
 *
 * The function calculates the transformation of a vector in body frame to a vector in global
 * world frame.
 */
inline const Vec3 RigidBody::vectorFromBFtoWF( const Vec3& v ) const
{
   return R_ * v;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a relative to a global coordinate.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return The transformed global coordinate.
 *
 * The function calculates the transformation of a point relative to the body's center of
 * mass to a point in global coordinates.
 */
inline const Vec3 RigidBody::pointFromBFtoWF( real px, real py, real pz ) const
{
   return gpos_ + ( R_ * Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a relative to a global coordinate.
 *
 * \param rpos the relative coordinate.
 * \return The transformed global coordinate.
 *
 * The function calculates the transformation of a point relative to the body's center of
 * mass to a point in global coordinates.
 */
inline const Vec3 RigidBody::pointFromBFtoWF( const Vec3& rpos ) const
{
   return gpos_ + ( R_ * rpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the global velocity of a relative point.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return The global velocity.
 *
 * The function calculates the global velocity of a point relative to the body's center of mass.
 */
inline const Vec3 RigidBody::velFromBF( real px, real py, real pz ) const
{
   return velFromBF( Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the global velocity of a relative point.
 *
 * \param rpos The relative coordinate.
 * \return The global velocity.
 *
 * The function calculates the global velocity of a point relative to the body's center of mass.
 */
inline const Vec3 RigidBody::velFromBF( const Vec3& rpos ) const
{
   if( !hasSuperBody() )
      return v_ + w_ % ( R_ * rpos );
   else return sb_->velFromWF( gpos_ + R_ * rpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a global to a relative vector.
 *
 * \param vx The x-component of the global vector.
 * \param vy The y-component of the global vector.
 * \param vz The z-component of the global vector.
 * \return The transformed relative vector.
 *
 * The function calculates the transformation of a vector in global world frame to a vector
 * in body frame.
 */
inline const Vec3 RigidBody::vectorFromWFtoBF( real vx, real vy, real vz ) const
{
   return trans(R_) * Vec3( vx, vy, vz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a global to a relative vector.
 *
 * \param v The global vector.
 * \return The transformed relative vector.
 *
 * The function calculates the transformation of a vector in global world frame to a vector
 * in body frame.
 */
inline const Vec3 RigidBody::vectorFromWFtoBF( const Vec3& v ) const
{
   return trans(R_) * v;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a global to a relative coordinate.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return The transformed relative coordinate.
 *
 * The function calculates the transformation of a point in global coordinates to a point
 * relative to the body's center of mass.
 */
inline const Vec3 RigidBody::pointFromWFtoBF( real px, real py, real pz ) const
{
   return trans(R_) * ( Vec3( px, py, pz ) - gpos_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Transformation from a global to a relative coordinate.
 *
 * \param gpos The global coordinate.
 * \return The transformed relative coordinate.
 *
 * The function calculates the transformation of a point in global coordinates to a point
 * relative to the body's center of mass.
 */
inline const Vec3 RigidBody::pointFromWFtoBF( const Vec3& gpos ) const
{
   return trans(R_) * ( gpos - gpos_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the global velocity of a point in global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return The global velocity.
 *
 * The function calculates the global velocity of a point in global coordinates.
 */
inline const Vec3 RigidBody::velFromWF( real px, real py, real pz ) const
{
   return velFromWF( Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the global velocity of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return The global velocity.
 *
 * The function calculates the global velocity of a point in global coordinates.
 */
inline const Vec3 RigidBody::velFromWF( const Vec3& gpos ) const
{
   if( !hasSuperBody() )
      return v_ + w_ % ( gpos - gpos_ );
   else return sb_->velFromWF( gpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the global acceleration of a point in global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return The global acceleration.
 *
 * The function calculates the global acceleration of a point in global coordinates.
 */
const Vec3 RigidBody::accFromWF( real px, real py, real pz ) const
{
   return accFromWF( Vec3( px, py, pz ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the relative linear velocity of the rigid body.
 *
 * \param vx The x-component of the relative linear velocity.
 * \param vy The y-component of the relative linear velocity.
 * \param vz The z-component of the relative linear velocity.
 * \return void
 *
 * This function sets the linear velocity of the rigid body in reference to the body's own
 * frame of reference. The given relative velocity is translated into the global world frame
 * depending on the orientation of the rigid body. If the body is contained in a superordinate body
 * the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setRelLinearVel( real vx, real vy, real vz )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      v_ = R_ * Vec3( vx, vy, vz );
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the relative linear velocity of the rigid body.
 *
 * \param lvel The relative linear velocity.
 * \return void
 *
 * This function sets the linear velocity of the rigid body in reference to the body's own
 * frame of reference. The given relative velocity is translated into the global world frame
 * depending on the orientation of the rigid body. If the body is contained in a superordinate body
 * the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setRelLinearVel( const Vec3& lvel )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      v_ = R_ * lvel;
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global linear velocity of the rigid body.
 *
 * \param vx The x-component of the global linear velocity.
 * \param vy The y-component of the global linear velocity.
 * \param vz The z-component of the global linear velocity.
 * \return void
 *
 * This function sets the linear velocity of the rigid body. If the body is contained in a
 * superordinate body the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setLinearVel( real vx, real vy, real vz )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      v_ = Vec3( vx, vy, vz );
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global linear velocity of the rigid body.
 *
 * \param lvel The global linear velocity.
 * \return void
 *
 * This function sets the linear velocity of the rigid body. If the body is contained in a
 * superordinate body the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setLinearVel( const Vec3& lvel )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      v_ = lvel;
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the relative angular velocity of the rigid body.
 *
 * \param ax The x-component of the relative angular velocity.
 * \param ay The y-component of the relative angular velocity.
 * \param az The z-component of the relative angular velocity.
 * \return void
 *
 * This function sets the angular velocity of the rigid body in reference to the body's own
 * frame of reference. The given relative velocity is translated into the global world frame
 * depending on the orientation of the rigid body. If the body is contained in a superordinate body
 * the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setRelAngularVel( real ax, real ay, real az )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      w_ = R_ * Vec3( ax, ay, az );
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the relative angular velocity of the rigid body.
 *
 * \param avel The relative angular velocity.
 * \return void
 *
 * This function sets the angular velocity of the rigid body in reference to the body's own
 * frame of reference. The given relative velocity is translated into the global world frame
 * depending on the orientation of the rigid body. If the body is contained in a superordinate body
 * the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setRelAngularVel( const Vec3& avel )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      w_ = R_ * avel;
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global angular velocity of the rigid body.
 *
 * \param ax The x-component of the global angular velocity.
 * \param ay The y-component of the global angular velocity.
 * \param az The z-component of the global angular velocity.
 * \return void
 *
 * This function sets the angular velocity of the rigid body. If the body is contained in a
 * superordinate body the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setAngularVel( real ax, real ay, real az )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      w_ = Vec3( ax, ay, az );
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global angular velocity of the rigid body.
 *
 * \param avel The global angular velocity.
 * \return void
 *
 * This function sets the angular velocity of the rigid body. If the body is contained in a
 * superordinate body the function has no effect.
 *
 * In case of a <b>MPI parallel simulation</b>, changing the settings of a (local) rigid body
 * on one process may invalidate the settings of the rigid body on another process. In order to
 * synchronize all rigid bodies after local changes, the World::synchronize() function should
 * be used to update remote rigid bodies accordingly. Note that any changes on remote rigid
 * bodies are neglected and overwritten by the settings of the rigid body on its local process!
 */
inline void RigidBody::setAngularVel( const Vec3& avel )
{
#if MOBILE_INFINITE
   if( !hasSuperBody() ) {
#else
   if( !hasSuperBody() && !fixed_ ) {
#endif
      w_ = avel;
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculation of the relative position within a superordinate body.
 *
 * \return void
 *
 * The function calculates the relative position depending on its current global position, the
 * current global position of the superordinate body and the rotation of the superordinate body.
 */
inline void RigidBody::calcRelPosition()
{
   rpos_ = trans(sb_->R_)*( gpos_ - sb_->gpos_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  FORCE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the rigid body has non-zero acting forces or torques.
 *
 * \return \a true if the currently acting force and/or torque is non-zero, \a false if not.
 */
inline bool RigidBody::hasForce() const
{
   return ( force_ != real(0) || torque_ != real(0) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current force acting on the body's center of mass.
 *
 * \return The acting force.
 *
 * This function returns the current force acting on the rigid body. If the body is contained
 * in a union, then this force is part of the total force acting on the union.\n
 * If the pe::pe_GAMES_MODE is active and the body is part of a union, the returned force is
 * always (0,0,0).
 */
inline const Vec3& RigidBody::getForce() const
{
   return force_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current torque acting on the body's center of mass.
 *
 * \return The acting torque.
 *
 * This function returns the current torque acting in the center of mass of the rigid body. If
 * the body is contained in a union, then this torque represents the part of the total torque
 * acting on the union that results from the forces on this body.\n
 * If the pe::pe_GAMES_MODE is active and the body is part of a union, the returned torque is
 * always (0,0,0).
 */
inline const Vec3& RigidBody::getTorque() const
{
   return torque_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Set the force acting at the body's center of mass.
 *
 * \param f The acting force.
 * \return void
 */
inline void RigidBody::setForce( const Vec3& f )
{
   // Increasing the force on this rigid body
   force_ = f;

   if( superBodies_.isEmpty() ) {
      // Waking this rigid body
      pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superbody detected" );
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Set the torque acting at the body's center of mass.
 *
 * \param tau The acting torque.
 * \return void
 */
inline void RigidBody::setTorque( const Vec3& tau )
{
   // Increasing the force on this rigid body
   torque_ = tau;

   if( superBodies_.isEmpty() ) {
      // Waking this rigid body
      pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superbody detected" );
      wake();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the total force acting in the body's center of mass.
 *
 * \param fx The x-component of the relative force.
 * \param fy The y-component of the relative force.
 * \param fz The z-component of the relative force.
 * \return void
 *
 * The function applies a body relative force to the rigid body. The given force is acting
 * directly in the body's center of mass and increases the total force acting on the body.
 * If the body is part of a superordinate body, the force is also acting on the superordinate
 * body. Depending on the position of the superordinate body's center of mass, the force can
 * also cause a torque in the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addRelForce( real fx, real fy, real fz )
{
   addForce( R_ * Vec3( fx, fy, fz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param f The acting relative force.
 * \return void
 *
 * The function applies a body relative force to the rigid body. The given force is acting
 * directly in the body's center of mass and increases the total force acting on the body.
 * If the body is part of a superordinate body, the force is also acting on the superordinate
 * body. Depending on the position of the superordinate body's center of mass, the force can
 * also cause a torque in the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addRelForce( const Vec3& f )
{
   addForce( R_ * f );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param fx The x-component of the force.
 * \param fy The y-component of the force.
 * \param fz The z-component of the force.
 * \return void
 *
 * The function applies a global force to the rigid body. The given force is acting directly
 * in the body's center of mass and increases the total acting force on the body. If the rigid
 * body is part of a superordinate body, the force is also acting on the superordinate body.
 * Depending on the position of the superordinate body's center of mass, the force can also
 * cause a torque in the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addForce( real fx, real fy, real fz )
{
   addForce( Vec3( fx, fy, fz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param f The acting force.
 * \return void
 *
 * The function applies a global force to the rigid body. The given force is acting directly
 * in the body's center of mass and increases the total acting force on the body. If the rigid
 * body is part of a superordinate body, the force is also acting on the superordinate body.
 * Depending on the position of the superordinate body's center of mass, the force can also
 * cause a torque in the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addForce( const Vec3& f )
{
   pe_GAMES_SECTION {
      if( !hasSuperBody() ) {
         force_ += f;
         wake();
      }
      else sb_->addForceAtPos( f, gpos_ );
   }

   pe_SCIENTIFIC_SECTION {
      // Increasing the force on this rigid body
      force_ += f;

      // Increasing the force and torque on the superordinate bodies
      if( !superBodies_.isEmpty() ) {
         for( Bodies::Iterator it=superBodies_.begin(); it!=superBodies_.end(); ++it ) {
            it->addForceAtPos( f, gpos_ );
         }
      }

      // Waking this rigid body
      else {
         pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid superbody detected" );
         wake();
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param fx The x-component of the relative force.
 * \param fy The y-component of the relative force.
 * \param fz The z-component of the relative force.
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return void
 *
 * The function applies a body relative force to the rigid body. The given force is acting at
 * the specified body relative coordinate and increases the total acting force on the rigid
 * body. Depending on the position, the force can cause a torque in the body's center of mass.
 * If the body is part of a superordinate body, the force is also acting on the superordinate
 * body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addRelForceAtRelPos( real fx, real fy, real fz, real px, real py, real pz )
{
   addForceAtPos( vectorFromBFtoWF( fx, fy, fz ), pointFromBFtoWF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param f The acting relative force.
 * \param rpos The relative coordinate.
 * \return void
 *
 * The function applies a body relative force to the rigid body. The given force is acting at
 * the specified body relative coordinate and increases the total acting force on the rigid
 * body. Depending on the position, the force can cause a torque in the body's center of mass.
 * If the body is part of a superordinate body, the force is also acting on the superordinate
 * body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addRelForceAtRelPos( const Vec3& f, const Vec3& rpos )
{
   addForceAtPos( vectorFromBFtoWF( f ), pointFromBFtoWF( rpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param fx The x-component of the relative force.
 * \param fy The y-component of the relative force.
 * \param fz The z-component of the relative force.
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return void
 *
 * The function applies a body relative force to the rigid body. The given force is acting at
 * the specified global coordinate and increases the total acting force on the rigid body.
 * Depending on the position, the force can cause a torque in the body's center of mass. If the
 * body is part of a superordinate body, the force is also acting on the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addRelForceAtPos( real fx, real fy, real fz, real px, real py, real pz )
{
   addForceAtPos( vectorFromBFtoWF( fx, fy, fz ), Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param f The acting relative force.
 * \param gpos The global coordinate.
 * \return void
 *
 * The function applies a body relative force to the rigid body. The given force is acting at
 * the specified global coordinate and increases the total acting force on the rigid body.
 * Depending on the position, the force can cause a torque in the body's center of mass. If the
 * body is part of a superordinate body, the force is also acting on the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addRelForceAtPos( const Vec3& f, const Vec3& gpos )
{
   addForceAtPos( vectorFromBFtoWF( f ), gpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param fx The x-component of the global force.
 * \param fy The y-component of the global force.
 * \param fz The z-component of the global force.
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return void
 *
 * The function applies a global force to the rigid body. The given force is acting at the
 * specified body relative coordinate and increases the total acting force on the rigid body.
 * Depending on the position, the force can cause a torque in the body's center of mass. If the
 * body is part of a superordinate body, the force is also acting on the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addForceAtRelPos( real fx, real fy, real fz, real px, real py, real pz )
{
   addForceAtPos( Vec3( fx, fy, fz ), pointFromBFtoWF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param f The acting global force.
 * \param rpos The relative coordinate.
 * \return void
 *
 * The function applies a global force to the rigid body. The given force is acting at the
 * specified body relative coordinate and increases the total acting force on the rigid body.
 * Depending on the position, the force can cause a torque in the body's center of mass. If the
 * body is part of a superordinate body, the force is also acting on the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addForceAtRelPos( const Vec3& f, const Vec3& rpos )
{
   addForceAtPos( f, pointFromBFtoWF( rpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param fx The x-component of the global force.
 * \param fy The y-component of the global force.
 * \param fz The z-component of the global force.
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return void
 *
 * The given force is acting at the specified coordinate and increases the total acting
 * force on the rigid body. Depending on the position, the force can cause a torque in the
 * body's center of mass. If the body is part of a superordinate body, the force is also
 * acting on the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addForceAtPos( real fx, real fy, real fz, real px, real py, real pz )
{
   addForceAtPos( Vec3( fx, fy, fz ), Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increases the force acting in the body's center of mass.
 *
 * \param f The acting global force.
 * \param gpos The global coordinate.
 * \return void
 *
 * The given force is acting at the specified coordinate and increases the total acting
 * force on the rigid body. Depending on the position, the force can cause a torque in the
 * body's center of mass. If the body is part of a superordinate body, the force is also
 * acting on the superordinate body.
 *
 * \b Note: If the pe::pe_GAMES_MODE is active and the body is part of a union, the force is
 * directly added to the union.
 */
inline void RigidBody::addForceAtPos( const Vec3& f, const Vec3& gpos )
{
   pe_GAMES_SECTION {
      if( !hasSuperBody() ) {
         force_  += f;
         torque_ += ( gpos - gpos_ ) % f;
         wake();
      }
      else sb_->addForceAtPos( f, gpos );
   }

   pe_SCIENTIFIC_SECTION {
      // Increasing the force and torque on this rigid body
      force_  += f;
      torque_ += ( gpos - gpos_ ) % f;

      // Increasing the force and torque on the superordinate bodies
      if( !superBodies_.isEmpty() ) {
         for( Bodies::Iterator it=superBodies_.begin(); it!=superBodies_.end(); ++it ) {
            it->addForceAtPos( f, gpos );
         }
      }

      // Waking this rigid body
      else {
         wake();
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increasing the torque acting in the body's center of mass.
 *
 * \param tx The x-component of the torque.
 * \param ty The y-component of the torque.
 * \param tz The z-component of the torque.
 * \return void
 *
 * The torque is acting directly in the body's center of mass and increases the total acting
 * torque on the body. If the rigid body is part of a superordinate body, the torque is applied
 * to the superordinate body instead. It is \b not possible to apply a torque on subordinate
 * rigid bodies!
 */
inline void RigidBody::addTorque( real tx, real ty, real tz )
{
   addTorque( Vec3( tx, ty, tz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Increasing the torque acting in the body's center of mass.
 *
 * \param t The acting torque.
 * \return void
 *
 * The torque is acting directly in the body's center of mass and increases the total acting
 * torque on the body. If the rigid body is part of a superordinate body, the torque is applied
 * to the superordinate body instead. It is \b not possible to apply a torque on subordinate
 * rigid bodies!
 */
inline void RigidBody::addTorque( const Vec3& t )
{
   if( !hasSuperBody() ) {
      torque_ += t;
      wake();
   }
   else sb_->addTorque( t );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resetting all acting forces and torques from the rigid body.
 *
 * \return void
 */
inline void RigidBody::resetForce()
{
   force_  = real(0);
   torque_ = real(0);
}
//*************************************************************************************************




//=================================================================================================
//
//  IMPULSE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Applying an impulse in the body's center of mass.
 *
 * \param jx The x-component of the impulse.
 * \param jy The y-component of the impulse.
 * \param jz The z-component of the impulse.
 * \return void
 *
 * The impulse is acting directly in the body's center of mass and instantaneously changes
 * the linear velocity of the rigid body. If the body is part of a superordinate body, the
 * impulse is also acting on the superordinate body. Depending on the position of the
 * superordinate body's center of mass, the impulse can also change the angular velocity
 * of the rigid body (and the superordinate body).
 */
inline void RigidBody::addImpulse( real jx, real jy, real jz )
{
   addImpulse( Vec3( jx, jy, jz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Applying an impulse in the body's center of mass.
 *
 * \param j The acting impulse.
 * \return void
 *
 * The impulse is acting directly in the body's center of mass and instantaneously changes
 * the linear velocity of the rigid body. If the body is part of a superordinate body, the
 * impulse is also acting on the superordinate body. Depending on the position of the
 * superordinate body's center of mass, the impulse can also change the angular velocity
 * of the rigid body (and the superordinate body).
 */
inline void RigidBody::addImpulse( const Vec3& j )
{
   if( !hasSuperBody() ) {
      v_ += j * invMass_;
      wake();
   }
   else sb_->addImpulseAtPos( j, gpos_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Applying an impulse at the given global coordinate.
 *
 * \param jx The x-component of the impulse.
 * \param jy The y-component of the impulse.
 * \param jz The z-component of the impulse.
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return void
 *
 * The given impulse is acting at the specified coordinate and instantaneously changes the linear
 * velocity of the rigid body. Depending on the position of the body's center of mass, the impulse
 * can also change the angular velocity. If the rigid body is part of a superordinate body, the
 * impulse is also acting on the superordinate body.
 */
inline void RigidBody::addImpulseAtPos( real jx, real jy, real jz, real px, real py, real pz )
{
   addImpulseAtPos( Vec3( jx, jy, jz ), Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Applying an impulse at the given global coordinate.
 *
 * \param j The acting impulse.
 * \param p The global coordinate.
 * \return void
 *
 * The given impulse is acting at the specified coordinate and instantaneously changes the linear
 * velocity of the rigid body. Depending on the position of the body's center of mass, the impulse
 * can also change the angular velocity. If the rigid body is part of a superordinate body, the
 * impulse is also acting on the superordinate body.
 */
inline void RigidBody::addImpulseAtPos( const Vec3& j, const Vec3& p )
{
   if( !hasSuperBody() ) {
      v_ += j * invMass_;
      w_ += ( R_ * Iinv_ * trans(R_) ) * ( ( p - gpos_ ) % j );
      wake();
   }
   else sb_->addImpulseAtPos( j, p );
}
//*************************************************************************************************




//=================================================================================================
//
//  FUNCTIONS FOR INTERNAL CHANGES IN COMPOUND GEOMETRIES
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Signals an internal modification of a contained subordiante body.
 *
 * \return void
 *
 * This function can be used by derived primitive geometries to signal an internal modification
 * to its superordinate body.
 */
inline void RigidBody::signalModification()
{
   if( hasSuperBody() ) sb_->handleModification();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signals a position change of a contained subordiante body.
 *
 * \return void
 *
 * This function can be used by derived primitive geometries to signal a translation or
 * a position change to its superordinate body.
 */
inline void RigidBody::signalTranslation()
{
   if( hasSuperBody() ) sb_->handleTranslation();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signals an orientation change of a contained subordiante body.
 *
 * \return void
 *
 * This function can be used by derived primitive geometries to signal a rotation or
 * orientation change to its superordinate body.
 */
inline void RigidBody::signalRotation()
{
   if( hasSuperBody() ) sb_->handleRotation();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Signals a fixation change of a contained subordiante body.
 *
 * \return void
 *
 * This function can be used by derived primitive geometries to signal a change of the fixation
 * flag to its superordinate body.
 */
inline void RigidBody::signalFixation()
{
   if( hasSuperBody() && sb_->isFixed() != fixed_ ) sb_->handleFixation();
}
//*************************************************************************************************




//=================================================================================================
//
//  CONTACT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Registering a single attached contact with the rigid body.
 *
 * \param contact The contact to be registered with the rigid body.
 * \return void
 */
inline void RigidBody::registerContact( ContactID contact )
{
   pe_INTERNAL_ASSERT( !hasSuperBody(), "Invalid contact on subordinate body detected" );
   contacts_.pushBack( contact );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clears all contacts registered with the rigid body.
 *
 * \return void
 */
inline void RigidBody::clearContacts()
{
   contacts_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether any contacts are registered with the rigid body.
 *
 * \return \a true if at least one contact is registered with the rigid body, \a false if not.
 */
inline bool RigidBody::hasContacts() const
{
   return !contacts_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of currently registered contacts.
 *
 * \return The number of registered contacts.
 */
inline size_t RigidBody::countContacts() const
{
   return contacts_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first attached contact.
 *
 * \return Iterator to the first attached contact.
 */
inline RigidBody::ContactIterator RigidBody::beginContacts()
{
   return contacts_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first attached contact.
 *
 * \return Iterator to the first attached contact.
 */
inline RigidBody::ConstContactIterator RigidBody::beginContacts() const
{
   return contacts_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last attached contact.
 *
 * \return Iterator just past the last attached contact.
 */
inline RigidBody::ContactIterator RigidBody::endContacts()
{
   return contacts_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last attached contact.
 *
 * \return Iterator just past the last attached contact.
 */
inline RigidBody::ConstContactIterator RigidBody::endContacts() const
{
   return contacts_.end();
}
//*************************************************************************************************




//=================================================================================================
//
//  ATTACHABLE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether any attachable is registered with the rigid body.
 *
 * \return \a true in case at least one attachable is registered with the rigid body, \a false otherwise.
 */
inline bool RigidBody::isAttached() const
{
   return !attachables_.isEmpty();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of currently registered attachables.
 *
 * \return The number of registered attachables.
 */
inline size_t RigidBody::countAttachables() const
{
   return attachables_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first registered attachable.
 *
 * \return Iterator to the first registered attachable.
 */
inline RigidBody::AttachableIterator RigidBody::beginAttachables()
{
   return attachables_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first registered attachable.
 *
 * \return Iterator to the first registered attachable.
 */
inline RigidBody::ConstAttachableIterator RigidBody::beginAttachables() const
{
   return attachables_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last registered attachable.
 *
 * \return Iterator just past the last registered attachable.
 */
inline RigidBody::AttachableIterator RigidBody::endAttachables()
{
   return attachables_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last registered attachable.
 *
 * \return Iterator just past the last registered attachable.
 */
inline RigidBody::ConstAttachableIterator RigidBody::endAttachables() const
{
   return attachables_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first attached rigid body.
 *
 * \return Iterator to the first attached rigid body.
 */
inline RigidBody::AttachedBodyIterator RigidBody::beginAttachedBodies()
{
   return attachedBodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first attached rigid body.
 *
 * \return Iterator to the first attached rigid body.
 */
inline RigidBody::ConstAttachedBodyIterator RigidBody::beginAttachedBodies() const
{
   return attachedBodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last attached rigid body.
 *
 * \return Iterator just past the last attached rigid body.
 */
inline RigidBody::AttachedBodyIterator RigidBody::endAttachedBodies()
{
   return attachedBodies_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last attached rigid body.
 *
 * \return Iterator just past the last attached rigid body.
 */
inline RigidBody::ConstAttachedBodyIterator RigidBody::endAttachedBodies() const
{
   return attachedBodies_.end();
}
//*************************************************************************************************




//=================================================================================================
//
//  MPI FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether the rigid body is remote or not.
 *
 * \return \a true in case of a remote rigid body, \a false in case of a local body.
 */
inline bool RigidBody::isRemote() const
{
   return remote_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the remote flag of the rigid body.
 *
 * \param remote \a true to declare the rigid body remote, \a false declare it local.
 * \return void
 *
 * This function sets the remote flag of the rigid body. Note that this function should not be
 * used explicitly, but is automatically called during the MPI communication to set the remote
 * status of a rigid body within the simulation world. Using this function explicitly may lead
 * to simulation errors during a parallel simulation!
 */
inline void RigidBody::setRemote( bool remote )
{
   remote_ = remote;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns whether the rigid body is global or not.
 *
 * \return \a true in case of a global rigid body, \a false otherwise.
 */
inline bool RigidBody::isGlobal() const
{
   return global_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the global flag of the rigid body.
 *
 * \return void
 *
 * This function declares the rigid body as a global body. Note that this function should not be
 * used explicitly, but is automatically called during the setup of global rigid bodies. Using
 * this function explicitly may lead to simulation errors during a parallel simulation!
 */
inline void RigidBody::setGlobal()
{
   global_ = true;    // Marking the rigid body as global
}
//*************************************************************************************************




//=================================================================================================
//
//  RIGID BODY SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Rigid body setup functions */
//@{
void destroy( BodyID body );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Rigid body operators */
//@{
std::ostream& operator<<( std::ostream& os, const RigidBody& b );
std::ostream& operator<<( std::ostream& os, ConstBodyID b );
//@}
//*************************************************************************************************

} // namespace pe

#endif
