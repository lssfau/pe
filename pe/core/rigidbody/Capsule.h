//=================================================================================================
/*!
 *  \file pe/core/rigidbody/Capsule.h
 *  \brief Header file for the Capsule class
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

#ifndef _PE_CORE_RIGIDBODY_CAPSULE_H_
#define _PE_CORE_RIGIDBODY_CAPSULE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <iosfwd>
#include <pe/core/Configuration.h>
#include <pe/core/rigidbody/CapsuleTrait.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Types.h>
#include <pe/math/shims/Square.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Algorithm.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup capsule Capsule
 * \ingroup geometric_primitive
 *
 * The capsule module combines all necessary functionality for the geometric primitive Capsule.
 * A detailed description of the capsule primitive can be found with the class Capsule. This
 * description also containes examples for the setup and destruction of a capsule.
 */
/*!\brief Capsule geometry.
 * \ingroup capsule
 *
 * \section capsule_general General
 *
 * The Capsule class represents the geometric primitive capsule, which is one of the basic
 * geometric primitives of the \b pe physics engine. The class is derived from the GeomPrimitive
 * base class, which makes the capsule both a geometric primitive and a rigid body.\n
 * A capsule is the combination of a cylinder and two hemisphere caps at both ends of the cylinder.
 * This combination allows to calculate a unique normal on each point of the capsule's surface.
 * In order to setup a capsule, the two values radius and length are required: radius specifies
 * the radius of both the cylinder part and the two hemispheres, length is the length of the
 * cylinder part. A capsule is created axis-aligned with the x-axis (the length of the cylinder
 * part is parallel to the x-axis).
 *
 * \image html capsule.png
 * \image latex capsule.eps "Capsule geometry" width=200pt
 *
 *
 * \section capsule_setup Creating and destroying a capsule primitive
 *
 * In order to create a capsule primitive, one of the following capsule creation functions can
 * be used:
 *
 * - pe::createCapsule( id_t uid, real x, real y, real z, real radius, real length, MaterialID material, bool visible )
 * - pe::createCapsule( id_t uid, const Vec3 &gpos, real radius, real length, MaterialID material, bool visible )
 *
 * In order to destroy a specific capsule primitive (which can also be contained in a Union), the
 * following function can be used:
 *
 * - pe::destroy( BodyID body )
 *
 * The following example demonstrates the creation and destruction of a capsule primitive:

   \code
   // Creates the iron capsule 1 at the global position ( 4.2, 3.7, -0.6 ) with radius 1.6
   // and length 6.4. Per default the capsule is visible in all visualizations. Note that
   // the capsule is automatically added to the simulation world and is immediately part
   // of the entire simulation. The function returns a handle to the newly created capsule,
   // which can be used to for instance rotate the capsule around the global y-axis.
   CapsuleID capsule = createCapsule( 1, 4.2, 3.7, -0.6, 1.6, 6.4, iron );
   capsule->rotate( 0.0, PI/3.0, 0.0 );

   // Rigid body simulation
   ...

   // Destroying the iron capsule
   destroy( capsule );
   \endcode

 * It is possible to add capsule primitives to a union compound geometry (for more details see
 * the Union class description). In case the capsule is created inside a pe::pe_CREATE_UNION
 * section, the capsule is automatically added to the newly created union:

   \code
   CapsuleID capsule;

   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron capsule 2 with radius 1.3 and length 2.4 at the global position
      // (-1,4,-5). Since the capsule is created inside a pe_CREATE_UNION section, the capsule
      // is directly added to the union 'newunion' and is henceforth considered to be part
      // of the union.
      capsule = createCapsule( 2, -1.0, 4.0, -5.0, 1.3, 2.4, iron );
      ...
   }

   // Destroying the capsule primitive (NOT the entire union)
   destroy( capsule );
   \endcode

 * In case of a MPI parallel simulation, capsules may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
class Capsule : public CapsuleTrait<Config>
{
protected:
   //**Type definitions****************************************************************************
   typedef CapsuleTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Capsule( id_t sid, id_t uid, const Vec3& gpos, real radius,
                     real length, MaterialID material, bool visible );
   explicit Capsule( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                     real radius, real length, MaterialID material, bool visible, bool fixed );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Capsule();
   //@}
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   struct Parameters : public GeomPrimitive::Parameters {
      real radius_, length_;
   };
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setVisible    ( bool visible );
   virtual void setPosition   ( real px, real py, real pz );
   virtual void setPosition   ( const Vec3& gpos );
   virtual void setOrientation( real r, real i, real j, real k );
   virtual void setOrientation( const Quat& q );
   //@}
   //**********************************************************************************************

   //**Translation functions***********************************************************************
   /*!\name Translation functions */
   //@{
   virtual void translate( real dx, real dy, real dz );
   virtual void translate( const Vec3& dp );
   //@}
   //**********************************************************************************************

   //**Rotation functions**************************************************************************
   /*!\name Rotation functions */
   //@{
   virtual void rotate( real x, real y, real z, real angle );
   virtual void rotate( const Vec3& axis, real angle );
   virtual void rotate( real xangle, real yangle, real zangle );
   virtual void rotate( const Vec3& euler );
   virtual void rotate( const Quat& dq );

   virtual void rotateAroundOrigin( real x, real y, real z, real angle );
   virtual void rotateAroundOrigin( const Vec3& axis, real angle );
   virtual void rotateAroundOrigin( real xangle, real yangle, real zangle );
   virtual void rotateAroundOrigin( const Vec3& euler );

   virtual void rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle );
   virtual void rotateAroundPoint( const Vec3& point, const Vec3& euler );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual bool containsRelPoint ( real px, real py, real pz ) const;
   virtual bool containsRelPoint ( const Vec3& rpos )          const;
   virtual bool containsPoint    ( real px, real py, real pz ) const;
   virtual bool containsPoint    ( const Vec3& gpos )          const;
   virtual bool isSurfaceRelPoint( real px, real py, real pz ) const;
   virtual bool isSurfaceRelPoint( const Vec3& rpos )          const;
   virtual bool isSurfacePoint   ( real px, real py, real pz ) const;
   virtual bool isSurfacePoint   ( const Vec3& gpos )          const;

   inline real getRelDepth   ( real px, real py, real pz ) const;
   inline real getRelDepth   ( const Vec3& rpos )          const;
   inline real getDepth      ( real px, real py, real pz ) const;
   inline real getDepth      ( const Vec3& gpos )          const;
   inline real getRelDistance( real px, real py, real pz ) const;
   inline real getRelDistance( const Vec3& rpos )          const;
   inline real getDistance   ( real px, real py, real pz ) const;
   inline real getDistance   ( const Vec3& gpos )          const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void update( const Vec3& dp );  // Translation update of a subordinate capsule
   virtual void update( const Quat& dq );  // Rotation update of a subordinate capsule
   //@}
   //**********************************************************************************************

private:
   //**Sphere setup functions**********************************************************************
   /*! \cond PE_INTERNAL */
   friend CapsuleID createCapsule( id_t uid, const Vec3& gpos, real radius,
                                   real length, MaterialID material, bool visible );
   friend CapsuleID instantiateCapsule( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                        const Quat& q, real radius, real length, MaterialID material,
                                        bool visible, bool fixed, bool reg );
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the capsule's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the capsule and a negative value,
 * if the point lies outside the capsule.
 */
inline real Capsule::getRelDepth( real px, real py, real pz ) const
{
   const real xabs( std::fabs( px ) );         // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   if( xabs > hlength ) {
      return ( radius_ - std::sqrt( sq(xabs-hlength) + sq(py) + sq(pz) ) );
   }
   else {
      return ( radius_ - std::sqrt( sq(py) + sq(pz) ) );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the capsule's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the capsule and a negative value,
 * if the point lies outside the capsule.
 */
inline real Capsule::getRelDepth( const Vec3& rpos ) const
{
   const real xabs( std::fabs( rpos[0] ) );    // Absolute x-distance
   const real hlength( real(0.5) * length_ );  // Capsule half length

   if( xabs > hlength ) {
      return ( radius_ - std::sqrt( sq(xabs-hlength) + sq(rpos[1]) + sq(rpos[2]) ) );
   }
   else {
      return ( radius_ - std::sqrt( sq(rpos[1]) + sq(rpos[2]) ) );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point in global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return Depth of the global point.
 *
 * Returns a positive value, if the point lies inside the capsule and a negative value,
 * if the point lies outside the capsule.
 */
inline real Capsule::getDepth( real px, real py, real pz ) const
{
   return getRelDepth( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return Depth of the global point.
 *
 * Returns a positive value, if the point lies inside the capsule and a negative value,
 * if the point lies outside the capsule.
 */
inline real Capsule::getDepth( const Vec3& gpos ) const
{
   return getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the capsule's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the capsule and a negative value,
 * if the point lies inside the capsule.
 */
inline real Capsule::getRelDistance( real px, real py, real pz ) const
{
   return -getRelDepth( px, py, pz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the capsule's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the capsule and a negative value,
 * if the point lies inside the capsule.
 */
inline real Capsule::getRelDistance( const Vec3& rpos ) const
{
   return -getRelDepth( rpos );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point in global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return Distance of the global point.
 *
 * Returns a positive value, if the point lies outside the capsule and a negative value,
 * if the point lies inside the capsule.
 */
inline real Capsule::getDistance( real px, real py, real pz ) const
{
   return -getRelDepth( pointFromWFtoBF( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return Distance of the global point.
 *
 * Returns a positive value, if the point lies outside the capsule and a negative value,
 * if the point lies inside the capsule.
 */
inline real Capsule::getDistance( const Vec3& gpos ) const
{
   return -getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  CAPSULE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Capsule setup functions */
//@{
inline CapsuleID createCapsule( id_t uid, real x, real y, real z, real radius,
                                real length, MaterialID material, bool visible=true );
       CapsuleID createCapsule( id_t uid, const Vec3& gpos, real radius,
                                real length, MaterialID material, bool visible=true );
       CapsuleID instantiateCapsule( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                     const Quat& q, real radius, real length, MaterialID material,
                                     bool visible, bool fixed, bool reg=true );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new capsule.
 * \ingroup capsule
 *
 * \param uid The user-specific ID of the capsule.
 * \param x The global x-position of the center of the capsule.
 * \param y The global y-position of the center of the capsule.
 * \param z The global z-position of the center of the capsule.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part of the capsule \f$ (0..\infty) \f$.
 * \param material The material of the capsule.
 * \param visible Specifies if the capsule is visible in a visualization.
 * \return Handle for the new capsule.
 * \exception std::invalid_argument Invalid capsule radius.
 * \exception std::invalid_argument Invalid capsule length.
 * \exception std::invalid_argument Invalid global capsule position.
 *
 * This function creates a capsule primitive in the \b pe simulation system. The capsule with
 * user-specific ID \a uid is placed at the global position \a (x,y,z), has the radius \a radius
 * and the length \a length, and consists of the material \a material. The \a visible flag sets
 * the capsule (in-)visible in all visualizations.
 *
 * \image html capsule.png
 * \image latex capsule.eps "Capsule geometry" width=200pt
 *
 * The following code example illustrates the setup of a capsule:

   \code
   // Creating the iron capsule 1 with a radius of 0.9 and a length 2.5 of at the global
   // position (2,3,4). Per default the capsule is visible in all visualizations. Note
   // that the capsule is automatically added to the simulation world and is immediately
   // part of the entire simulation. The function returns a handle to the newly created
   // capsule, which can be used to for instance rotate the capsule around the global
   // y-axis.
   CapsuleID capsule = createCapsule( 1, 2.0, 3.0, 4.0, 0.9, 2.5, iron );
   capsule->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the capsule is created inside a pe::pe_CREATE_UNION section, the capsule is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron capsule 2 with radius 1.3 and length 2.4 at the global position
      // (-1,4,-5). Since the union is created inside a pe_CREATE_UNION section, the capsule
      // is directly added to the union 'newunion' and is henceforth considered to be part
      // of the union.
      createCapsule( 2, -1.0, 4.0, -5.0, 1.3, 2.4, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, capsules may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
inline CapsuleID createCapsule( id_t uid, real x, real y, real z, real radius,
                                real length, MaterialID material, bool visible )
{
   return createCapsule( uid, Vec3(x,y,z), radius, length, material, visible );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Capsule operators */
//@{
std::ostream& operator<<( std::ostream& os, const Capsule& cc );
std::ostream& operator<<( std::ostream& os, ConstCapsuleID cc );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC COUNT FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Capsule,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<Capsule>( RigidBody *const * first,
                                         RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Capsule,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<const Capsule>( RigidBody *const * first,
                                               RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Capsule,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<Capsule>( const RigidBody *const * first,
                                         const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Capsule,const RigidBody). The function traverses the range \f$ [first,last) \f$ of
 * pointers to rigid bodies and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<const Capsule>( const RigidBody *const * first,
                                               const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Capsule,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<Capsule>( GeomPrimitive *const * first,
                                         GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Capsule,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<const Capsule>( GeomPrimitive *const * first,
                                               GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Capsule,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<Capsule>( const GeomPrimitive *const * first,
                                         const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to capsules in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of capsules.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Capsule,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to capsules.
 */
template<>
inline size_t polymorphicCount<const Capsule>( const GeomPrimitive *const * first,
                                               const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == capsuleType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC FIND FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Capsule,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a capsule.
 */
template<>
inline RigidBody *const * polymorphicFind<Capsule>( RigidBody *const * first,
                                                    RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, RigidBody );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Capsule,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a capsule.
 */
template<>
inline RigidBody *const * polymorphicFind<const Capsule>( RigidBody *const * first,
                                                          RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, RigidBody );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Capsule,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a capsule.
 */
template<>
inline const RigidBody *const * polymorphicFind<Capsule>( const RigidBody *const * first,
                                                          const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, const RigidBody );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Capsule,const RigidBody). The function traverses the range \f$ [first,last) \f$ of
 * pointers to rigid bodies until it finds the next pointer to a capsule.
 */
template<>
inline const RigidBody *const * polymorphicFind<const Capsule>( const RigidBody *const * first,
                                                                const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, const RigidBody );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Capsule,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives until it finds the next pointer to a capsule.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<Capsule>( GeomPrimitive *const * first,
                                                        GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, GeomPrimitive );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Capsule,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a capsule.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<const Capsule>( GeomPrimitive *const * first,
                                                              GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, GeomPrimitive );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Capsule,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a capsule.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<Capsule>( const GeomPrimitive *const * first,
                                                              const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Capsule, const GeomPrimitive );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a capsule in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a capsule.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Capsule,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives until it finds the next pointer to a capsule.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<const Capsule>( const GeomPrimitive *const * first,
                                                                    const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Capsule, const GeomPrimitive );

   while( first != last && (*first)->getType() != capsuleType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
