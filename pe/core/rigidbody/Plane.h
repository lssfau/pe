//=================================================================================================
/*!
 *  \file pe/core/rigidbody/Plane.h
 *  \brief Header file for the Plane class
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

#ifndef _PE_CORE_RIGIDBODY_PLANE_H
#define _PE_CORE_RIGIDBODY_PLANE_H


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <iosfwd>
#include <pe/core/Configuration.h>
#include <pe/core/rigidbody/PlaneTrait.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Types.h>
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
/*!\defgroup plane Plane
 * \ingroup geometric_primitive
 *
 * The plane module combines all necessary functionality for the geometric primitive Plane.
 * A detailed description of the plane primitive can be found with the class Plane. This
 * description also containes examples for the setup and destruction of a plane.
 */
/*!\brief Plane geometry.
 * \ingroup plane
 *
 * \section plane_general General
 *
 * The Plane class represents the geometric primitive plane, which is one of the basic geometric
 * primitives of the \b pe physics engine. The class is derived from the GeomPrimitive base
 * class, which makes the plane both a geometric primitive and a rigid body.\n
 * The plane geometry is an infinite rigid body dividing the global space in two half spaces.
 * One of these half spaces is considered to be inside the plane. Bodies entering this half
 * space are therefore colliding with the plane. The other half space is considered to be
 * outside the plane. The plane is represented by the following equation:
 *
 *                                \f[ ax + by + cz = d , \f]
 *
 * where \a a, \a b and \a c are the x, y and z component of the normal vector. The normal
 * \a n of the plane is a normalized vector that points towards the half space outside the
 * plane. \a d is the distance/displacement from the origin of the global world frame to the
 * plane. A positive value of \a d indicates that the origin of the global world frame is
 * inside the plane, whereas a negative value of \a d indicates that the origin is outside
 * the plane. A value of 0 therefore indicates that the origin is on the surface of the plane.
 * The global position \f$ (x,y,z) \f$ of the plane can be considered the anchor point of the
 * plane. Rotations that are performed via the setOrientation() or the rotate() functions
 * rotate the plane around this anchor point.
 *
 * \image html plane.png
 * \image latex plane.eps "Plane geometry" width=520pt
 *
 *
 * \section plane_setup Creating and destroying a plane primitive
 *
 * In order to create a plane primitive, one of the following plane creation functions can be
 * used:
 *
 * - pe::createPlane( id_t uid, real a, real b, real c, real d, MaterialID material, bool visible )
 * - pe::createPlane( id_t uid, const Vec3& normal, real d, MaterialID material, bool visible )
 * - pe::createPlane( id_t uid, real a, real b, real c, real x, real y, real z, MaterialID material, bool visible )
 * - pe::createPlane( id_t uid, const Vec3& normal, const Vec3& gpos, MaterialID material, bool visible )
 *
 * Note that in case of the first two functions, the global position of the plane is per default
 * set to the surface point closest to the origin of the global world frame. In case of the last
 * two functions, the global position is set according to the specified global coordinate.\n
 * In order to destroy a specific plane primitive (which can also be contained in a Union), the
 * following function can be used:
 *
 * - pe::destroy( BodyID body )
 *
 * The following example demonstrates the creation and destruction of a plane primitive:

   \code
   // Creates the granite plane 1 with normal (0,0,1) and displacement 0.5. Per default the
   // plane is visible in all visualizations. Note that the plane is automatically added to
   // the simulation world and is immediately part of the entire simulation. The function
   // returns a handle to the newly created plane, which can be used to for instance rotate
   // the plane around the global y-axis.
   PlaneID plane = createPlane( 1, 0.0, 0.0, 1.0, 0.5, granite );
   plane->rotate( 0.0, PI/3.0, 0.0 );

   // Rigid body simulation
   ...

   // Destroying the granite plane
   destroy( plane );
   \endcode

 * It is possible to add plane primitives to a union compound geometry (for more details see
 * the Union class description). In case the plane is created inside a pe::pe_CREATE_UNION
 * section, the plane is automatically added to the newly created union:

   \code
   PlaneID plane;

   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the oak plane 2 with normal (-1,0,-2) and anchor point (-1,0,0). Since
      // the plane is created inside a pe_CREATE_UNION section, the plane is directly added
      // to the union 'newunion' and is henceforth considered to be part of the union.
      plane = createPlane( 2, Vec3( -1.0, 0.0, -2.0 ), Vec3( -1.0, 0.0, 0.0 ), oak );
      ...
   }

   // Destroying the plane primitive (NOT the entire union)
   destroy( box );
   \endcode

 * In case of a MPI parallel simulation, planes may only be created inside a pe_GLOBAL_SECTION
 * environment:

   \code
   pe_GLOBAL_SECTION {
      // Creating the global plane 1 with normal (0,0,1) and anchor point (2,1,0). Since the
      // plane is created inside a pe_GLOBAL_SECTION it is globally known on all MPI processes.
      // Therefore any function call that might modify the plane (translations, rotations, ...)
      // has to be executed on all processes (i.e., it is invalid to modify the plane inside
      // a pe_EXCLUSIVE_SECTION)!
      createPlane( 1, Vec3( 0.0, 0.0, 1.0 ), Vec3( 2.0, 1.0, 0.0 ), iron );
   }
   \endcode
 */
class Plane : public PlaneTrait<Config>
{
protected:
   //**Type definitions****************************************************************************
   typedef PlaneTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit Plane( id_t sid, id_t uid, const Vec3& gpos, const Vec3& normal,
                   real d, MaterialID material, bool visible );
   explicit Plane( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                   MaterialID material, bool visible );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Plane();
   //@}
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   struct Parameters : public GeomPrimitive::Parameters {
      // Normal and distance can be derived from geometric primitive properties (position and orientation).
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
   virtual void update( const Vec3& dp );  // Translation update of a subordinate plane
   virtual void update( const Quat& dq );  // Rotation update of a subordinate plane
   //@}
   //**********************************************************************************************

private:
   //**Plane setup functions***********************************************************************
   /*! \cond PE_INTERNAL */
   friend PlaneID createPlane( id_t uid, Vec3 normal, real d,
                               MaterialID material, bool visible );
   friend PlaneID createPlane( id_t uid, Vec3 normal, const Vec3& gpos,
                               MaterialID material, bool visible );
   friend PlaneID instantiatePlane( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                    const Quat& q, MaterialID material,
                                    bool visible, bool reg );
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
/*!\brief Calculates the depth of a point relative to the plane's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the plane and a negative value,
 * if the point lies outside the plane.
 */
inline real Plane::getRelDepth( real /*px*/, real /*py*/, real pz ) const
{
   return -pz;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the plane's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the plane and a negative value,
 * if the point lies outside the plane.
 */
inline real Plane::getRelDepth( const Vec3& rpos ) const
{
   return -rpos[2];
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
 * Returns a positive value, if the point lies inside the plane and a negative value,
 * if the point lies outside the plane.
 */
inline real Plane::getDepth( real px, real py, real pz ) const
{
   const Vec3T gpos( px, py, pz );
   return d_ - ( gpos * normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return Depth of the global point.
 *
 * Returns a positive value, if the point lies inside the plane and a negative value,
 * if the point lies outside the plane.
 */
inline real Plane::getDepth( const Vec3& gpos ) const
{
   return d_ - ( trans(gpos) * normal_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the plane's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the plane and a negative value, if the
 * point lies inside the plane.
 */
inline real Plane::getRelDistance( real px, real py, real pz ) const
{
   return -getRelDepth( px, py, pz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the plane's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the plane and a negative value, if the
 * point lies inside the plane.
 */
inline real Plane::getRelDistance( const Vec3& rpos ) const
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
 * Returns a positive value, if the point lies outside the plane and a negative value, if the
 * point lies inside the plane.
 */
inline real Plane::getDistance( real px, real py, real pz ) const
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
 * Returns a positive value, if the point lies outside the plane and a negative value, if the
 * point lies inside the plane.
 */
inline real Plane::getDistance( const Vec3& gpos ) const
{
   return -getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  PLANE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Plane setup functions */
//@{
inline PlaneID createPlane( id_t uid, real a, real b, real c,
                            real d, MaterialID material, bool visible=true );
       PlaneID createPlane( id_t uid, Vec3 normal, real d,
                            MaterialID material, bool visible=true );
inline PlaneID createPlane( id_t uid, real a, real b, real c,
                            real x, real y, real z, MaterialID material, bool visible=true );
       PlaneID createPlane( id_t uid, Vec3 normal, const Vec3& gpos,
                            MaterialID material, bool visible=true );
       PlaneID instantiatePlane( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                 const Quat& q, MaterialID material,
                                 bool visible, bool reg=true );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new plane.
 * \ingroup plane
 *
 * \param uid The user-specific ID of the plane.
 * \param a The x-component of the plane's normal vector.
 * \param b The y-component of the plane's normal vector.
 * \param c The z-component of the plane's normal vector.
 * \param d The plane's displacement from the global origin.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 * \return Handle for the new plane.
 * \exception std::invalid_argument Invalid plane parameters.
 * \exception std::logic_error Invalid creation of a plane inside an exclusive section.
 * \exception std::logic_error Invalid creation of a plane outside a global section.
 *
 * This function creates a new plane with the user-specific ID \a uid, the normal \a (a,b,c),
 * the distance/displacement from the origin of the global world frame \a d, and the material
 * \a material. The global position of the plane is per default set to the closest surface
 * point to the origin of the global world frame. The \a visible flag sets the plane
 * (in-)visible in all visualizations.
 *
 * \image html plane.png
 * \image latex plane.eps "Plane geometry" width=520pt
 *
 * The following code example illustrates the setup of a plane:

   \code
   // Creating the iron plane 1 with the normal (1,0,2) and the displacement 1.0. Per default
   // the plane is visible in all visualizations. Note that the plane is automatically added
   // to the simulation world and is immediately part of the entire simulation. The function
   // returns a handle to the newly created plane, which can be used to for instance rotate
   // the plane around the global y-axis.
   PlaneID plane = createPlane( 1, 1.0, 0.0, 2.0, 1.0, iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the plane is created inside a pe::pe_CREATE_UNION section, the plane is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron plane 2 with the normal (-1,0,-2) and the displacement 1.0.
      // Since the plane is created inside a pe_CREATE_UNION section, the plane is directly
      // added to the union 'newunion' and is henceforth considered to be part of the union.
      createPlane( 2, -1.0, 0.0, -2.0, 1.0, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, planes may only be created inside a pe_GLOBAL_SECTION
 * environment:

   \code
   pe_GLOBAL_SECTION {
      // Creating the global plane 1 with normal (0,0,1) and displacement 2.0. Since the plane
      // is created inside a pe_GLOBAL_SECTION it is globally known on all MPI processes.
      // Therefore any function call that might modify the plane (translations, rotations, ...)
      // has to be executed on all processes (i.e., it is invalid to modify the plane inside
      // a pe_EXCLUSIVE_SECTION)!
      createPlane( 1, 0.0, 0.0, 1.0, 2.0, iron );
   }
   \endcode
 */
inline PlaneID createPlane( id_t uid, real a, real b, real c, real d,
                            MaterialID material, bool visible )
{
   return createPlane( uid, Vec3(a,b,c), d, material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new plane containing the surface point (x,y,z).
 * \ingroup plane
 *
 * \param uid The user-specific ID of the plane.
 * \param a The x-component of the plane's normal vector.
 * \param b The y-component of the plane's normal vector.
 * \param c The z-component of the plane's normal vector.
 * \param x The global x-position of the surface point.
 * \param y The global y-position of the surface point.
 * \param z The global z-position of the surface point.
 * \param material The material of the plane.
 * \param visible Specifies if the plane is visible in a visualization.
 * \return Handle for the new plane.
 * \exception std::invalid_argument Invalid plane parameters.
 *
 * This function creates a new plane with the user-specific ID \a uid, the normal \a (a,b,c),
 * the surface point \a (x,y,z), and the material \a material. The given coordinate \a (x,y,z)
 * additionally specifies the anchor point of the plane. The \a visible flag sets the plane
 * (in-)visible in all visualizations.
 *
 * \image html plane.png
 * \image latex plane.eps "Plane geometry" width=520pt
 *
 * The following code example illustrates the setup of a plane:

   \code
   // Creating the iron plane 1 with the normal (1,0,2) and the anchor point (1,0,0). Per
   // default the plane is visible in all visualizations. Note that the plane is automatically
   // added to the simulation world and is immediately part of the entire simulation. The
   // function returns a handle to the newly created plane, which can be used to for instance
   // rotate the plane around the global y-axis.
   PlaneID plane = createPlane( 1, 1.0, 0.0, 2.0, 1.0, 0.0, 0.0, iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the plane is created inside a pe::pe_CREATE_UNION section, the plane is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron plane 2 with the normal (-1,0,-2) and the anchor point (-1,0,0).
      // Since the plane is created inside a pe_CREATE_UNION section, the plane is directly
      // added to the union 'newunion' and is henceforth considered to be part of the union.
      createPlane( 2, -1.0, 0.0, -2.0, -1.0, 0.0, 0.0, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, planes may only be created inside a pe_GLOBAL_SECTION
 * environment:

   \code
   pe_GLOBAL_SECTION {
      // Creating the global plane 1 with normal (0,0,1) and anchor point (2,1,0). Since the
      // plane is created inside a pe_GLOBAL_SECTION it is globally known on all MPI processes.
      // Therefore any function call that might modify the plane (translations, rotations, ...)
      // has to be executed on all processes (i.e., it is invalid to modify the plane inside
      // a pe_EXCLUSIVE_SECTION)!
      createPlane( 1, 0.0, 0.0, 1.0, 2.0, 1.0, 0.0, iron );
   }
   \endcode
 */
inline PlaneID createPlane( id_t uid, real a, real b, real c,
                            real x, real y, real z, MaterialID material, bool visible )
{
   return createPlane( uid, Vec3(a,b,c), Vec3(x,y,z), material, visible );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Plane operators */
//@{
std::ostream& operator<<( std::ostream& os, const Plane& p );
std::ostream& operator<<( std::ostream& os, ConstPlaneID p );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC COUNT FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Plane,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to rigid
 * bodies and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<Plane>( RigidBody *const * first,
                                       RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Plane,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<const Plane>( RigidBody *const * first,
                                             RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Plane,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<Plane>( const RigidBody *const * first,
                                       const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Plane,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<const Plane>( const RigidBody *const * first,
                                             const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Plane,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<Plane>( GeomPrimitive *const * first,
                                       GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Plane,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<const Plane>( GeomPrimitive *const * first,
                                             GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Plane,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<Plane>( const GeomPrimitive *const * first,
                                       const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to planes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of planes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Plane,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to planes.
 */
template<>
inline size_t polymorphicCount<const Plane>( const GeomPrimitive *const * first,
                                             const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == planeType ) ++count;
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
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Plane,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a plane.
 */
template<>
inline RigidBody *const * polymorphicFind<Plane>( RigidBody *const * first,
                                                  RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, RigidBody );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Plane,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a plane.
 */
template<>
inline RigidBody *const * polymorphicFind<const Plane>( RigidBody *const * first,
                                                        RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, RigidBody );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Plane,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a plane.
 */
template<>
inline const RigidBody *const * polymorphicFind<Plane>( const RigidBody *const * first,
                                                        const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, const RigidBody );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Plane,const RigidBody). The function traverses the range \f$ [first,last) \f$ of
 * pointers to rigid bodies until it finds the next pointer to a plane.
 */
template<>
inline const RigidBody *const * polymorphicFind<const Plane>( const RigidBody *const * first,
                                                              const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, const RigidBody );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Plane,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives until it finds the next pointer to a plane.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<Plane>( GeomPrimitive *const * first,
                                                      GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, GeomPrimitive );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Plane,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a plane.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<const Plane>( GeomPrimitive *const * first,
                                                            GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, GeomPrimitive );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Plane,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a plane.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<Plane>( const GeomPrimitive *const * first,
                                                            const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Plane, const GeomPrimitive );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a plane in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a plane.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Plane,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives until it finds the next pointer to a plane.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<const Plane>( const GeomPrimitive *const * first,
                                                                  const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Plane, const GeomPrimitive );

   while( first != last && (*first)->getType() != planeType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
