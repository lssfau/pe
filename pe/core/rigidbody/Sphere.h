//=================================================================================================
/*!
 *  \file pe/core/rigidbody/Sphere.h
 *  \brief Header file for the Sphere class
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

#ifndef _PE_CORE_RIGIDBODY_SPHERE_H_
#define _PE_CORE_RIGIDBODY_SPHERE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <iosfwd>
#include <pe/core/Configuration.h>
#include <pe/core/rigidbody/SphereTrait.h>
#include <pe/core/Thresholds.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Algorithm.h>
#include <pe/util/Assert.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup sphere Sphere
 * \ingroup geometric_primitive
 *
 * The sphere module combines all necessary functionality for the geometric primitive Sphere.
 * A detailed description of the sphere primitive can be found with the class Sphere. This
 * description also containes examples for the setup and destruction of a sphere.
 */
/*!\brief Sphere geometry.
 * \ingroup sphere
 *
 * \section sphere_general General
 *
 * The Sphere class represents the geometric primitive sphere. The class is derived from the
 * GeomPrimitive base class, which makes the sphere both a geometric primitive and a rigid
 * body.\n
 * The sphere primitive is the simplest geometric primitive, represented only by its geometric
 * global position and its radius. A special property of the sphere geometry is that in every
 * point of its surface a unique surface normal is defined.
 *
 * \image html sphere.png
 * \image latex sphere.eps "Sphere geometry" width=200pt
 *
 *
 * \section sphere_setup Creating and destroying a sphere primitive
 *
 * In order to create a sphere primitive, one of the following sphere creation functions can be
 * used:
 *
 * - pe::createSphere( id_t uid, real x, real y, real z, real radius, MaterialID material, bool visible )
 * - pe::createSphere( id_t uid, const Vec3 &gpos, real radius, MaterialID material, bool visible )
 *
 * In order to destroy a specific sphere primitive (which can also be contained in a Union), the
 * following function can be used:
 *
 * - pe::destroy( BodyID body )
 *
 * The following example demonstrates the creation and destruction of a sphere primitive:

   \code
   // Creating an iron sphere 1 at the global position ( 4.2, 3.7, -0.6 ) with a radius of
   // 1.2. Per default the sphere is visible in all visualizations. Note that the sphere
   // is automatically added to the simulation world and is immediately part of the entire
   // simulation. The function returns a handle to the newly created sphere, which can be
   // used to for instance rotate the sphere around the global y-axis.
   SphereID sphere = createSphere( 1, 4.2, 3.7, -0.6, 1.2, iron );
   sphere->rotate( 0.0, PI/3.0, 0.0 );

   // Rigid body simulation
   ...

   // Destroying the iron sphere
   destroy( sphere );
   \endcode

 * It is possible to add sphere primitives to a union compound geometry (for more details see
 * the Union class description). In case the sphere is created inside a pe::pe_CREATE_UNION
 * section, the sphere is automatically added to the newly created union:

   \code
   SphereID sphere;

   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron sphere 2 with radius 1.3 at the global position (-1,4,-5).
      // Since the sphere is created inside a pe_CREATE_UNION section, the sphere is
      // directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      sphere = createSphere( 2, -1.0, 4.0, -5.0, 1.3, iron );
      ...
   }

   // Destroying the sphere primitive (NOT the entire union)
   destroy( sphere );
   \endcode

 * In case of a MPI parallel simulation, spheres may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
class Sphere : public SphereTrait<Config>
{
protected:
   //**Type definitions****************************************************************************
   typedef SphereTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Sphere( id_t sid, id_t uid, const Vec3& gpos,
                    real radius, MaterialID material, bool visible );
   explicit Sphere( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                    real radius, MaterialID material, bool visible, bool fixed );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Sphere();
   //@}
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   struct Parameters : public GeomPrimitive::Parameters {
      real radius_;
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
   virtual void update( const Vec3& dp );   // Translation update of a subordinate sphere
   virtual void update( const Quat& dq );   // Rotation update of a subordinate sphere
   //@}
   //**********************************************************************************************

   //**Sphere setup functions**********************************************************************
   /*! \cond PE_INTERNAL */
   friend SphereID createSphere( id_t uid, const Vec3& gpos, real radius,
                                 MaterialID material, bool visible );
   friend SphereID instantiateSphere( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                      const Quat& q, real radius, MaterialID material,
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
/*!\brief Calculates the depth of a point relative to the sphere's geometric center.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the sphere and a negative value,
 * if the point lies outside the sphere.
 */
inline real Sphere::getRelDepth( real px, real py, real pz ) const
{
   return ( radius_ - Vec3( px, py, pz ).length() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point relative to the sphere's geometric center.
 *
 * \param rpos The relative coordinate.
 * \return Depth of the relative point.
 *
 * Returns a positive value, if the point lies inside the sphere and a negative value,
 * if the point lies outside the sphere.
 */
inline real Sphere::getRelDepth( const Vec3& rpos ) const
{
   return ( radius_ - rpos.length() );
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
 * Returns a positive value, if the point lies inside the sphere and a negative value,
 * if the point lies outside the sphere.
 */
inline real Sphere::getDepth( real px, real py, real pz ) const
{
   const Vec3 gpos( px, py, pz );
   return ( radius_ - ( gpos - gpos_ ).length() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the depth of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return Depth of the global point.
 *
 * Returns a positive value, if the point lies inside the sphere and a negative value,
 * if the point lies outside the sphere.
 */
inline real Sphere::getDepth( const Vec3& gpos ) const
{
   return ( radius_ - ( gpos - gpos_ ).length() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the sphere's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the sphere and a negative value, if the
 * point lies inside the sphere.
 */
inline real Sphere::getRelDistance( real px, real py, real pz ) const
{
   return ( Vec3( px, py, pz ).length() - radius_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the sphere's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the sphere and a negative value, if the
 * point lies inside the sphere.
 */
inline real Sphere::getRelDistance( const Vec3& rpos ) const
{
   return ( rpos.length() - radius_ );
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
 * Returns a positive value, if the point lies outside the sphere and a negative value, if the
 * point lies inside the sphere.
 */
inline real Sphere::getDistance( real px, real py, real pz ) const
{
   return ( ( Vec3( px, py, pz ) - gpos_ ).length() - radius_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point in global coordinates.
 *
 * \param gpos The global coordinate.
 * \return Distance of the global point.
 *
 * Returns a positive value, if the point lies outside the sphere and a negative value, if the
 * point lies inside the sphere.
 */
inline real Sphere::getDistance( const Vec3& gpos ) const
{
   return ( ( gpos - gpos_ ).length() - radius_ );
}
//*************************************************************************************************




//=================================================================================================
//
//  SPHERE SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Sphere setup functions */
//@{
inline SphereID createSphere( id_t uid, real x, real y, real z, real radius,
                              MaterialID material, bool visible=true );
       SphereID createSphere( id_t uid, const Vec3& gpos, real radius,
                              MaterialID material, bool visible=true );
       SphereID instantiateSphere( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                   const Quat& q, real radius, MaterialID material,
                                   bool visible, bool fixed, bool reg=true );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new sphere.
 * \ingroup sphere
 *
 * \param uid The user-specific ID of the sphere.
 * \param x The global x-position of the center of the sphere.
 * \param y The global y-position of the center of the sphere.
 * \param z The global z-position of the center of the sphere.
 * \param radius The radius of the sphere \f$ (0..\infty) \f$.
 * \param material The material of the sphere.
 * \param visible Specifies if the sphere is visible in a visualization.
 * \return Handle for the new sphere.
 * \exception std::invalid_argument Invalid sphere radius.
 * \exception std::invalid_argument Invalid global sphere position.
 *
 * This function creates a sphere primitive in the \b pe simulation system. The sphere with
 * user-specific ID \a uid is placed at the global position \a (x,y,z), has the radius \a radius,
 * and consists of the material \a material. The \a visible flag sets the sphere (in-)visible
 * in all visualizations.
 *
 * \image html sphere.png
 * \image latex sphere.eps "Sphere geometry" width=200pt
 *
 * The following code example illustrates the setup of a sphere:

   \code
   // Creating the iron sphere 1 with a radius of 2.5 at the global position (2,3,4).
   // Per default the sphere is visible in all visualizations. Note that the sphere is
   // automatically added to the simulation world and is immediately part of the entire
   // simulation. The function returns a handle to the newly created sphere, which can
   // be used to for instance rotate the sphere around the global y-axis.
   SphereID sphere = createSphere( 1, 2.0, 3.0, 4.0, 2.5, iron );
   sphere->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the sphere is created inside a pe::pe_CREATE_UNION section, the sphere is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron sphere 2 with radius 1.3 at the global position (-1,4,-5).
      // Since the union is created inside a pe_CREATE_UNION section, the sphere is
      // directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      createSphere( 2, -1.0, 4.0, -5.0, 1.3, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, spheres may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
inline SphereID createSphere( id_t uid, real x, real y, real z, real radius,
                              MaterialID material, bool visible )
{
   return createSphere( uid, Vec3(x,y,z), radius, material, visible );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Sphere operators */
//@{
std::ostream& operator<<( std::ostream& os, const Sphere& s );
std::ostream& operator<<( std::ostream& os, ConstSphereID s );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC COUNT FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Sphere,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<Sphere>( RigidBody *const * first,
                                        RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Sphere,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<const Sphere>( RigidBody *const * first,
                                              RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Sphere,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<Sphere>( const RigidBody *const * first,
                                        const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Sphere,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<const Sphere>( const RigidBody *const * first,
                                              const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Sphere,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<Sphere>( GeomPrimitive *const * first,
                                        GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Sphere,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<const Sphere>( GeomPrimitive *const * first,
                                              GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Sphere,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<Sphere>( const GeomPrimitive *const * first,
                                        const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to spheres in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of spheres.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Sphere,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to spheres.
 */
template<>
inline size_t polymorphicCount<const Sphere>( const GeomPrimitive *const * first,
                                              const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == sphereType ) ++count;
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
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Sphere,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a sphere.
 */
template<>
inline RigidBody *const * polymorphicFind<Sphere>( RigidBody *const * first,
                                                   RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, RigidBody );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Sphere,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a sphere.
 */
template<>
inline RigidBody *const * polymorphicFind<const Sphere>( RigidBody *const * first,
                                                         RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, RigidBody );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Sphere,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a sphere.
 */
template<>
inline const RigidBody *const * polymorphicFind<Sphere>( const RigidBody *const * first,
                                                         const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, const RigidBody );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Sphere,const RigidBody). The function traverses the range \f$ [first,last) \f$ of
 * pointers to rigid bodies until it finds the next pointer to a sphere.
 */
template<>
inline const RigidBody *const * polymorphicFind<const Sphere>( const RigidBody *const * first,
                                                               const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, const RigidBody );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Sphere,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives until it finds the next pointer to a sphere.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<Sphere>( GeomPrimitive *const * first,
                                                       GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, GeomPrimitive );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Sphere,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a sphere.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<const Sphere>( GeomPrimitive *const * first,
                                                             GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, GeomPrimitive );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Sphere,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a sphere.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<Sphere>( const GeomPrimitive *const * first,
                                                             const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Sphere, const GeomPrimitive );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a sphere in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a sphere.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Sphere,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives until it finds the next pointer to a sphere.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<const Sphere>( const GeomPrimitive *const * first,
                                                                   const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Sphere, const GeomPrimitive );

   while( first != last && (*first)->getType() != sphereType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
