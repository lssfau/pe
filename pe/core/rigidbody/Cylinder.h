//=================================================================================================
/*!
 *  \file pe/core/rigidbody/Cylinder.h
 *  \brief Header file for the Cylinder class
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

#ifndef _PE_CORE_RIGIDBODY_CYLINDER_H_
#define _PE_CORE_RIGIDBODY_CYLINDER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <iosfwd>
#include <pe/core/Configuration.h>
#include <pe/core/rigidbody/CylinderTrait.h>
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
/*!\defgroup cylinder Cylinder
 * \ingroup geometric_primitive
 *
 * The cylinder module combines all necessary functionality for the geometric primitive Cylinder.
 * A detailed description of the cylinder primitive can be found with the class Cylinder. This
 * description also containes examples for the setup and destruction of a cylinder.
 */
/*!\brief Cylinder geometry.
 * \ingroup cylinder
 *
 * \section cylinder_general General
 *
 * The Cylinder class represents the geometric primitive cylinder, which is one of the basic
 * geometric primitives of the \b pe physics engine. The class is derived from the GeomPrimitive
 * base class, which makes the cylinder both a geometric primitive and a rigid body.\n
 * In order to setup a cylinder, it is only necessary to specify its radius (the thickness) and
 * its length. A cylinder is created axis-aligned with the x-axis (the length of the cylinder is
 * parallel to the x-axis).
 *
 * \image html cylinder.png
 * \image latex cylinder.eps "Cylinder geometry" width=200pt
 *
 *
 * \section cylinder_setup Creating and destroying a cylinder primitive
 *
 * In order to create a cylinder primitive, one of the following cylinder creation functions can
 * be used:
 *
 * - pe::createCylinder( id_t uid, real x, real y, real z, real radius, real length, MaterialID material, bool visible )
 * - pe::createCylinder( id_t uid, const Vec3 &gpos, real radius, real length, MaterialID material, bool visible )
 *
 * In order to destroy a specific cylinder primitive (which can also be contained in a Union), the
 * following function can be used:
 *
 * - pe::destroy( BodyID body )
 *
 * The following example demonstrates the creation and destruction of a cylinder primitive:

   \code
   // Creates the iron cylinder 1 at the global position ( 4.2, 3.7, -0.6 ) with radius 1.6
   // and length 6.4. Per default the cylinder is visible in all visualizations. Note that
   // the cylinder is automatically added to the simulation world and is immediately part
   // of the entire simulation. The function returns a handle to the newly created cylinder,
   // which can be used to for instance rotate the cylinder around the global y-axis.
   CylinderID cylinder = createCylinder( 1, 4.2, 3.7, -0.6, 1.6, 6.4, iron );
   cylinder->rotate( 0.0, PI/3.0, 0.0 );

   // Rigid body simulation
   ...

   // Destroying the iron cylinder
   destroy( cylinder );
   \endcode

 * It is possible to add cylinder primitives to a union compound geometry (for more details see
 * the Union class description). In case the cylinder is created inside a pe::pe_CREATE_UNION
 * section, the cylinder is automatically added to the newly created union:

   \code
   CylinderID cylinder;

   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron cylinder 2 with radius 1.3 and length 2.4 at the global position
      // (-1,4,-5). Since the cylinder is created inside a pe_CREATE_UNION section, the cylinder
      // is directly added to the union 'newunion' and is henceforth considered to be part
      // of the union.
      cylinder = createCylinder( 2, -1.0, 4.0, -5.0, 1.3, 2.4, iron );
      ...
   }

   // Destroying the cylinder primitive (NOT the entire union)
   destroy( cylinder );
   \endcode

 * In case of a MPI parallel simulation, cylinders may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
class Cylinder : public CylinderTrait<Config>
{
protected:
   //**Type definitions****************************************************************************
   typedef CylinderTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Cylinder( id_t sid, id_t uid, const Vec3& gpos, real radius,
                      real length, MaterialID material, bool visible );
   explicit Cylinder( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                      real radius, real length, MaterialID material, bool visible, bool fixed );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Cylinder();
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

          real getRelDepth   ( real px, real py, real pz ) const;
          real getRelDepth   ( const Vec3& rpos )          const;
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
   virtual void update( const Vec3& dp );  // Translation update of a subordinate cylinder
   virtual void update( const Quat& dq );  // Rotation update of a subordinate cylinder
   //@}
   //**********************************************************************************************

private:
   //**Sphere setup functions**********************************************************************
   /*! \cond PE_INTERNAL */
   friend CylinderID createCylinder( id_t uid, const Vec3& gpos, real radius,
                                     real length, MaterialID material, bool visible );
   friend CylinderID instantiateCylinder( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
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
/*!\brief Calculates the depth of a point in global coordinates.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return Depth of the global point.
 *
 * Returns a positive value, if the point lies inside the cylinder and a negative value,
 * if the point lies outside the cylinder.
 */
inline real Cylinder::getDepth( real px, real py, real pz ) const
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
 * Returns a positive value, if the point lies inside the cylinder and a negative value,
 * if the point lies outside the cylinder.
 */
inline real Cylinder::getDepth( const Vec3& gpos ) const
{
   return getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the cylinder's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the cylinder and a negative value,
 * if the point lies inside the cylinder.
 */
inline real Cylinder::getRelDistance( real px, real py, real pz ) const
{
   return -getRelDepth( px, py, pz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the cylinder's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the cylinder and a negative value,
 * if the point lies inside the cylinder.
 */
inline real Cylinder::getRelDistance( const Vec3& rpos ) const
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
 * Returns a positive value, if the point lies outside the cylinder and a negative value,
 * if the point lies inside the cylinder.
 */
inline real Cylinder::getDistance( real px, real py, real pz ) const
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
 * Returns a positive value, if the point lies outside the cylinder and a negative value,
 * if the point lies inside the cylinder.
 */
inline real Cylinder::getDistance( const Vec3& gpos ) const
{
   return -getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  CYLINDER SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Cylinder setup functions */
//@{
inline CylinderID createCylinder( id_t uid, real x, real y, real z, real radius,
                                  real length, MaterialID material, bool visible=true );
       CylinderID createCylinder( id_t uid, const Vec3& gpos, real radius,
                                  real length, MaterialID material, bool visible=true );
       CylinderID instantiateCylinder( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                       const Quat& q, real radius, real length, MaterialID material,
                                       bool visible, bool fixed, bool reg=true );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new cylinder.
 * \ingroup cylinder
 *
 * \param uid The user-specific ID of the cylinder.
 * \param x The global x-position of the center of the cylinder.
 * \param y The global y-position of the center of the cylinder.
 * \param z The global z-position of the center of the cylinder.
 * \param radius The radius of the cylinder part and the end caps \f$ (0..\infty) \f$.
 * \param length The length of the cylinder part of the cylinder \f$ (0..\infty) \f$.
 * \param material The material of the cylinder.
 * \param visible Specifies if the cylinder is visible in a visualization.
 * \return Handle for the new cylinder.
 * \exception std::invalid_argument Invalid cylinder radius.
 * \exception std::invalid_argument Invalid cylinder length.
 * \exception std::invalid_argument Invalid global cylinder position.
 *
 * This function creates a cylinder primitive in the \b pe simulation system. The cylinder with
 * user-specific ID \a uid is placed at the global position \a (x,y,z), has the radius \a radius
 * and the length \a length, and consists of the material \a material. The \a visible flag sets
 * the cylinder (in-)visible in all visualizations.
 *
 * \image html cylinder.png
 * \image latex cylinder.eps "Cylinder geometry" width=200pt
 *
 * The following code example illustrates the setup of a cylinder:

   \code
   // Creating the iron cylinder 1 with a radius of 0.9 and a length 2.5 of at the global
   // position (2,3,4). Per default the cylinder is visible in all visualizations. Note
   // that the cylinder is automatically added to the simulation world and is immediately
   // part of the entire simulation. The function returns a handle to the newly created
   // cylinder, which can be used to for instance rotate the cylinder around the global
   // y-axis.
   CylinderID cylinder = createCylinder( 1, 2.0, 3.0, 4.0, 0.9, 2.5, iron );
   cylinder->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the cylinder is created inside a pe::pe_CREATE_UNION section, the cylinder is
 * automatically added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron cylinder 2 with radius 1.3 and length 2.4 at the global position
      // (-1,4,-5). Since the union is created inside a pe_CREATE_UNION section, the cylinder
      // is directly added to the union 'newunion' and is henceforth considered to be part
      // of the union.
      createCylinder( 2, -1.0, 4.0, -5.0, 1.3, 2.4, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, cylinders may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
inline CylinderID createCylinder( id_t uid, real x, real y, real z, real radius,
                                  real length, MaterialID material, bool visible )
{
   return createCylinder( uid, Vec3(x,y,z), radius, length, material, visible );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Cylinder operators */
//@{
std::ostream& operator<<( std::ostream& os, const Cylinder& c );
std::ostream& operator<<( std::ostream& os, ConstCylinderID c );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC COUNT FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Cylinder,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<Cylinder>( RigidBody *const * first,
                                          RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Cylinder,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<const Cylinder>( RigidBody *const * first,
                                                RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Cylinder,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<Cylinder>( const RigidBody *const * first,
                                          const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Cylinder,const RigidBody). The function traverses the range \f$ [first,last) \f$ of
 * pointers to rigid bodies and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<const Cylinder>( const RigidBody *const * first,
                                                const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Cylinder,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<Cylinder>( GeomPrimitive *const * first,
                                          GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Cylinder,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<const Cylinder>( GeomPrimitive *const * first,
                                                GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Cylinder,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<Cylinder>( const GeomPrimitive *const * first,
                                          const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to cylinders in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of cylinders.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Cylinder,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to cylinders.
 */
template<>
inline size_t polymorphicCount<const Cylinder>( const GeomPrimitive *const * first,
                                                const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == cylinderType ) ++count;
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
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Cylinder,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a cylinder.
 */
template<>
inline RigidBody *const * polymorphicFind<Cylinder>( RigidBody *const * first,
                                                     RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, RigidBody );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Cylinder,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a cylinder.
 */
template<>
inline RigidBody *const * polymorphicFind<const Cylinder>( RigidBody *const * first,
                                                          RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, RigidBody );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Cylinder,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a cylinder.
 */
template<>
inline const RigidBody *const * polymorphicFind<Cylinder>( const RigidBody *const * first,
                                                           const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, const RigidBody );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Cylinder,const RigidBody). The function traverses the range \f$ [first,last) \f$ of
 * pointers to rigid bodies until it finds the next pointer to a cylinder.
 */
template<>
inline const RigidBody *const * polymorphicFind<const Cylinder>( const RigidBody *const * first,
                                                                 const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, const RigidBody );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Cylinder,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a cylinder.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<Cylinder>( GeomPrimitive *const * first,
                                                         GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, GeomPrimitive );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Cylinder,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives until it finds the next pointer to a cylinder.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<const Cylinder>( GeomPrimitive *const * first,
                                                               GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, GeomPrimitive );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Cylinder,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives until it finds the next pointer to a cylinder.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<Cylinder>( const GeomPrimitive *const * first,
                                                               const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Cylinder, const GeomPrimitive );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a cylinder in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a cylinder.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Cylinder,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$
 * of pointers to geometric primitives until it finds the next pointer to a cylinder.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<const Cylinder>( const GeomPrimitive *const * first,
                                                                     const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Cylinder, const GeomPrimitive );

   while( first != last && (*first)->getType() != cylinderType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
