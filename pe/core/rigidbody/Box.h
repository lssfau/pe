//=================================================================================================
/*!
 *  \file pe/core/rigidbody/Box.h
 *  \brief Header file for the Box class
 *
 *  Copyright (C) 2009 Klaus Iglberger
 *                2012 Tobias Preclik
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

#ifndef _PE_CORE_RIGIDBODY_BOX_H_
#define _PE_CORE_RIGIDBODY_BOX_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <cmath>
#include <iosfwd>
#include <pe/core/Configuration.h>
#include <pe/core/rigidbody/BoxTrait.h>
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
/*!\defgroup box Box
 * \ingroup geometric_primitive
 *
 * The box module combines all necessary functionality for the geometric primitive Box. A
 * detailed description of the box primitive can be found with the class Box. This description
 * also containes examples for the setup and destruction of a box.
 */
/*!\brief Box geometry.
 * \ingroup box
 *
 * \section box_general General
 *
 * The Box class represents the geometric primitive box, which is one of the basic geometric
 * primitives of the \b pe physics engine. The class is derived from the GeomPrimitive base class,
 * which makes the box both a geometric primitive and a rigid body.\n
 * A box is created axis-aligned with the global coordinate system, where its geometric center is
 * exactly in the middle of the box, the x-side is aligned with the x-axis, the y-side is aligned
 * with the y-axis and the z-side is aligned with the z-axis.
 *
 * \image html box.png
 * \image latex box.eps "Box geometry" width=200pt
 *
 *
 * \section box_setup Creating and destroying a box primitive
 *
 * In order to create a box primitive, one of the following box creation functions can be used:
 *
 * - pe::createBox( id_t uid, real x, real y, real z, real lx, real ly, real lz, MaterialID material, bool visible )
 * - pe::createBox( id_t uid, const Vec3 &gpos, real lx, real ly, real lz, MaterialID material, bool visible )
 * - pe::createBox( id_t uid, real x, real y, real z, const Vec3 &lengths, MaterialID material, bool visible )
 * - pe::createBox( id_t uid, const Vec3 &gpos, const Vec3 &lengths, MaterialID material, bool visible )
 *
 * In order to destroy a specific box primitive (which can also be contained in a Union), the
 * following function can be used:
 *
 * - pe::destroy( BodyID body )
 *
 * The following example demonstrates the creation and destruction of a box primitive:

   \code
   // Creates the iron box 1 at the global position ( 4.2, 3.7, -0.6 ) with side lengths
   // of ( 1.2, 4.8, 0.8 ). Per default the box is visible in all visualizations. Note that
   // the box is automatically added to the simulation world and is immediately part of the
   // entire simulation. The function returns a handle to the newly created box, which can
   // be used to for instance rotate the box around the global y-axis.
   BoxID box = createBox( 1, 4.2, 3.7, -0.6, 1.2, 4.8, 0.8, iron );
   box->rotate( 0.0, PI/3.0, 0.0 );

   // Rigid body simulation
   ...

   // Destroying the iron box
   destroy( box );
   \endcode

 * It is possible to add box primitives to a union compound geometry (for more details see the
 * Union class description). In case the box is created inside a pe::pe_CREATE_UNION section,
 * the box is automatically added to the newly created union:

   \code
   BoxID box;

   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron box 2 with side lengths (2.1,1.3,4.5) at the global position
      // (-1,4,-5). Since the box is created inside a pe_CREATE_UNION section, the box
      // is directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      box = createBox( 2, -1.0, 4.0, -5.0, Vec3( 2.1, 1.3, 4.5 ), iron );
      ...
   }

   // Destroying the box primitive (NOT the entire union)
   destroy( box );
   \endcode

 * In case of a MPI parallel simulation, boxes may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
class Box : public BoxTrait<Config>
{
protected:
   //**Type definitions****************************************************************************
   typedef BoxTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Box( id_t sid, id_t uid, const Vec3& gpos,
                 const Vec3& lengths, MaterialID material, bool visible );
   explicit Box( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                 const Vec3& lengths, MaterialID material, bool visible, bool fixed );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Box();
   //@}
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   struct Parameters : public GeomPrimitive::Parameters {
      Vec3 lengths_;
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
   virtual void update( const Vec3& dp );  // Translational update of a subordinate box
   virtual void update( const Quat& dq );  // Rotational update of a subordinate box
   //@}
   //**********************************************************************************************

private:
   //**Box setup functions*************************************************************************
   /*! \cond PE_INTERNAL */
   friend BoxID createBox( id_t uid, const Vec3& gpos, const Vec3& lengths,
                           MaterialID material, bool visible );
   friend BoxID instantiateBox( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                                const Vec3& lengths, MaterialID material, bool visible, bool fixed, bool reg );
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
 * Returns a positive value, if the point lies inside the box and a negative value, if the point
 * lies outside the box. The returned depth is calculated relative to the closest side of the box.
 */
inline real Box::getDepth( real px, real py, real pz ) const
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
 * Returns a positive value, if the point lies inside the box and a negative value, if the point
 * lies outside the box. The returned depth is calculated relative to the closest side of the box.
 */
inline real Box::getDepth( const Vec3& gpos ) const
{
   return getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the box's frame of reference.
 *
 * \param px The x-component of the relative coordinate.
 * \param py The y-component of the relative coordinate.
 * \param pz The z-component of the relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the box and a negative value, if the
 * point lies inside the box. The returned distance is calculated relative to the closest
 * side of the box.
 */
inline real Box::getRelDistance( real px, real py, real pz ) const
{
   return -getRelDepth( px, py, pz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Calculates the distance of a point relative to the box's frame of reference.
 *
 * \param rpos The relative coordinate.
 * \return Distance of the relative point.
 *
 * Returns a positive value, if the point lies outside the box and a negative value, if the
 * point lies inside the box. The returned distance is calculated relative to the closest
 * side of the box.
 */
inline real Box::getRelDistance( const Vec3& rpos ) const
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
 * Returns a positive value, if the point lies outside the box and a negative value, if the
 * point lies inside the box. The returned distance is calculated relative to the closest
 * side of the box.
 */
inline real Box::getDistance( real px, real py, real pz ) const
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
 * Returns a positive value, if the point lies outside the box and a negative value, if the
 * point lies inside the box. The returned distance is calculated relative to the closest
 * side of the box.
 */
inline real Box::getDistance( const Vec3& gpos ) const
{
   return -getRelDepth( pointFromWFtoBF( gpos ) );
}
//*************************************************************************************************




//=================================================================================================
//
//  BOX SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Box setup functions */
//@{
inline BoxID createBox( id_t uid, real x, real y, real z, real lx, real ly, real lz,
                        MaterialID material, bool visible=true );
inline BoxID createBox( id_t uid, const Vec3& gpos, real lx, real ly, real lz,
                        MaterialID material, bool visible=true );
inline BoxID createBox( id_t uid, real x, real y, real z, const Vec3& lengths,
                        MaterialID material, bool visible=true );
       BoxID createBox( id_t uid, const Vec3& gpos, const Vec3& lengths,
                        MaterialID material, bool visible=true );
       BoxID instantiateBox( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, const Quat& q,
                             const Vec3& lengths, MaterialID material, bool visible, bool fixed, bool reg=true );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new box.
 * \ingroup box
 *
 * \param uid The user-specific ID of the box.
 * \param x The global x-position of the center of the box.
 * \param y The global y-position of the center of the box.
 * \param z The global z-position of the center of the box.
 * \param lx The side length of the box in x-dimension \f$ (0..\infty) \f$.
 * \param ly The side length of the box in y-dimension \f$ (0..\infty) \f$.
 * \param lz The side length of the box in z-dimension \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 * \return Handle for the new box.
 * \exception std::invalid_argument Invalid side length.
 * \exception std::invalid_argument Invalid global box position.
 *
 * This function creates a box primitive in the \b pe simulation system. The new box with the
 * user-specific ID \a uid is placed at the global position \a (x,y,z), has the side lengths
 * \a (lx,ly,lz), and consists of the material \a material. The \a visible flag sets the box
 * (in-)visible in all visualizations.
 *
 * \image html box.png
 * \image latex box.eps "Box geometry" width=200pt
 *
 * The following code example illustrates the setup of a box:

   \code
   // Creating the iron box 1 with the side lengths (3,2,1) at the global position (2,3,4).
   // Per default the box is visible in all visualizations. Note that the box is automatically
   // added to the simulation world and is immediately part of the entire simulation. The
   // function returns a handle to the newly created box, which can be used to for instance
   // rotate the box around the global y-axis.
   BoxID box = createBox( 1, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the box is created inside a pe::pe_CREATE_UNION section, the box is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron box 2 with side lengths (2.1,1.3,4.5) at the global position
      // (-1,4,-5). Since the union is created inside a pe_CREATE_UNION section, the box
      // is directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      createBox( 2, -1.0, 4.0, -5.0, 2.1, 1.3, 4.5, iron );
      ...
   }
   \endcode

 * In case of an MPI parallel simulation, boxes may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
inline BoxID createBox( id_t uid, real x, real y, real z, real lx, real ly, real lz,
                        MaterialID material, bool visible )
{
   return createBox( uid, Vec3(x,y,z), Vec3(lx,ly,lz), material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new box.
 * \ingroup box
 *
 * \param uid The user-specific ID of the box.
 * \param gpos The global position of the center of the box.
 * \param lx The side length of the box in x-dimension \f$ (0..\infty) \f$.
 * \param ly The side length of the box in y-dimension \f$ (0..\infty) \f$.
 * \param lz The side length of the box in z-dimension \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 * \return Handle for the new box.
 * \exception std::invalid_argument Invalid side length.
 * \exception std::invalid_argument Invalid global box position.
 *
 * This function creates a box primitive in the \b pe simulation system. The new box with the
 * user-specific ID \a uid is placed at the global position \a gpos, has the side lengths
 * \a (lx,ly,lz), and consists of the material \a material. The \a visible flag sets the box
 * (in-)visible in all visualizations.
 *
 * \image html box.png
 * \image latex box.eps "Box geometry" width=200pt
 *
 * The following code example illustrates the setup of a box:

   \code
   // Creating the iron box 1 with the side lengths (3,2,1) at the global position (2,3,4).
   // Per default the box is visible in all visualizations. Note that the box is automatically
   // added to the simulation world and is immediately part of the entire simulation. The
   // function returns a handle to the newly created box, which can be used to for instance
   // rotate the box around the global y-axis.
   BoxID box = createBox( 1, Vec3( 2.0, 3.0, 4.0 ), 3.0, 2.0, 1.0, iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the box is created inside a pe::pe_CREATE_UNION section, the box is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron box 2 with side lengths (2.1,1.3,4.5) at the global position
      // (-1,4,-5). Since the union is created inside a pe_CREATE_UNION section, the box
      // is directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      createBox( 2, Vec3( -1.0, 4.0, -5.0 ), 2.1, 1.3, 4.5, iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, boxes may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
inline BoxID createBox( id_t uid, const Vec3& gpos, real lx, real ly, real lz,
                        MaterialID material, bool visible )
{
   return createBox( uid, gpos, Vec3(lx,ly,lz), material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new box.
 * \ingroup box
 *
 * \param uid The user-specific ID of the box.
 * \param x The global x-position of the center of the box.
 * \param y The global y-position of the center of the box.
 * \param z The global z-position of the center of the box.
 * \param lengths The side lengths of the box \f$ (0..\infty) \f$.
 * \param material The material of the box.
 * \param visible Specifies if the box is visible in a visualization.
 * \return Handle for the new box.
 * \exception std::invalid_argument Invalid side length.
 * \exception std::invalid_argument Invalid global box position.
 *
 * This function creates a box primitive in the \b pe simulation system. The new box with the
 * user-specific ID \a uid is placed at the global position \a (x,y,z), has the side lengths
 * \a lengths, and consists of the material \a material. The \a visible flag sets the box
 * (in-)visible in all visualizations.
 *
 * \image html box.png
 * \image latex box.eps "Box geometry" width=200pt
 *
 * The following code example illustrates the setup of a box:

   \code
   // Creating the iron box 1 with the side lengths (3,2,1) at the global position (2,3,4).
   // Per default the box is visible in all visualizations. Note that the box is automatically
   // added to the simulation world and is immediately part of the entire simulation. The
   // function returns a handle to the newly created box, which can be used to for instance
   // rotate the box around the global y-axis.
   BoxID box = createBox( 1, 2.0, 3.0, 4.0, Vec3( 3.0, 2.0, 1.0 ), iron );
   box->rotate( 0.0, PI/3.0, 0.0 );
   \endcode

 * In case the box is created inside a pe::pe_CREATE_UNION section, the box is automatically
 * added to the newly created union:

   \code
   pe_CREATE_UNION( newunion, 1 )
   {
      ...
      // Creating the iron box 2 with side lengths (2.1,1.3,4.5) at the global position
      // (-1,4,-5). Since the union is created inside a pe_CREATE_UNION section, the box
      // is directly added to the union 'newunion' and is henceforth considered to be
      // part of the union.
      createBox( 2, -1.0, 4.0, -5.0, Vec3( 2.1, 1.3, 4.5 ), iron );
      ...
   }
   \endcode

 * In case of a MPI parallel simulation, boxes may only be created if their global position
 * lies inside the domain of the local MPI process. Only in case they are created inside a
 * pe::pe_CREATE_UNION section, this rule is relaxed to the extend that only the final center
 * of mass of the resulting union must be inside the domain of the local process.
 */
inline BoxID createBox( id_t uid, real x, real y, real z, const Vec3& lengths,
                        MaterialID material, bool visible )
{
   return createBox( uid, Vec3(x,y,z), lengths, material, visible );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Box operators */
//@{
std::ostream& operator<<( std::ostream& os, const Box& b );
std::ostream& operator<<( std::ostream& os, ConstBoxID b );
//@}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC COUNT FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxess in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Box,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to rigid
 * bodies and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<Box>( RigidBody *const * first,
                                     RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Box,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<const Box>( RigidBody *const * first,
                                           RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Box,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<Box>( const RigidBody *const * first,
                                     const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Box,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<const Box>( const RigidBody *const * first,
                                           const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Box,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<Box>( GeomPrimitive *const * first,
                                     GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Box,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<const Box>( GeomPrimitive *const * first,
                                           GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, GeomPrimitive );

   size_t count( 0 );
   for( GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Box,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<Box>( const GeomPrimitive *const * first,
                                     const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to boxes in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of boxes.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Box,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives and counts all pointers to boxes.
 */
template<>
inline size_t polymorphicCount<const Box>( const GeomPrimitive *const * first,
                                           const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, const GeomPrimitive );

   size_t count( 0 );
   for( const GeomPrimitive *const * it=first; it!=last; ++it )
      if( (*it)->getType() == boxType ) ++count;
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
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Box,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a box.
 */
template<>
inline RigidBody *const * polymorphicFind<Box>( RigidBody *const * first,
                                                RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, RigidBody );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Box,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a box.
 */
template<>
inline RigidBody *const * polymorphicFind<const Box>( RigidBody *const * first,
                                                      RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, RigidBody );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Box,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a box.
 */
template<>
inline const RigidBody *const * polymorphicFind<Box>( const RigidBody *const * first,
                                                      const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, const RigidBody );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Box,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a box.
 */
template<>
inline const RigidBody *const * polymorphicFind<const Box>( const RigidBody *const * first,
                                                            const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, const RigidBody );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Box,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers to
 * geometric primitives until it finds the next pointer to a box.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<Box>( GeomPrimitive *const * first,
                                                    GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, GeomPrimitive );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Box,GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a box.
 */
template<>
inline GeomPrimitive *const * polymorphicFind<const Box>( GeomPrimitive *const * first,
                                                          GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, GeomPrimitive );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Box,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of pointers
 * to geometric primitives until it finds the next pointer to a box.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<Box>( const GeomPrimitive *const * first,
                                                          const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Box, const GeomPrimitive );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a box in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a box.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Box,const GeomPrimitive). The function traverses the range \f$ [first,last) \f$ of
 * pointers to geometric primitives until it finds the next pointer to a box.
 */
template<>
inline const GeomPrimitive *const * polymorphicFind<const Box>( const GeomPrimitive *const * first,
                                                                const GeomPrimitive *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Box, const GeomPrimitive );

   while( first != last && (*first)->getType() != boxType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
