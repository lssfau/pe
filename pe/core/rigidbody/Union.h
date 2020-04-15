//=================================================================================================
/*!
 *  \file pe/core/rigidbody/Union.h
 *  \brief Header file for the Union class
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

#ifndef _PE_CORE_RIGIDBODY_UNION_H_
#define _PE_CORE_RIGIDBODY_UNION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/rigidbody/Box.h>
#include <pe/core/rigidbody/Capsule.h>
#include <pe/core/Configuration.h>
#include <pe/core/rigidbody/Cylinder.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/Link.h>
#include <pe/core/rigidbody/Plane.h>
#include <pe/core/rigidbody/UnionTrait.h>
#include <pe/core/rigidbody/Sphere.h>
#include <pe/core/Types.h>
#include <pe/math/Vector3.h>
#include <pe/math/Vector6.h>
#include <pe/system/Precision.h>
#include <pe/util/Algorithm.h>
#include <pe/util/Types.h>
#include <pe/core/rigidbody/TriangleMesh.h>
#include <pe/povray.h>

namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup union Union
 * \ingroup rigid_body
 *
 * The union module combines all necessary functionality for the Union class. A detailed
 * description of the union can be found with the class Union. This description also containes
 * examples for the setup and destruction of a union.
 */
/*!\brief Compound geometry.
 * \ingroup union
 *
 * \section union_general General
 *
 * A union is a compound of an arbitrary number of rigid bodies (as for instance spheres,
 * boxes, capsules, planes, and other unions). This compound represents a single rigid body
 * with its own global position, mass, velocity, and orientation. All bodies within the union
 * are considered subordinate bodies. Their global position, linear, and angular velocity are
 * determined by their relative position within the body frame of the superordinate union.
 * However, each contained body has its own global orientation/rotation and therefore the
 * contained bodies can also be rotated as part of a union.
 *
 * \image html union1.png
 * \image latex union1.eps "Union geometry" width=600pt
 *
 *
 * \section union_empty Empty unions
 *
 * It is possible to have empty unions (i.e. unions with no contained rigid bodies). Empty
 * unions can be seen as massless bodies without any volume. Nevertheless, empty unions have
 * a global position and a linear and angular velocity. So it is possible to reposition an
 * empty union or to assign velocities to it. Note however, that as soon as a new finite
 * rigid body is assigned to the union, the center of mass is recalculated according to the
 * contained rigid bodies. Also note, that in case the last body of a union is destroyed or
 * added to a different body manager, the emptied union keeps the last global position and
 * velocities.\n\n
 *
 *
 * \section union_infinite Infinite unions
 *
 * As soon as an infinite rigid body (e.g. a plane) is part of the union, the union itself is
 * considered to be an infinite rigid body with an infinite mass and an infinite moment of
 * inertia. The global position of an infinite union is treated like an anchor point (similar
 * to the anchor point of a plane) and does not correspond to the center of mass anymore (i.e.
 * an infinite union has no center of mass). The anchor point is calculated as the average of
 * the anchor points of all contained infinite rigid bodies and acts both as reference point
 * for the contained rigid bodies as well as rotation point for any rotations performed by the
 * setOrientation() and rotate() functions.\n\n
 *
 *
 * \section union_setup Creating and destroying a union
 *
 * In order to create and destroy a union the following functions can be used:
 *
 * -# pe::createUnion( id_t uid, bool visible );
 * -# pe::createUnion( id_t uid, BodyID body1, bool visible );
 * -# pe::createUnion( id_t uid, BodyID body1, BodyID body2, bool visible );
 * -# pe::createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3, bool visible );
 * -# pe::createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3, BodyID body4, bool visible );
 * -# pe::createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3, BodyID body4, BodyID body5, bool visible );
 * -# pe::destroy( pe::BodyID body )
 *
 * The creation functions return a handle to the new union. The destruction function takes such
 * a handle as only argument and destroys the given union.\n\n
 *
 *
 * \section union_setup Setup of a union
 *
 * \image html union2.png
 * \image latex union2.eps "Union geometry" width=450pt
 *
 * Per default all newly created rigid bodies are contained in the simulation world. In order
 * to create compounds of rigid bodies, there exist several ways to add rigid bodies to a union.
 * The first way is to directly add bodies to a union during the creation of the union via the
 * createUnion() functions:

   \code
   // Creating a sphere and a capsule primitive. Per default both primitives are created in the
   // simulation world.
   SphereID  sphere1  = createSphere ( 1, 2.0, -4.0, 3.0, 1.0, fir );
   CapsuleID capsule2 = createCapsule( 2, 5.0, -4.0, 2.0, 1.0, 2.0, fir );

   // Creating a union containing the two primitives. From now on, both primitives are part of
   // the union. Any translation or rotation of the union will also translate/rotate the two
   // contained bodies. Additionally, if the union is destroyed, so are the two primitives.
   UnionID union1 = createUnion( 1, sphere1, capsule2 );
   \endcode

 * The second way to add rigid bodies to a union compound is via one of the following add()
 * functions:
 *
 * -# pe::Union::add( BodyID body );
 * -# pe::Union::add( IteratorType first, IteratorType last );
 *
 * Example:

   \code
   // Creating an empty union
   UnionID union2 = createUnion( 2 );

   // Creating a single, iron sphere. Per default it is created in the simulation world.
   SphereID sphere2 = createSphere( 2, 0.0, 0.0, 0.0, 1.0, iron );

   // Adding the sphere to the union. From now on, the sphere is part of the union. Translations/
   // rotations also affect the sphere, and if the union is destroyed, so is the sphere.
   union2->add( sphere2 );
   \endcode

 * The third (and probably most convenient) way to add rigid bodies to a union is via the
 * pe::pe_CREATE_UNION macro. This macro provides an environment to setup and configure a union
 * compound geometry. The following example demonstrates the use of the pe::pe_CREATE_UNION
 * environment:

   \code
   int main( int argc, char** argv )
   {
      // ...

      // Setup of a union compound geometry
      // This command starts a new section for the setup and configuration of a new union
      // object. All rigid bodies created within the following section will be automatically
      // added to the new union. The new union itself is available via the name specified as
      // the first argument of the macro. The second argument specifies the user-specific ID.
      pe_CREATE_UNION( union1, 1 )
      {
         // Creating an iron sphere. The sphere is directly added to the union 1
         SphereID sphere1 = createSphere( 1, 2.0, -3.0, 0.5, 1.0, iron );

         // Creating an oak box at the origin, which is also directly added to union 1.
         // Afterward the box is translated next to sphere 1.
         BoxID box2 = createBox( 2, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, oak );
         box2->translate( 4.0, -3.0, 0.5 );

         // Creating a link between sphere 1 and the box 2
         LinkID link1 = createLink( union1, 1, sphere1, box2 );

         // Rotating the entire union around the y-axis
         union1->rotate( 0.0, PI/3.45, 0.0 );
      }

      // ...
   }
   \endcode

 * Note that in case any of the function calls inside the pe::pe_CREATE_UNION environment
 * results in an exception being thrown, which is not caught inside the section, the entire
 * union is destroyed! Also note that it is also possible to add other unions to a union. This
 * also means that it is possible to nest pe::pe_CREATE_UNION environments!\n
 * In order to remove rigid bodies from a union, it is only necessary to add a body contained
 * in a union to another body manager (as for instance the simulation world). The following
 * example demonstrates this:

   \code
   // Creating an empty union and adding a single iron sphere. From now on, the sphere is part
   // of the union. Translations/rotations also affect the sphere, and if the union is destroyed,
   // so is the sphere.
   UnionID union1 = createUnion( 1 );
   SphereID sphere = createSphere( 2, 0.0, 0.0, 0.0, 1.0, iron );
   union1->add( sphere );

   // Removing the sphere from the union again and transfering it to the simulation world.
   // From now on, the sphere is no longer part of the union, i.e. translations/rotations
   // of the union no longer affect the sphere. Additionally, if the union is destroyed,
   // the sphere will persist as part of the simulation world.
   WorldID world = theWorld();
   world->add( sphere );
   \endcode

 * Note that rigid bodies that are removed from a union will keep their current linear and angular
 * velocity. Also note that removing a rigid body that is removed from a fixed union will also be
 * fixed after the removal operation!
 *
 *
 * \section union_bodies Access to the contained rigid bodies
 *
 * In order to handle and access the bodies contained in the union, the Union class provides
 * several functions:

   \code
   // Size functions
   pe::Union::SizeType total   = union->size();          // Total number of bodies in the union
   pe::Union::SizeType spheres = union->size<Sphere>();  // Number of spheres in the union

   // Direct access to the rigid bodies (non-constant version)
   pe::BodyID body = union->getBody( 0 );  // Direct access to the first rigid body in the union

   // Iterators over all contained rigid bodies (non-constant versions)
   pe::Union::Iterator begin = union->begin();
   pe::Union::Iterator end   = union->end();

   for( ; begin!=end; ++begin )
      ...

   // Iterators over a specific rigid body type (non-constant versions)
   pe::Union::CastIterator<Sphere> begin = union->begin<Sphere>();
   pe::Union::CastIterator<Sphere> end   = union->end<Sphere>();

   for( ; begin!=end; ++begin )
      ...
   \endcode

 * \n \section union_links Access to the contained links
 *
 * A link between two rigid bodies represents a point of interest within a union. During
 * each time step, the forces and torques acting in each specified link are calculated.
 * In order to be able to calculate the forces and torques, the two bodies have to touch
 * each other and the structure of the superordinate union must be separable in two distinct
 * sections that are only connected to each other at the specified link. Links within disjoint
 * unions or links within cyclic structures cannot be handled. Additionally, links in infinite
 * unions (i.e. union containing at least a single infinite rigid body) cannot be handled. In
 * these cases the link is invalidated and no force and torque calculation is performed. Note,
 * that it is also not possible to calculate the forces and torques in nested unions!\n
 * The example shows a union consisting of two spheres. The link between the two spheres can
 * be seen as a reading point for contact forces and torques.
 *
 * \image html link.png
 * \image latex link.eps "Link between two spheres" width=200pt
 *
 * The following example demonstrates the setup of the illustrated link and how the links
 * contained in the union can be accessed:

   \code
   // Setup of a union containing two iron spheres connected by a link
   pe_CREATE_UNION( union1, 1 )
   {
      // Setup of the two iron spheres at the coordinates (-2,3,4) and (0,3,4)
      SphereID sphere1 = createSphere( 1, -2.0, 3.0, 4.0, 1.0, iron );
      SphereID sphere2 = createSphere( 2,  0.0, 3.0, 4.0, 1.0, iron );

      // Setup of the link between the two spheres
      LinkID link = createLink( union1, 1, sphere1, sphere2 );

      // Calculating the number of links contained in the union
      pe::Union::SizeType links = union1->countLinks();  // Number of links in the union

      // Direct access to the links (non-constant version)
      pe::LinkID link = union1->getLink( 0 );  // Direct access to the first link in the union

      // Iterators over all contained links (non-constant versions)
      pe::Union::LinkIterator begin = union1->beginLinks();
      pe::Union::LinkIterator end   = union1->endLinks();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode
 */
class Union : public UnionTrait<Config>
{
protected:
   //**Type definitions****************************************************************************
   typedef UnionTrait<Config>  Parent;  //!< The type of the parent class.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef UnionBase::SizeType           SizeType;           //!< Size type of the union.
   typedef UnionBase::Iterator           Iterator;           //!< Iterator over the contained rigid bodies.
   typedef UnionBase::ConstIterator      ConstIterator;      //!< Constant iterator over the contained rigid bodies.
   typedef UnionBase::LinkIterator       LinkIterator;       //!< Iterator over the contained links.
   typedef UnionBase::ConstLinkIterator  ConstLinkIterator;  //!< Constant iterator over the contained links.
   struct Parameters : public RigidBody::Parameters {
      real m_;
      Mat3 I_;
      Quat q_;
      Vec6 aabb_;

      std::vector<Sphere::Parameters>   spheres_;
      std::vector<Box::Parameters>      boxes_;
      std::vector<Capsule::Parameters>  capsules_;
      std::vector<Cylinder::Parameters> cylinders_;
      std::vector<Union::Parameters>    unions_;
      std::vector<Plane::Parameters>    planes_;
   };
   //**********************************************************************************************

   //**Forward declarations for nested classes*****************************************************
   template< typename C > class CastIterator;
   template< typename C > class ConstCastIterator;
   //**********************************************************************************************

protected:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit Union( id_t sid, id_t uid, bool visible );
   explicit Union( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos, real mass,
                   const Mat3& I, const Quat& q, const Vec6& aabb, bool visible, bool fixed );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~Union();
   //@}
   //**********************************************************************************************

public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
                          inline bool                 isEmpty() const;
                          inline SizeType             size()    const;
   template< typename C > inline SizeType             size()    const;
                          inline BodyID               getBody( size_t index );
                          inline ConstBodyID          getBody( size_t index ) const;

                          inline Iterator             begin();
                          inline ConstIterator        begin() const;
   template< typename C > inline CastIterator<C>      begin();
   template< typename C > inline ConstCastIterator<C> begin() const;

                          inline Iterator             end();
                          inline ConstIterator        end() const;
   template< typename C > inline CastIterator<C>      end();
   template< typename C > inline ConstCastIterator<C> end() const;

                          inline SizeType             countLinks() const;
                          inline LinkID               getLink( size_t index );
                          inline ConstLinkID          getLink( size_t index ) const;
                          inline LinkIterator         beginLinks();
                          inline ConstLinkIterator    beginLinks() const;
                          inline LinkIterator         endLinks();
                          inline ConstLinkIterator    endLinks()   const;
   //@}
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
   /*!\name Translation functions
   // \brief The translation functions displace the center of mass of the entire union. All
   // \brief contained rigid bodies are moved by the same displacement.
   */
   //@{
   virtual void translate( real dx, real dy, real dz );
   virtual void translate( const Vec3& dp );
   //@}
   //**********************************************************************************************

   //**Rotation functions**************************************************************************
   /*!\name Rotation functions
   // \brief The rotation functions rotate the entire union around the union's center of mass,
   // \brief the origin or a specific coordinate. These functions cause all contained bodies
   // \brief to rotate. The orientation of the rigid bodies within the union in reference to
   // \brief the body frame of the union is not changed.
   */
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
   //@}
   //**********************************************************************************************

   //**Destroy functions***************************************************************************
   /*!\name Destroy functions */
   //@{
                          Iterator        destroy( Iterator pos );
   template< typename C > CastIterator<C> destroy( CastIterator<C> pos );
   //@}
   //**********************************************************************************************

   //**Rigid body manager functions****************************************************************
   /*!\name Rigid body manager functions */
   //@{
   virtual void add( BodyID body );

   template< typename IteratorType >
   void add( IteratorType first, IteratorType last );
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   virtual void print( std::ostream& os, const char* tab ) const;
   //@}
   //**********************************************************************************************

protected:
   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   void setInfinite();
   //@}
   //**********************************************************************************************

   //**Fixation functions**************************************************************************
   /*!\name Fixation functions */
   //@{
   virtual void fix  ();
   virtual void unfix();
   //@}
   //**********************************************************************************************

   //**Rigid body manager functions****************************************************************
   /*!\name Rigid body manager functions */
   //@{
   virtual void remove( BodyID  body );
   //@}
   //**********************************************************************************************

   //**Link setup functions************************************************************************
   /*!\name Link setup functions */
   //@{
   LinkID createLink ( id_t id, BodyID b1, BodyID b2 );
   void   destroyLink( LinkID link );
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   virtual void update( const Vec3& dp );  // Translation update of a subordinate union
   virtual void update( const Quat& dq );  // Rotation update of a subordinate union
   //@}
   //**********************************************************************************************

   //**Signal functions***************************************************************************
   /*!\name Signal functions */
   //@{
   virtual void handleModification();
   virtual void handleTranslation();
   virtual void handleRotation();
   virtual void handleFixation();
   //@}
   //**********************************************************************************************

private:
   //**Union setup functions***********************************************************************
   /*! \cond PE_INTERNAL */
   friend UnionID createUnion( id_t uid, bool visible );
   friend UnionID instantiateUnion( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                    real mass, const Mat3& I, const Quat& q, const Vec6& aabb,
                                    bool visible, bool fixed, bool reg );
   /*! \endcond */
   //**********************************************************************************************

   //**Link setup functions************************************************************************
   /*! \cond PE_INTERNAL */
   friend LinkID createLink( UnionID u, id_t id, BodyID b1, BodyID b2 );
   friend void destroy( LinkID link );
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
/*!\brief Returns whether the union contains any bodies or not.
 *
 * \return \a true if the union is empty, \a false if it not.
 */
inline bool Union::isEmpty() const
{
   return bodies_.size() == 0;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rigid bodies contained in the union.
 *
 * \return The number of rigid bodies.
 */
inline Union::SizeType Union::size() const
{
   return bodies_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rigid bodies of type \a C contained in the union.
 *
 * \return The number of bodies of type \a C.
 *
 * This function calculates the total number of rigid bodies of type \a C within the union,
 * where \a C is a rigid body type (Sphere, Box, Capsule, Plane, ...). The attempt to use
 * this function for non-body types results in a compile time error.

   \code
   pe::Union::SizeType total   = union->size();          // Total number of bodies in the union
   pe::Union::SizeType spheres = union->size<Sphere>();  // Number of spheres contained in the union
   \endcode

 * \b Note: The total number of rigid bodies of type \a C is not cached inside the union but
 * is calculated each time the function is called. Using the templated version of size() to
 * calculate the total number of bodies of type \a C is therefore more expensive than using
 * the non-template version of size() to get the total number of bodies in the union!
 */
template< typename C >
inline Union::SizeType Union::size() const
{
   return bodies_.size<C>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the indexed rigid body.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed body.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
inline BodyID Union::getBody( size_t index )
{
   pe_USER_ASSERT( index < bodies_.size(), "Invalid access index" );
   return bodies_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant handle to the indexed rigid body.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Constant handle to the accessed body.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
inline ConstBodyID Union::getBody( size_t index ) const
{
   pe_USER_ASSERT( index < bodies_.size(), "Invalid access index" );
   return bodies_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first body of the union.
 *
 * \return Iterator to the first body of the union.
 */
inline Union::Iterator Union::begin()
{
   return bodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first body of the union.
 *
 * \return Iterator to the first body of the union.
 */
inline Union::ConstIterator Union::begin() const
{
   return bodies_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first body of type \a C within the union.
 *
 * \return Iterator to the first body of type \a C.
 *
 * This function returns an iterator to the first rigid body of type \a C within the union,
 * where \a C is a rigid body type (Sphere, Box, Capsule, Plane, ...). In case there are no
 * bodies of type \a C contained in the union, an iterator just past the last body contained
 * in the union is returned. In combination with the according end function (see example),
 * this iterator allows to iterate over all rigid bodies of type \a C contained in the union.
 * The attempt to use this function for non-body types results in a compile time error.

   \code
   // Definition of function for non-constant unions
   void f( pe::UnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::CastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::CastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant unions
   void f( pe::ConstUnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::ConstCastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::ConstCastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all bodies of type
 * \a C contained in the union is more expensive than using the non-template version to iterate
 * over all contained rigid bodies. Use this function only if you require a type-specific member
 * of type \a C (e.g. Sphere::getRadius()).
 */
template< typename C >
inline Union::CastIterator<C> Union::begin()
{
   return bodies_.begin<C>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first body of type \a C within the union.
 *
 * \return Iterator to the first body of type \a C.
 *
 * This function returns an iterator to the first rigid body of type \a C within the union,
 * where \a C is a rigid body type (Sphere, Box, Capsule, Plane, ...). In case there are no
 * bodies of type \a C contained in the union, an iterator just past the last body contained
 * in the union is returned. In combination with the according end function (see example),
 * this iterator allows to iterate over all rigid bodies of type \a C contained in the union.
 * The attempt to use this function for non-body types results in a compile time error.

   \code
   // Definition of function for non-constant unions
   void f( pe::UnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::CastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::CastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant unions
   void f( pe::ConstUnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::ConstCastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::ConstCastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all bodies of type
 * \a C contained in the union is more expensive than using the non-template version to iterate
 * over all contained rigid bodies. Use this function only if you require a type-specific member
 * of type \a C (e.g. Sphere::getRadius()).
 */
template< typename C >
inline Union::ConstCastIterator<C> Union::begin() const
{
   return bodies_.begin<C>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last body of the union.
 *
 * \return Iterator just past the last body of the union.
 */
inline Union::Iterator Union::end()
{
   return bodies_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last body of the union.
 *
 * \return Iterator just past the last body of the union.
 */
inline Union::ConstIterator Union::end() const
{
   return bodies_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last body of the union.
 *
 * \return Iterator just past the last body of the union.
 *
 * This function returns an iterator just past the last rigid body of the union. In combination
 * with the according begin function (see example), this iterator allows to iterate over all
 * bodies of type \a C contained in the union. The attempt to use this function for non-body
 * types results in a compile time error.

   \code
   // Definition of function for non-constant unions
   void f( UnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::CastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::CastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant unions
   void f( ConstUnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::ConstCastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::ConstCastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all rigid bodies of
 * type \a C contained in the union is more expensive than using the non-template version to
 * iterate over all contained bodies. Use this function only if you require a type-specific
 * member of type \a C (e.g. Sphere::getRadius()).
 */
template< typename C >
inline Union::CastIterator<C> Union::end()
{
   return bodies_.end<C>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last body of the union.
 *
 * \return Iterator just past the last body of the union.
 *
 * This function returns an iterator just past the last rigid body of the union. In combination
 * with the according begin function (see example), this iterator allows to iterate over all
 * bodies of type \a C contained in the union. The attempt to use this function for non-body
 * types results in a compile time error.

   \code
   // Definition of function for non-constant unions
   void f( UnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::CastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::CastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant unions
   void f( ConstUnionID union )
   {
      // Loop over all spheres contained in the union
      pe::Union::ConstCastIterator<Sphere> begin = union->begin<Sphere>();
      pe::Union::ConstCastIterator<Sphere> end   = union->begin<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all rigid bodies of
 * type \a C contained in the union is more expensive than using the non-template version to
 * iterate over all contained bodies. Use this function only if you require a type-specific
 * member of type \a C (e.g. Sphere::getRadius()).
 */
template< typename C >
inline Union::ConstCastIterator<C> Union::end() const
{
   return bodies_.end<C>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of contained links.
 *
 * \return The number of contained links.
 */
inline Union::SizeType Union::countLinks() const
{
   return links_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the indexed link.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed link.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
inline LinkID Union::getLink( size_t index )
{
   pe_USER_ASSERT( index < links_.size(), "Invalid link index" );
   return links_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant handle to the indexed link.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Constant handle to the accessed link.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
inline ConstLinkID Union::getLink( size_t index ) const
{
   pe_USER_ASSERT( index < links_.size(), "Invalid link index" );
   return links_[index];
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the beginning of the contained links.
 *
 * \return Iterator to the beginning of the contained links.
 */
inline Union::LinkIterator Union::beginLinks()
{
   return links_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator to the beginning of the contained links.
 *
 * \return Constant iterator to the beginning of the contained links.
 */
inline Union::ConstLinkIterator Union::beginLinks() const
{
   return links_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the end of the contained links.
 *
 * \return Iterator to the end of the contained links.
 */
inline Union::LinkIterator Union::endLinks()
{
   return links_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant iterator to the end of the contained links.
 *
 * \return Constant iterator to the end of the contained links.
 */
inline Union::ConstLinkIterator Union::endLinks() const
{
   return links_.end();
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTROY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destroying a rigid body contained in the union.
 *
 * \param pos Iterator to the rigid body to be destroyed.
 * \return Iterator to the body after the destroyed body.
 *
 * This function destroys a rigid body contained in the union and updates all union properties
 * that change due to the destroyed rigid body: the center of mass, the relative position of
 * all bodies, the attached sections of all contained link, the moment of intertia and the
 * axis-aligned bounding box. Note that destroying the rigid body invalidates all remaining
 * references/IDs to the body!
 */
template< typename C >
typename Union::CastIterator<C> Union::destroy( CastIterator<C> pos )
{
   // Checking the validity of the iterator
   pe_INTERNAL_ASSERT( pos->getManager() == ManagerID( this ), "Invalid body iterator" );

   // Destroying and deregistering the rigid body
   destroyBody( *pos );
   pos = bodies_.erase( pos );

   // Setting the finite flag
   finite_ = true;
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b ) {
      if( !b->isFinite() ) {
         finite_ = false;
         break;
      }
   }

   // Setting the union's total mass and center of mass
   calcCenterOfMass();

   // Setting the contained primitives' relative position in reference to the center of mass
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateRelPosition( *b );

   // Updating the sections of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   // Setting the axis-aligned bounding box
   Union::calcBoundingBox();

   // Signaling the internal modification to the superordinate body
   signalModification();

   return pos;
}
//*************************************************************************************************




//=================================================================================================
//
//  RIGID BODY MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a range of rigid bodies to the union.
 *
 * \param first Iterator to the first body of the rigid body range.
 * \param last Iterator to the body one past the last body of the rigid body range.
 * \return void
 * \exception std::logic_error Invalid adding into a global union inside an exclusive section.
 * \exception std::logic_error Global flags of body and union do not match.
 *
 * This function adds a range of rigid bodies to the union. It updates all union properties
 * that change due to the new rigid bodies: the center of mass, the relative position of all
 * contained rigid bodies, the attached sections of all contained links, the moment of inertia,
 * and the axis-aligned bounding box.\n
 * The union takes full responsibility for the newly added bodies, including the necessary
 * memory management. After adding the bodies to the union, the bodies are considered part of
 * the union. All functions called on the union (as for instance all kinds of set functions,
 * translation or rotation functions) additionally affect the rigid bodies. However, the bodies
 * can still individually be positioned, oriented, translated, rotated, or made (in-)visible
 * within the union.\n\n
 *
 *
 * \section union_add_infinite Adding infinite rigid bodies
 *
 * Adding an infinite rigid body (as for instance a plane) to the union makes the union an
 * infinite rigid body. This additionally resets the linear and angular velocity of the union
 * and fixes its global position. Note that removing the last infinite body from an union will
 * not restore previous settings such as the velocities and will not unfix the union!\n\n
 *
 *
 * \section union_add_global Global bodies/unions
 *
 * Adding a global rigid body (i.e. a body defined inside the pe_GLOBAL_SECTION) to the union
 * requires the union to be also global. Adding a non-global rigid body to a union requires
 * the union to be also non-global. The attempt to add a global rigid body to a non-global
 * union or a non-global body to a global union results in a \a std::logic_error exception.
 *
 *
 * \section union_add_rules Additional rules
 *
 * The following rules apply for the mobility and visibility of the resulting union:
 *  - If either the union or the added rigid body is fixed, the new compound will also be fixed.
 *    For instance, adding a fixed rigid body to an union will fix the union, and adding a rigid
 *    body to a fixed union will fix the body.
 *  - Neither the (in-)visibility of the added rigid body nor (in-)visibility of the union will
 *    change due to the add operation. For instance adding a visible rigid body to an invisible
 *    union will not change the visibility of the body. Neither is the visibility of the union
 *    changed. In order to change the visiblity, the setVisible() function can be either called
 *    individually for the rigid body (to exclusively make the body (in-)visible) or the entire
 *    union (to make the entire union (in-)visible.
 */
template< typename IteratorType >
void Union::add( IteratorType first, IteratorType last )
{
   // Checking whether the bodies are added to a global union inside an exclusive section
   if( global_ && ExclusiveSection::isActive() )
      throw std::logic_error( "Invalid adding into a global union inside an exclusive section" );

   // Checking the global flags of the bodies and the union in MPI parallel simulations
   for( IteratorType it=first; it!=last; ++it ) {
      if( (*first)->isGlobal() ^ global_ )
         throw std::logic_error( "Global flags of body and union do not match" );
   }

   // Adding the rigid bodies to the union
   for( ; first!=last; ++first )
   {
      BodyID body( *first );

      pe_INTERNAL_ASSERT( body->hasManager(), "Rigid body has no body manager" );

      // Checking for "self-assignment" and the body manager
      if( body == BodyID( this ) || body->getManager() == ManagerID( this ) ) continue;

      // Deregistering the rigid body from the old body manager
      body->getManager()->remove( body );

      // Registering the union as new body manager
      setManager( body );

      // Registering the rigid body
      setSuperBody( body );
      registerSuperBody( body );
      bodies_.pushBack( body );

      // Setting the finiteness of the union
      if( !body->isFinite() ) setInfinite();

      // Setting the finite and fixed flag of the union
      if( !body->isFinite() ) setInfinite();
      else if( !global_ && body->isFixed() ) setFixed( true );
   }

   // Setting the union's total mass and center of mass
   calcCenterOfMass();

   // Setting the contained primitives' relative position in reference to the center of mass
   for( Iterator b=bodies_.begin(); b!=bodies_.end(); ++b )
      updateRelPosition( *b );

   // Updating the sections of the contained links
   for( LinkIterator l=links_.begin(); l!=links_.end(); ++l )
      l->setupLink( bodies_ );

   // Setting the moment of inertia
   calcInertia();

   // Updating the axis-aligned bounding box
   Union::calcBoundingBox();

   // Signaling the internal modification to the superordinate body
   signalModification();
}
//*************************************************************************************************




//=================================================================================================
//
//  UNION SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Union setup functions */
//@{
// Creating standard unions
PE_PUBLIC UnionID createUnion( id_t uid, bool visible=true );
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body, bool visible=true );
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body1, BodyID body2, bool visible=true );
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3,
                               bool visible=true );
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3,
                               BodyID body4, bool visible=true );
PE_PUBLIC UnionID createUnion( id_t uid, BodyID body1, BodyID body2, BodyID body3,
                               BodyID body4, BodyID body5, bool visible=true );
PE_PUBLIC UnionID instantiateUnion( id_t sid, id_t uid, const Vec3& gpos, const Vec3& rpos,
                                    real mass, const Mat3& I, const Quat& q, const Vec6& aabb,
                                    bool visible, bool fixed, bool reg=true );
          UnionID instantiateUnion( const Union::Parameters& objparam, const Vec3& offset, bool remote, bool reg=true );

// Creating tetra sphere unions
inline    UnionID createTetrasphere( id_t uid, real x, real y, real z, real radius,
                                  MaterialID material, bool visible=true );
PE_PUBLIC UnionID createTetrasphere( id_t uid, const Vec3& gpos, real radius,
                                  MaterialID material, bool visible=true );

// Creating granular particle unions
inline    UnionID createGranularParticle( id_t uid, real x, real y, real z, real radius,
                                       MaterialID material, bool visible=true );
PE_PUBLIC UnionID createGranularParticle( id_t uid, const Vec3& gpos, real radius,
                                       MaterialID material, bool visible=true );

// Creating particle agglomerates
PE_PUBLIC UnionID createAgglomerate( id_t uid, const Vec3& gpos, real radius, MaterialID material,
                           size_t number, real threshold, bool visible=true );

// Creating an y-shaped (tristar) union
inline    UnionID createTristar( id_t uid, real x, real y, real z, real radius, real length,
                              MaterialID material, bool visible=true );
PE_PUBLIC UnionID createTristar( id_t uid, const Vec3& gpos, real radius, real length,
                              MaterialID material, bool visible=true );

// Creating a chain link
inline    UnionID createChainLink( id_t uid, real x, real y, real z, real radius, real length,
                                real width, MaterialID material, bool visible=true );
PE_PUBLIC UnionID createChainLink( id_t uid, const Vec3& gpos, real radius, real length,
                                real width, MaterialID material, bool visible=true );

// Creating a dummy
inline    UnionID createDummy( id_t uid, real x, real y, real z, real size,
                            MaterialID material, bool visible=true );
PE_PUBLIC UnionID createDummy( id_t uid, const Vec3& gpos, real size,
                            MaterialID material, bool visible=true );

// Creating a stella octangula
inline    UnionID createStellaOctangula( id_t uid, real x, real y, real z, real radius,
                                         MaterialID material, povray::WriterID pov, bool visible=true );
PE_PUBLIC UnionID createStellaOctangula( id_t uid, const Vec3& gpos, real radius,
                                         MaterialID material, povray::WriterID pov, bool visible=true );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new tetrasphere.
 * \ingroup union
 *
 * \param uid The user-specific ID of the tetrasphere.
 * \param x The global x-position of the center of the tetrasphere.
 * \param y The global y-position of the center of the tetrasphere.
 * \param z The global z-position of the center of the tetrasphere.
 * \param radius Radius of the four spheres of the tetrasphere \f$ (0..\infty) \f$.
 * \param material The material of the tetrasphere.
 * \param visible Specifies if the tetrasphere is visible in a visualization.
 * \return Handle for the new tetrasphere.
 * \exception std::invalid_argument Invalid tetrasphere parameters.
 *
 * \image html tetrasphere.png
 * \image latex tetrasphere.eps "Tetrasphere" width=200pt
 *
 * This function creates a union containing four spheres that are arranged in a tetrahedral
 * fashion. Each sphere will have the same radius and material. \a (x,y,z) specifies the global
 * position of the center of mass of the entire union.
 */
UnionID createTetrasphere( id_t uid, real x, real y, real z, real radius,
                           MaterialID material, bool visible )
{
   return createTetrasphere( uid, Vec3(x,y,z), radius, material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a new granular particle.
 * \ingroup union
 *
 * \param uid The user-specific ID of the particle.
 * \param x The global x-position of the center of the particle.
 * \param y The global y-position of the center of the particle.
 * \param z The global z-position of the center of the particle.
 * \param radius Radius of the granular particle \f$ (0..\infty) \f$.
 * \param material The material of the particle.
 * \param visible Specifies if the particle is visible in a visualization.
 * \return Handle for the new particle.
 * \exception std::invalid_argument Invalid particle parameters.
 *
 * \image html granularparticle.png
 * \image latex granularparticle.eps "Examples for granular particles" width=650pt
 *
 * This function creates a union containing two to four spheres that are arranged in a random
 * fashion. Each sphere will have the same material. \a (x,y,z) specifies the global position
 * of the center of mass of the entire union and \a radius specifies the radius of the granular
 * particle.
 */
inline UnionID createGranularParticle( id_t uid, real x, real y, real z, real radius,
                                       MaterialID material, bool visible )
{
   return createGranularParticle( uid, Vec3(x,y,z), radius, material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a y-shaped (tristar) union consisting of three capsules.
 *
 * \param uid The user-specific ID of the tristar.
 * \param x The global x-position of the center of the tristar.
 * \param y The global y-position of the center of the tristar.
 * \param z The global z-position of the center of the tristar.
 * \param radius Radius of the three capsules of the tristar \f$ (0..\infty) \f$.
 * \param length Length of the three capsules of the tristar \f$ (0..\infty) \f$.
 * \param material The material of the tristar.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new tristar.
 * \exception std::invalid_argument Invalid parameters.
 *
 * \image html tristar.png
 * \image latex tristar.eps "Tristar union" width=200pt
 *
 * This function creates a y-shaped (tristar) union consisting of three capsules. Each capsule
 * will have the same radius, length and material. The given global position (x,y,z) specifies
 * the position of the center of mass of the entire union.
 */
inline UnionID createTristar( id_t uid, real x, real y, real z, real radius, real length,
                              MaterialID material, bool visible )
{
   return createTristar( uid, Vec3(x,y,z), radius, length, material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a chain link consisting of eight capsules
 *
 * \param uid The user-specific ID of the chain link.
 * \param x The global x-position of the center of the chain link.
 * \param y The global y-position of the center of the chain link.
 * \param z The global z-position of the center of the chain link.
 * \param radius Radius of the eight capsules of the chain link \f$ (0..\infty) \f$.
 * \param length Length of the chain link \f$ (0..\infty) \f$.
 * \param width Width of the chain link \f$ (0..\infty) \f$.
 * \param material The material of the chain link.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new chain link.
 * \exception std::invalid_argument Invalid parameters.
 *
 * \image html chainlink.png
 * \image latex chainlink.eps "Chain link union" width=400pt
 *
 * This function creates a chain link consisting of eight capsules. The \a radius parameter
 * specifies the radius of every single capsule and the \a length and \a width parameters
 * specify the length and width of the entire chain link (see the above illustration). The
 * given global position (x,y,z) specifies the position of the center of mass of the chain
 * link.
 */
inline UnionID createChainLink( id_t uid, real x, real y, real z, real radius, real length,
                                real width, MaterialID material, bool visible )
{
   return createChainLink( uid, Vec3(x,y,z), radius, length, width, material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a dummy.
 *
 * \param uid The user-specific ID of the dummy.
 * \param x The global x-position of the center of the dummy.
 * \param y The global y-position of the center of the dummy.
 * \param z The global z-position of the center of the dummy.
 * \param size Size of the dummy \f$ (0..\infty) \f$.
 * \param material The material of the dummy.
 * \param visible Specifies if the entire union is visible in a visualization.
 * \return Handle for the new dummy.
 * \exception std::invalid_argument Invalid dummy size.
 *
 * \image html dummy.png
 * \image latex dummy.eps "Dummy union" width=400pt
 *
 * This function creates a dummy consisting of a single box (the torso), one capsule (the arms)
 * and three spheres (the head and the two legs) as illustrated in the figure above. The \a size
 * parameter specifies the height of the dummy. All other proportions are scaled accordingly.
 * The given global position (x,y,z) specifies the position of the center of mass of the dummy.
 */
inline UnionID createDummy( id_t uid, real x, real y, real z, real size,
                            MaterialID material, bool visible )
{
   return createDummy( uid, Vec3(x,y,z), size, material, visible );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setup of a stella octangula.
 *
 * \param uid The user-specific ID of the dummy.
 * \param x The global x-position of the center of the dummy.
 * \param y The global y-position of the center of the dummy.
 * \param z The global z-position of the center of the dummy.
 * \param radius Radius of the stella octangula \f$ (0..\infty) \f$.
 * \param material The material of the particle.
 * \param visible Specifies if the particle is visible in a visualization.
 * \param pov Handel to the povray writer if setColoredTriangleTexture() should be called for the tetraheadrons. Otherwise NULL
 * \return Handle for the new stella octangula.
 *
 *
 * This function creates a union containing two tetrahedron arranged as an stella octangula,
 * meaning that the tetrahedron are aranged in a way that all tip penetrats a face of the
 * the other tetrahedron in its centre
 */
inline UnionID createStellaOctangula( id_t uid, real x, real y, real z, real radius,
                                      MaterialID material, povray::WriterID pov, bool visible )
{
   return createStellaOctangula( uid, Vec3(x,y,z), radius, material, pov, visible );
}
//*************************************************************************************************





//=================================================================================================
//
//  NESTED CLASS UNION::CASTITERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Dynamic cast iterator for rigid bodies.
 * \ingroup union
 *
 * The CastIterator represents a forward iterator over all rigid bodies of type \a C contained
 * in the union, where \a C is a rigid body type (Sphere, Box, Capsule, Plane, ...).
 *
 * \b Note: Using a CastIterator is computationally more expensive than using a standard iterator
 * over all bodies contained in the union.
 */
template< typename C >
class Union::CastIterator : public Union::Bodies::CastIterator<C>
{
public:
   //**Constructor*********************************************************************************
   inline CastIterator();

   template< typename Other >
   inline CastIterator( const Bodies::CastIterator<Other>& it );

   // No explicitly declared copy constructor.
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the Union::CastIterator class.
 */
template< typename C >
inline Union::CastIterator<C>::CastIterator()
   : Bodies::CastIterator<C>()  // Initializing the base class
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor for the Union::CastIterator class.
 *
 * \param it The base class cast iterator to be copied.
 */
template< typename C >
template< typename Other >
inline Union::CastIterator<C>::CastIterator( const Bodies::CastIterator<Other>& it )
   : Bodies::CastIterator<C>( it )  // Initializing the base class
{}
//*************************************************************************************************




//=================================================================================================
//
//  NESTED CLASS UNION::CONSTCASTITERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Dynamic cast iterator for rigid bodies.
 * \ingroup union
 *
 * The CastIterator represents a forward iterator over all rigid bodies of type \a C contained
 * in the union, where \a C is a rigid body type (Sphere, Box, Capsule, Plane, ...). The
 * ConstCastIterator is the counterpart of CastIterator for constant unions.
 *
 * \b Note: Using a ConstCastIterator is computationally more expensive than using a standard
 * iterator over all bodies contained in the union.
 */
template< typename C >
class Union::ConstCastIterator : public Union::Bodies::ConstCastIterator<C>
{
public:
   //**Constructor*********************************************************************************
   inline ConstCastIterator();

   template< typename Other >
   inline ConstCastIterator( const Bodies::CastIterator<Other>& it );

   template< typename Other >
   inline ConstCastIterator( const Bodies::ConstCastIterator<Other>& it );

   // No explicitly declared copy constructor.
   //**********************************************************************************************
};
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Default constructor for the Union::ConstCastIterator class.
 */
template< typename C >
inline Union::ConstCastIterator<C>::ConstCastIterator()
   : Bodies::ConstCastIterator<C>()  // Initializing the base class
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from Bodies::CastIterator instances.
 *
 * \param it The foreign Bodies::CastIterator instance to be copied.
 */
template< typename C >
template< typename Other >
inline Union::ConstCastIterator<C>::ConstCastIterator( const Bodies::CastIterator<Other>& it )
   : Bodies::ConstCastIterator<C>( it )  // Initializing the base class
{}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Conversion constructor from Bodies::ConstCastIterator instances.
 *
 * \param it The foreign Bodies::ConstCastIterator instance to be copied.
 */
template< typename C >
template< typename Other >
inline Union::ConstCastIterator<C>::ConstCastIterator( const Bodies::ConstCastIterator<Other>& it )
   : Bodies::ConstCastIterator<C>( it )  // Initializing the base class
{}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name Union operators */
//@{
std::ostream& operator<<( std::ostream& os, const Union& u );
std::ostream& operator<<( std::ostream& os, ConstUnionID u );

template< typename L, typename R >
inline bool operator==( const Union::CastIterator<L>& lhs, const Union::CastIterator<R>& rhs );

template< typename L, typename R >
inline bool operator==( const Union::CastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs );

template< typename L, typename R >
inline bool operator==( const Union::ConstCastIterator<L>& lhs, const Union::CastIterator<R>& rhs );

template< typename L, typename R >
inline bool operator==( const Union::ConstCastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs );

template< typename L, typename R >
inline bool operator!=( const Union::CastIterator<L>& lhs, const Union::CastIterator<R>& rhs );

template< typename L, typename R >
inline bool operator!=( const Union::CastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs );

template< typename L, typename R >
inline bool operator!=( const Union::ConstCastIterator<L>& lhs, const Union::CastIterator<R>& rhs );

template< typename L, typename R >
inline bool operator!=( const Union::ConstCastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs );
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two CastIterator objects.
 *
 * \param lhs The left hand side cast iterator.
 * \param rhs The right hand side cast iterator.
 * \return \a true if the iterators point to the same element, \a false if not.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator==( const Union::CastIterator<L>& lhs, const Union::CastIterator<R>& rhs )
{
   return lhs.base() == rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between a CastIterator and a ConstCastIterator.
 *
 * \param lhs The left hand side cast iterator.
 * \param rhs The right hand side constant cast iterator.
 * \return \a true if the iterators point to the same element, \a false if not.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator==( const Union::CastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs )
{
   return lhs.base() == rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between a ConstCastIterator and a CastIterator.
 *
 * \param lhs The left hand side constant cast iterator.
 * \param rhs The right hand side cast iterator.
 * \return \a true if the iterators point to the same element, \a false if not.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator==( const Union::ConstCastIterator<L>& lhs, const Union::CastIterator<R>& rhs )
{
   return lhs.base() == rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Equality comparison between two ConstCastIterator objects.
 *
 * \param lhs The left hand side constant cast iterator.
 * \param rhs The right hand side constant cast iterator.
 * \return \a true if the iterators point to the same element, \a false if not.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator==( const Union::ConstCastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs )
{
   return lhs.base() == rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two CastIterator objects.
 *
 * \param lhs The left hand side cast iterator.
 * \param rhs The right hand side cast iterator.
 * \return \a true if the iterators don't point to the same element, \a false if they do.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator!=( const Union::CastIterator<L>& lhs, const Union::CastIterator<R>& rhs )
{
   return lhs.base() != rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between a CastIterator and a ConstCastIterator.
 *
 * \param lhs The left hand side cast iterator.
 * \param rhs The right hand side constant cast iterator.
 * \return \a true if the iterators don't point to the same element, \a false if they do.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator!=( const Union::CastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs )
{
   return lhs.base() != rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between a ConstCastIterator and a CastIterator.
 *
 * \param lhs The left hand side constant cast iterator.
 * \param rhs The right hand side cast iterator.
 * \return \a true if the iterators don't point to the same element, \a false if they do.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator!=( const Union::ConstCastIterator<L>& lhs, const Union::CastIterator<R>& rhs )
{
   return lhs.base() != rhs.base();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Inequality comparison between two ConstCastIterator objects.
 *
 * \param lhs The left hand side constant cast iterator.
 * \param rhs The right hand side constant cast iterator.
 * \return \a true if the iterators don't point to the same element, \a false if they do.
 *
 * This function is defined to cope with a bug in the Visual C++ compiler.
 */
template< typename L, typename R >
inline bool operator!=( const Union::ConstCastIterator<L>& lhs, const Union::ConstCastIterator<R>& rhs )
{
   return lhs.base() != rhs.base();
}
//*************************************************************************************************




//=================================================================================================
//
//  SPECIALIZATIONS FOR THE POLYMORPHIC COUNT FUNCTION
//
//=================================================================================================

//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to unions in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of unions.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Union,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to rigid
 * bodies and counts all pointers to unions.
 */
template<>
inline size_t polymorphicCount<Union>( RigidBody *const * first,
                                       RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Union, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == unionType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to unions in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of unions.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Union,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to unions.
 */
template<>
inline size_t polymorphicCount<const Union>( RigidBody *const * first,
                                             RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Union, RigidBody );

   size_t count( 0 );
   for( RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == unionType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to unions in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of unions.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (Union,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies and counts all pointers to unions.
 */
template<>
inline size_t polymorphicCount<Union>( const RigidBody *const * first,
                                       const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Union, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == unionType ) ++count;
   return count;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Counts the pointers to unions in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The number of unions.
 *
 * This function is a specialization of the polymorphicCount() function for the type combination
 * (const Union,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies and counts all pointers to unions.
 */
template<>
inline size_t polymorphicCount<const Union>( const RigidBody *const * first,
                                             const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Union, const RigidBody );

   size_t count( 0 );
   for( const RigidBody *const * it=first; it!=last; ++it )
      if( (*it)->getType() == unionType ) ++count;
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
/*!\brief Finds the next pointer to a union in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a union.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Union,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers to
 * rigid bodies until it finds the next pointer to a union.
 */
template<>
inline RigidBody *const * polymorphicFind<Union>( RigidBody *const * first,
                                                  RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Union, RigidBody );

   while( first != last && (*first)->getType() != unionType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a union in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a union.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Union,RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a union.
 */
template<>
inline RigidBody *const * polymorphicFind<const Union>( RigidBody *const * first,
                                                        RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Union, RigidBody );

   while( first != last && (*first)->getType() != unionType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a union in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a union.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (Union,const RigidBody). The function traverses the range \f$ [first,last) \f$ of pointers
 * to rigid bodies until it finds the next pointer to a union.
 */
template<>
inline const RigidBody *const * polymorphicFind<Union>( const RigidBody *const * first,
                                                        const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( Union, const RigidBody );

   while( first != last && (*first)->getType() != unionType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************


//*************************************************************************************************
/*! \cond PE_INTERNAL */
/*!\brief Finds the next pointer to a union in the range \f$ [first,last) \f$.
 *
 * \param first Iterator to the first pointer of the pointer range.
 * \param last Iterator to the pointer one past the last pointer of the pointer range.
 * \return The next pointer to a union.
 *
 * This function is a specialization of the polymorphicFind() function for the type combination
 * (const Union,const RigidBody). The function traverses the range \f$ [first,last) \f$ of
 * pointers to rigid bodies until it finds the next pointer to a union.
 */
template<>
inline const RigidBody *const * polymorphicFind<const Union>( const RigidBody *const * first,
                                                              const RigidBody *const * last )
{
   pe_CONSTRAINT_MUST_BE_STRICTLY_DERIVED_FROM( const Union, const RigidBody );

   while( first != last && (*first)->getType() != unionType ) ++first;
   return first;
}
/*! \endcond */
//*************************************************************************************************

} // namespace pe

#endif
