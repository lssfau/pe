//=================================================================================================
/*!
 *  \file pe/core/World.h
 *  \brief Header file for the rigid body simulation world
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

#ifndef _PE_CORE_WORLD_H_
#define _PE_CORE_WORLD_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <stdexcept>
#include <pe/core/BodyManager.h>
#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/core/CollisionSystem.h>
#include <pe/core/Configuration.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/Settings.h>
#include <pe/core/Types.h>
#include <pe/core/WorldID.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/Assert.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/singleton/Singleton.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup world World
 * \ingroup core
 */
/*!\brief A simulation world for an arbitrary number of rigid bodies.
 * \ingroup world
 *
 * \section world_general General
 *
 * The World class represents a single rigid body simulation: all rigid bodies of the world are
 * simultaneously moved forward in time, rigid bodies in this world can only interact with other
 * bodies of this world and all rigid bodies within a simulation world are exposed to the same
 * specified gravity of the world. From an implemenation point of view, a World is a container
 * for an arbitrary number of rigid bodies.\n\n
 *
 *
 * \section world_setup The simulation world
 *
 * There is only a single active simulation world that contains all the rigid bodies. This one
 * simulation world is simply refered to as "the world". In order to access the one simulation
 * world use the following function:
 *
 * -# pe::theWorld()
 *
 * This function returns a handle to the simulation world, which can be used to configure the
 * rigid body simulation or to access the contained rigid bodies.\n
 * All newly created rigid bodies are automatically added to the one simulation world. However,
 * it is possible to create compounds of rigid bodies by adding rigid bodies to a Union:

   \code
   // Creating a single, iron sphere with radius 1 at the position (0,0,0). It is automatically
   // added to the simulation world.
   SphereID s = createSphere( 1, 0.0, 0.0, 0.0, 1.0, iron );

   // Creating an empty union. It is also automatically added to the simulation world.
   UnionID u = createUnion( 2 );

   // Adding the sphere to the union. From now on, the sphere is part of the union, not the
   // simulation world. If the union is destroyed, so is the sphere.
   u->add( s );
   \endcode

 * In order to separate rigid bodies from a union and to add them to the simulation world again,
 * the add() function of the world can be used:

   \code
   // Transfering the sphere from the union back to the simulation world. Destroying the union
   // will now not affect the sphere.
   world->add( s );
   \endcode

 * \n \section world_bodies Access to the contained rigid bodies
 *
 * In order to handle and access the rigid bodies contained in the world, the World class provides
 * several functions:

   \code
   // Getting a handle to the simulation world
   pe::WorldID world = theWorld();

   // Size functions
   pe::World::SizeType total   = world->size();          // Total number of bodies in the world
   pe::World::SizeType spheres = world->size<Sphere>();  // Number of spheres in the simulation world

   // Direct access to the rigid bodies (non-constant version)
   pe::BodyID body = world->getBody( 0 );  // Direct access to the first rigid body in the world

   // Iterators over all contained rigid bodies (non-constant versions)
   pe::World::Iterator begin = world->begin();
   pe::World::Iterator end   = world->end();

   for( ; begin!=end; ++begin )
      ...

   // Iterators over a specific rigid body type (non-constant versions)
   pe::World::Bodies::CastIterator<Sphere> begin = world->begin<Sphere>();
   pe::World::Bodies::CastIterator<Sphere> end   = world->end<Sphere>();

   for( ; begin!=end; ++begin )
      ...
   \endcode
 */
class PE_PUBLIC World : public BodyManager
            , private Singleton<World,CollisionSystem<Config>,logging::Logger>
{
private:
   //**Type definitions****************************************************************************
   typedef BodyStorage<Config>  BS;             //!< Body storage of the simulation world.
   //**********************************************************************************************

public:
   //**Type definitions****************************************************************************
   typedef BS::Bodies           Bodies;         //!< Body container for the simulation world.
   typedef BS::SizeType         SizeType;       //!< Rigid body count of the simulation world.
   typedef BS::Iterator         Iterator;       //!< Iterator over the rigid bodies.
   typedef BS::ConstIterator    ConstIterator;  //!< Constant iterator over the rigid bodies.
   //**********************************************************************************************

private:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit World();
   //@}
   //**********************************************************************************************

public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~World();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
                          inline real                 getDamping() const;
                          inline const Vec3&          getGravity() const;

                          inline SizeType             size()  const;
   template< typename C > inline SizeType             size()  const;
                          inline BodyID               getBody( unsigned int index );
                          inline ConstBodyID          getBody( unsigned int index ) const;

                          inline Iterator             begin();
                          inline ConstIterator        begin() const;
   template< typename C > inline Bodies::CastIterator<C>      begin();
   template< typename C > inline Bodies::ConstCastIterator<C> begin() const;

                          inline Iterator             end();
                          inline ConstIterator        end()   const;
   template< typename C > inline Bodies::CastIterator<C>      end();
   template< typename C > inline Bodies::ConstCastIterator<C> end()   const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setAutoForceReset( bool reset );
   inline void setDamping( real damping );
   inline void setGravity( real gx, real gy, real gz );
   inline void setGravity( const Vec3& gravity );
   //@}
   //**********************************************************************************************

   //**Translation functions***********************************************************************
   /*!\name Translation functions */
   //@{
   void translate( real dx, real dy, real dz );
   void translate( const Vec3& dp );
   //@}
   //**********************************************************************************************

   //**Rotation functions**************************************************************************
   /*!\name Rotation functions */
   //@{
   inline void rotateAroundOrigin( real x, real y, real z, real angle );
          void rotateAroundOrigin( const Vec3& axis, real angle );
          void rotateAroundOrigin( real xangle, real yangle, real zangle );
   inline void rotateAroundOrigin( const Vec3& euler );

          void rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle );
          void rotateAroundPoint( const Vec3& point, const Vec3& euler );
   //@}
   //**********************************************************************************************

   //**Add functions*******************************************************************************
   /*!\name Add functions */
   //@{
   virtual void add( BodyID body );
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   void simulationStep( real timestep );
   void run( unsigned int steps, real timestep );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline bool intersectsWith( ConstBodyID body );
   inline bool ownsPoint( real px, real py, real pz );
   inline bool ownsPoint( const Vec3& gpos );
          void clear();
   //@}
   //**********************************************************************************************

   //**Destroy functions***************************************************************************
   /*!\name Destroy functions */
   //@{
   Iterator destroy( Iterator pos );
   //@}
   //**********************************************************************************************

   //**Communication functions*********************************************************************
   /*!\name Communication functions */
   //@{
   static void synchronize();
   //@}
   //**********************************************************************************************

   //**Debugging functions*************************************************************************
   /*!\name Debugging functions */
   //@{
   bool checkSetup() const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os ) const;
   //@}
   //**********************************************************************************************

private:
   //**Remove functions****************************************************************************
   /*!\name Remove functions */
   //@{
   virtual void remove( BodyID body );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void removeFromCollisionDetector( BodyID body );
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend WorldID theWorld();
   pe_BEFRIEND_SINGLETON;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  WORLD SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name World setup functions */
//@{
inline PE_PUBLIC WorldID theWorld();
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the \b pe simulation world.
 * \ingroup world
 *
 * \return Handle to the active simulation world.
 *
 * This function can be seen as the starting point for every rigid body simulation. It
 * returns a handle to the one active simulation world that can be used to configure the
 * rigid body simulation or to access all contained rigid bodies. The first call to this
 * function will activate the simulation world and return the world handle, subsequent
 * calls will only return the handle.
 */
inline WorldID theWorld()
{
   return World::instance();
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the damping factor of the simulation world.
 *
 * \return The damping factor of the simulation world.
 *
 * The damping factor controls how the linear and angular velocity of a rigid body is reduced
 * by drag forces. Every moving rigid body loses kinetic energy in every time step accoring to
 * this factor. The value of damping is in the range \f$ [0..1] \f$. A damping factor of 0 will
 * reduce the velocity to nothing. This would mean that the body could not sustain any motion
 * without a force. A damping factor of 1 means that the body keeps all its velocity (which is
 * equivalent to no damping).
 */
inline real World::getDamping() const
{
   return Settings::damping();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the gravity of the simulation world.
 *
 * \return The gravity of the simulation world.
 */
inline const Vec3& World::getGravity() const
{
   return Settings::gravity();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rigid bodies in the simulation world.
 *
 * \return The number of rigid bodies.
 */
inline World::SizeType World::size() const
{
   return theCollisionSystem()->getBodyStorage().size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of rigid bodies of type \a C contained in the simulation world.
 *
 * \return The number of rigid bodies of type \a C.
 *
 * This function calculates the total number of rigid bodies of type \a C within the simulation
 * world, where \a C is a body type (Sphere, Box, Capsule, Plane, Union, ...). The attempt to
 * use this function for non-body types results in a compile time error.

   \code
   pe::World::SizeType total   = world->size();          // Total number of rigid bodies in the world
   pe::World::SizeType spheres = world->size<Sphere>();  // Number of spheres contained in the world
   \endcode

 * \b Note: The total number of rigid bodies of type \a C is not cached inside the simulation
 * world but is calculated each time the function is called. Using the templated version of
 * size() to calculate the total number rigid bodies of type \a C is therefore more expensive
 * than using the non-template version of size() to get the total number of rigid bodies in
 * the world!
 */
template< typename C >
inline World::SizeType World::size() const
{
   return theCollisionSystem()->getBodyStorage().size<C>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the indexed rigid body.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Handle to the accessed rigid body.
 *
 * \b Note: No runtime check is performed to ensure the validity of the access index.
 */
inline BodyID World::getBody( unsigned int index )
{
   // WARNING: Using friend relationship to get non-constant BodyID from body storage.
   return theCollisionSystem()->bodystorage_.at( index );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant handle to the indexed rigid body.
 *
 * \param index Access index. The index has to be in the range \f$[0..size-1]\f$.
 * \return Constant handle to the accessed rigid body.
 *
 * \b Note: No runtime check is performed to insure the validity of the access index.
 */
inline ConstBodyID World::getBody( unsigned int index ) const
{
   return theCollisionSystem()->getBodyStorage().at( index );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first rigid body of the simulation world.
 *
 * \return Iterator to the first rigid body of the simulation world.
 */
inline World::Iterator World::begin()
{
   // WARNING: Using friend relationship to get non-constant iterator from body storage.
   return theCollisionSystem()->bodystorage_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first rigid body of the simulation world.
 *
 * \return Iterator to the first rigid body of the simulation world.
 */
inline World::ConstIterator World::begin() const
{
   return theCollisionSystem()->getBodyStorage().begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first rigid body of type \a C within the simulation world.
 *
 * \return Iterator to the first rigid body of type \a C.
 *
 * This function returns an iterator to the first rigid body of type \a C within the simulation
 * world, where \a C is a body type (Sphere, Box, Capsule, Plane, Union, ...). In case there are
 * no rigid bodies of type \a C contained in the world, an iterator just past the last rigid body
 * of the world is returned. In combination with the according end function (see example), this
 * iterator allows to iterate over all rigid bodies of type \a C contained in the simulation world.
 * The attempt to use this function for non-body types results in a compile time error.

   \code
   // Definition of function for non-constant worlds
   void f( pe::WorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::CastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::CastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant worlds
   void f( pe::ConstWorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::ConstCastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::ConstCastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all bodies of type
 * \a C contained in the simulation world is more expensive than using the non-template version
 * to iterate over all contained bodies. Use this function only if you require a type-specific
 * member of type \a C (e.g. Sphere::getRadius()).
 */
template< typename T >
inline World::Bodies::CastIterator<T> World::begin()
{
   // WARNING: Using friend relationship to get non-constant iterator from body storage.
   return theCollisionSystem()->bodystorage_.begin<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first rigid body of type \a C within the simulation world.
 *
 * \return Iterator to the first rigid body of type \a C.
 *
 * This function returns an iterator to the first rigid body of type \a C within the simulation
 * world, where \a C is a body type (Sphere, Box, Capsule, Plane, Union, ...). In case there are
 * no rigid bodies of type \a C contained in the world, an iterator just past the last rigid body
 * of the world is returned. In combination with the according end function (see example), this
 * iterator allows to iterate over all rigid bodies of type \a C contained in the simulation world.
 * The attempt to use this function for non-body types results in a compile time error.

   \code
   // Definition of function for non-constant worlds
   void f( WorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::CastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::CastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant worlds
   void f( ConstWorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::ConstCastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::ConstCastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all bodies of type
 * \a C contained in the simulation world is more expensive than using the non-template versions
 * to iterate over all contained bodies. Use this function only if you require a type-specific
 * member of type \a C (e.g. Sphere::getRadius()).
 */
template< typename T >
inline World::Bodies::ConstCastIterator<T> World::begin() const
{
   return theCollisionSystem()->getBodyStorage().begin<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last rigid body of the simulation world.
 *
 * \return Iterator just past the last rigid body of the simulation world.
 */
inline World::Iterator World::end()
{
   // WARNING: Using friend relationship to get non-constant iterator from body storage.
   return theCollisionSystem()->bodystorage_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last rigid body of the simulation world.
 *
 * \return Iterator just past the last rigid body of the simulation world.
 */
inline World::ConstIterator World::end() const
{
   return theCollisionSystem()->getBodyStorage().end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last rigid body of the simulation world.
 *
 * \return Iterator just past the last rigid body of the simulation world.
 *
 * This function returns an iterator just past the last rigid body of the simulation world. In
 * combination with the according begin function (see example), this iterator allows to iterate
 * over all bodies of type \a C contained in the simulation world. The attempt to use this
 * function for non-body types results in a compile time error.

   \code
   // Definition of function for non-constant worlds
   void f( WorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::CastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::CastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant worlds
   void f( ConstWorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::ConstCastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::ConstCastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all bodies of type
 * \a C contained in the simulation world is more expensive than using the non-template version
 * to iterate over all contained bodies. Use this function only if you require a type-specific
 * member of type \a C (e.g. Sphere::getRadius()).
 */
template< typename T >
inline World::Bodies::CastIterator<T> World::end()
{
   // WARNING: Using friend relationship to get non-constant iterator from body storage.
   return theCollisionSystem()->bodystorage_.end<T>();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last rigid body of the simulation world.
 *
 * \return Iterator just past the last rigid body of the simulation world.
 *
 * This function returns an iterator just past the last rigid body of the simulation world. In
 * combination with the according begin function (see example), this iterator allows to iterate
 * over all bodies of type \a C contained in the simulation world. The attempt to use this
 * function for non-body types results in a compile time error.

   \code
   // Definition of function for non-constant worlds
   void f( WorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::CastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::CastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }

   // Definition of function f for constant worlds
   void f( ConstWorldID world )
   {
      // Loop over all spheres contained in the simulation world
      pe::World::Bodies::ConstCastIterator<Sphere> begin = world->begin<Sphere>();
      pe::World::Bodies::ConstCastIterator<Sphere> end   = world->end<Sphere>();

      for( ; begin!=end; ++begin )
         ...
   }
   \endcode

 * \b Note: Using the templated versions of begin() and end() to traverse all bodies of type
 * \a C contained in the simulation world is more expensive than using the non-template version
 * to iterate over all contained bodies. Use this function only if you require a type-specific
 * member of type \a C (e.g. Sphere::getRadius()).
 */
template< typename T >
inline World::Bodies::ConstCastIterator<T> World::end() const
{
   return theCollisionSystem()->getBodyStorage().end<T>();
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Switching the automatic force reset on or off.
 *
 * \param reset \a true to switch the force reset on, \a false to switch it off.
 * \return void
 *
 * If the automatic force reset is used, the accumulated forces on the rigid bodies will
 * automatically be set to zero after each simulation step. If the force reset is switched
 * off, all rigid bodies will keep the acting forces for the next simulation step.
 *
 * \b Note: Only deactivate the automatic force reset if you are sure what you are doing
 *          (e.g. manually resetting the forces on the rigid bodies after every time step).
 *          Failure to reset the forces properly can lead to severe instabilities in the
 *          physics engine!
 */
inline void World::setAutoForceReset( bool reset )
{
   Settings::forceReset_ = reset;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the damping factor for artificial drag forces.
 *
 * \param damping The damping factor for the simulation world \f$ [0..1] \f$.
 * \return void
 * \exception std::invalid_argument Invalid damping factor.
 *
 * The damping factor controls how the linear and angular velocity of a rigid body is reduced
 * by drag forces. Every moving rigid body loses kinetic energy in every time step accoring to
 * this factor. The value of damping must be in the range \f$ [0..1] \f$. If the damping is 0,
 * then the velocity will be reduced to nothing. This would mean that the body could not sustain
 * any motion without a force. A value of 1 means that the body keeps all its velocity (which is
 * equivalent to no damping). The default value for the damping factor is 1 (no damping).
 */
inline void World::setDamping( real damping )
{
   if( damping < real(0) || damping > real(1) )
      throw std::invalid_argument( "Invalid damping factor" );

   Settings::damping_ = damping;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the gravity in the simulation world.
 *
 * \param gx The x-component of the gravity.
 * \param gy The y-component of the gravity.
 * \param gz The z-component of the gravity.
 * \return void
 */
inline void World::setGravity( real gx, real gy, real gz )
{
   Settings::gravity_ = Vec3( gx, gy, gz );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the gravity in the simulation world.
 *
 * \param gravity The gravity vector.
 * \return void
 */
inline void World::setGravity( const Vec3& gravity )
{
   Settings::gravity_ = gravity;
}
//*************************************************************************************************




//=================================================================================================
//
//  ROTATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Rotation of the entire simulation world around the origin of the global world frame.
 *
 * \param x The x-component of the global rotation axis.
 * \param y The y-component of the global rotation axis.
 * \param z The z-component of the global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function rotates the entire simulation world around the origin of the global world frame.
 * The world is rotated around the given axis \a (x,y,z) by \a angle degrees (radian measure).
 */
inline void World::rotateAroundOrigin( real x, real y, real z, real angle )
{
   rotateAroundOrigin( Vec3( x, y, z ), angle );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the entire simulation world around the origin of the global world frame.
 *
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 *
 * This function rotates the entire simulation world around the origin of the global world frame.
 * The world is rotated by the Euler angles \a euler (all components in radian measure). The
 * rotations are applied in the order x, y, and z.
 */
inline void World::rotateAroundOrigin( const Vec3& euler )
{
   rotateAroundOrigin( euler[0], euler[1], euler[2] );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Tests if a rigid body intersects with the local domain of the simulation world.
 *
 * \return \a true if the body intersects with the local domain, \a false if not.
 *
 * This function tests whether the given rigid body is partially contained in the local domain of
 * the simulation world. In case the rigid body is at least partially contained, the function
 * returns \a true, otherwise it returns \a false. Note that for a non-parallel simulation,
 * this function always returns \a true.
 */
inline bool World::intersectsWith( ConstBodyID body )
{
   return theCollisionSystem()->getDomain().intersectsWith( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is owned by the local domain of the simulation world.
 *
 * \param px The x-component of the global coordinate.
 * \param py The y-component of the global coordinate.
 * \param pz The z-component of the global coordinate.
 * \return \a true if the coordinate is owned by the local domain, \a false if not.
 *
 * This function tests whether the given global coordinate is owned by the local domain of
 * the simulation world. In case the coordinate is within the bounds of the domain, the function
 * returns \a true, otherwise it returns \a false. If the point is located on the boundary surface
 * of two domains then it is part of the domain with lower rank. Note that for a non-parallel
 * simulation, this function always returns \a true.
 */
inline bool World::ownsPoint( real px, real py, real pz )
{
   return theCollisionSystem()->getDomain().ownsPoint( Vec3( px, py, pz ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Tests if a global coordinate is owned by the local domain of the simulation world.
 *
 * \param gpos The global coordinate.
 * \return \a true if the coordinate is owned by the local domain, \a false if not.
 *
 * This function tests whether the given global coordinate is owned by the local domain of
 * the simulation world. In case the coordinate is within the bounds of the domain, the function
 * returns \a true, otherwise it returns \a false. If the point is located on the boundary surface
 * of two domains then it is part of the domain with lower rank. Note that for a non-parallel
 * simulation, this function always returns \a true.
 */
inline bool World::ownsPoint( const Vec3& gpos )
{
   return theCollisionSystem()->getDomain().ownsPoint( gpos );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name World operators */
//@{
std::ostream& operator<<( std::ostream& os, const World& world );
std::ostream& operator<<( std::ostream& os, const WorldID& world );
std::ostream& operator<<( std::ostream& os, const ConstWorldID& world );
//@}
//*************************************************************************************************

} // namespace pe

#endif
