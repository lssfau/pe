//=================================================================================================
/*!
 *  \file src/core/World.cpp
 *  \brief Source file for the rigid body simulation world
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <ostream>
#include <pe/core/CollisionSystem.h>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/attachable/ForceGenerator.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/Overlap.h>
#include <pe/core/ParallelTrait.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/TimeStep.h>
#include <pe/core/Trigger.h>
#include <pe/core/World.h>
#include <pe/util/ColorMacros.h>
#include <pe/util/Logging.h>


namespace pe {

//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the World class.
 */
World::World()
   : BodyManager()                       // Initialization of the BodyManager base object
   , Singleton<World,CollisionSystem<Config>,logging::Logger>()  // Initialization of the Singleton base object
{
   // Logging the successful setup of the simulation world
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the simulation world instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor of the World class.
 *
 * Destroying a simulation world will also destroy any contained rigid body.
 */
World::~World()
{
   // Clearing the collision system
   theCollisionSystem()->clear();

   // Logging the successful destruction of the simulation world
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the simulation world instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  TRANSLATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Translation of the entire simulation world by the displacement vector (dx,dy,dz).
 *
 * \param dx The x-component of the translation/displacement.
 * \param dy The y-component of the translation/displacement.
 * \param dz The z-component of the translation/displacement.
 * \return void
 *
 * This function translates all rigid bodies contained in the simulation world by the displacement
 * vector (\a dx, \a dy, \a dz).
 */
void World::translate( real dx, real dy, real dz )
{
   const Iterator bodyEnd( end() );
   for( Iterator body=begin(); body!=bodyEnd; ++body ) {
      body->translate( dx, dy, dz );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Translation of the entire simulation world by the displacement vector \a dp.
 *
 * \param dp The displacement vector.
 * \return void
 *
 * This function translates all rigid bodies contained in the simulation world by the displacement
 * vector \a dp.
 */
void World::translate( const Vec3& dp )
{
   const Iterator bodyEnd( end() );
   for( Iterator body=begin(); body!=bodyEnd; ++body ) {
      body->translate( dp );
   }
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
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function rotates the entire simulation world around the origin of the global world frame.
 * The world is rotated around the given axis \a axis by \a angle degrees (radian measure).
 */
void World::rotateAroundOrigin( const Vec3& axis, real angle )
{
   const Iterator bodyEnd( end() );
   for( Iterator body=begin(); body!=bodyEnd; ++body ) {
      body->rotateAroundOrigin( axis, angle );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the entire simulation world around the origin of the global world frame.
 *
 * \param xangle Rotation around the x-axis (radian measure).
 * \param yangle Rotation around the y-axis (radian measure).
 * \param zangle Rotation around the z-axis (radian measure).
 * \return void
 *
 * This function rotates the entire simulation world around the origin of the global world frame.
 * The world is rotated by the Euler angles \a xangle, \a yangle and \a zangle (all components in
 * radian measure). The rotations are applied in the order x, y, and z.
 */
void World::rotateAroundOrigin( real xangle, real yangle, real zangle )
{
   const Iterator bodyEnd( end() );
   for( Iterator body=begin(); body!=bodyEnd; ++body ) {
      body->rotateAroundOrigin( xangle, yangle, zangle );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the entire simulation world around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param axis The global rotation axis.
 * \param angle The rotation angle (radian measure).
 * \return void
 *
 * This function rotates the entire simulation world around the given global coordinate \a point.
 * The world is rotated around the given axis \a axis by \a angle degrees (radian measure).
 */
void World::rotateAroundPoint( const Vec3& point, const Vec3& axis, real angle )
{
   const Iterator bodyEnd( end() );
   for( Iterator body=begin(); body!=bodyEnd; ++body ) {
      body->rotateAroundPoint( point, axis, angle );
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Rotation of the entire simulation world around a specific global coordinate.
 *
 * \param point The global center of the rotation.
 * \param euler 3-dimensional vector of the three rotation angles (radian measure).
 * \return void
 *
 * This function rotates the entire simulation world around the given global coordinate \a point.
 * The world is rotated by the Euler angles \a xangle, \a yangle and \a zangle (all components in
 * radian measure). The rotations are applied in the order x, y, and z.
 */
void World::rotateAroundPoint( const Vec3& point, const Vec3& euler )
{
   const Iterator bodyEnd( end() );
   for( Iterator body=begin(); body!=bodyEnd; ++body ) {
      body->rotateAroundPoint( point, euler );
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  RIGID BODY MANAGER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a rigid body to the simulation world.
 *
 * \param body The rigid body to be added to the world.
 * \return void
 *
 * This function adds a rigid body to the simulation world. This function can for example be
 * used to separate a rigid body from a Union. The world takes full responsibility for the
 * rigid body including the necessary memory management.
 *
 * \b Note: This function doesn't have to be called for newly created rigid bodies. New bodies
 * are automatically added to the simulation world.
 */
void World::add( BodyID body )
{
   if( body->hasManager() )
   {
      if( body->getManager() == ManagerID( this ) ) return;

      ManagerID manager = body->getManager();
      manager->remove( body );
   }

   setManager( body );
   body->wake();

   // WARNING: Using friend relationship to add rigid body to collision system.
   theCollisionSystem()->add( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the simulation world.
 *
 * \param body The rigid body to be removed.
 * \return void
 *
 * This function is a requirement for all rigid body manager instances and removes/deregisters
 * a rigid body from the simulation world.
 *
 * \b Note: This function doesn't have to be called explicitly. It is automatically called in
 * case the body manager is changed or if the rigid body is destroyed.
 *
 * \b Note: the body is not deallocated.
 *
 * \b Note: All end iterators are invalidated as well as iterators to the element itself and
 * iterators to the back element. All cast iterators are invalidated.
 */
void World::remove( BodyID body )
{
   pe_INTERNAL_ASSERT( body->getManager() == this, "Rigid body has wrong body manager" );

   resetManager( body );

   // WARNING: Using friend relationship to remove rigid body from collision system.
   theCollisionSystem()->remove( body );
}
//*************************************************************************************************




//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Simulation of one time step of size \a timestep.
 *
 * \param timestep Size of the time step.
 * \return void
 * \exception std::invalid_argument Invalid time step size.
 * \exception std::runtime_error Invalid function call inside exclusive section.
 *
 * The \a simulationStep function simultaneously calculates the movements of all rigid bodies
 * of the simulation world for a time step of size \a timestep. In order to minimize simulation
 * errors, the \a timestep has to be adequate for the current simulation.
 */
void World::simulationStep( real timestep )
{
   // Checking the size of the time step
   if( timestep <= real(0) )
      throw std::invalid_argument( "Invalid time step size" );

   // Checking if the function is called inside an exclusive section
   if( MPISettings::size() > 1 && ExclusiveSection::isActive() )
      throw std::runtime_error( "Invalid function call inside exclusive section" );

   // Configuring the time step
   ++TimeStep::step_;
   TimeStep::size_ = timestep;

   // Logging the start of the simulation step
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Start of simulation step " << TimeStep::step_ << " (step size=" << TimeStep::size_ << ")";
   }

   // Triggering the active force generators
   ForceGenerator::applyForces();

   // Calculating a time step of the simulation world
   theCollisionSystem()->simulationStep( timestep );

   // Triggering all registered trigger objects
   Trigger::triggerAll();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Simulation of one time step of size \a timestep.
 *
 * \param steps Number of time steps to be simulated.
 * \param timestep Size of the time step.
 * \return void
 * \exception std::invalid_argument Invalid time step size.
 * \exception std::runtime_error Invalid function call inside exclusive section.
 *
 * The \a run function performs \a steps time steps in the rigid body simulation world.
 * Each time step uses a time step size of \a time. In order to minimize simulation errors,
 * the \a timestep has to be adequate for the current simulation.
 */
void World::run( unsigned int steps, real timestep )
{
   for( unsigned int step=0; step<steps; ++step )
      simulationStep( timestep );
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Clearing the simulation world.
 *
 * \return void
 *
 * This function resets the simulation world to its initial state. It resets the gravity, the
 * force removal flag and destroys all contained rigid bodies.
 */
void World::clear()
{
   Settings::gravity_.reset();    // Resetting the gravity
   Settings::forceReset_ = true;  // Resetting the force reset flag

   // Clearing the collision system
   theCollisionSystem()->clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Deregisters a rigid body from the collision detector.
 *
 * \param body The rigid body to be deregistered from the collision detector.
 * \return void
 */
void World::removeFromCollisionDetector( BodyID body )
{
   // WARNING: Using friend relationship to remove rigid body from collision detector.
   theCollisionSystem()->removeFromCollisionDetector( body );
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTROY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destroying a rigid body contained in the simulation world.
 *
 * \param pos Iterator to the rigid body to be destroyed.
 * \return Iterator to the body after the destroyed body.
 *
 * This function destroys a rigid body contained in the simulation world. Note that
 * destroying the rigid body invalidates all remaining references/IDs to the body since
 * deallocation is performed.
 *
 * \b Note: All end iterators are invalidated as well as iterators to the element itself and
 * iterators to the back element. All cast iterators are invalidated.
 */
World::Iterator World::destroy( Iterator pos )
{
   BodyID bodyid( *pos );

   // Checking the validity of the iterator
   pe_INTERNAL_ASSERT( pos->getManager() == ManagerID( this ), "Invalid body iterator" );

   // WARNING: Using friend relationship to remove rigid body from collision system.
   Iterator it( theCollisionSystem()->remove( pos ) );

   // Deallocate the rigid body
   destroyBody( bodyid );
   return it;
}
//*************************************************************************************************




//=================================================================================================
//
//  DEBUGGING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setup check of the simulation world.
 *
 * \return \a true if no setup error is detected, \a false if a setup error is encountered.
 *
 * A setup check can be used after the setup of all rigid bodies in the simulation world to
 * detect badly placed bodies.
 */
bool World::checkSetup() const
{
   const ConstIterator bodyBegin( begin() );
   const ConstIterator bodyEnd  ( end()   );

   for( ConstIterator b1=bodyBegin; b1!=bodyEnd; ++b1 ) {
      for( ConstIterator b2=b1+1; b2!=bodyEnd; ++b2 ) {
         if( overlap( *b1, *b2 ) ) return false;
      }
   }

   return true;
}
//*************************************************************************************************




//=================================================================================================
//
//  COMMUNICATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Synchronization of all rigid bodies among the connected remote MPI processes.
 *
 * \return void
 * \exception std::runtime_error Invalid function call inside exclusive section.
 *
 * This function synchronizes all rigid bodies on the local process with all connected MPI
 * processes. This function should be used if there have been any changes to the local rigid
 * bodies that may also affect remote processes.
 *
 * \b Note: This function must not be called from inside an exclusive section. Calling this
 * function inside an exclusive section results in a \a std::runtime_error exception!
 */
void World::synchronize()
{
   // Checking if the function is called inside an exclusive section
   if( MPISettings::size() > 1 && ExclusiveSection::isActive() )
      throw std::runtime_error( "Invalid function call inside exclusive section" );

   CollisionSystemID cs( theCollisionSystem() );

   // Early exit in case no remote processes have been connected
   if( cs->getProcessStorage().isEmpty() ) return;

   // Checking whether the configuration is parallel
   pe_INTERNAL_ASSERT( ParallelTrait<Config>::value == 1, "Non-parallel configuration detected" );

   // Synchronizing the rigid bodies with the connected processes
   cs->synchronize();
}
//*************************************************************************************************




//=================================================================================================
//
//  OUTPUT FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Output of the current state of the simulation world.
 *
 * \param os Reference to the output stream.
 * \return void
 */
void World::print( std::ostream& os ) const
{
   os << " System gravity = " << Settings::gravity() << "\n"
      << " Number of rigid bodies = " << size() << "\n";
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Global output operator for the simulation world.
 * \ingroup world
 *
 * \param os Reference to the output stream.
 * \param world Reference to a constant world object.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const World& world )
{
   os << "--" << pe_BROWN << "SIMULATION WORLD PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------\n";
   world.print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for world handles.
 * \ingroup world
 *
 * \param os Reference to the output stream.
 * \param world World handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const WorldID& world )
{
   os << "--" << pe_BROWN << "SIMULATION WORLD PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------\n";
   world->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Global output operator for constant world handles.
 * \ingroup world
 *
 * \param os Reference to the output stream.
 * \param world Constant world handle.
 * \return Reference to the output stream.
 */
std::ostream& operator<<( std::ostream& os, const ConstWorldID& world )
{
   os << "--" << pe_BROWN << "SIMULATION WORLD PARAMETERS" << pe_OLDCOLOR
      << "---------------------------------------------------\n";
   world->print( os );
   os << "--------------------------------------------------------------------------------\n"
      << std::endl;
   return os;
}
//*************************************************************************************************

} // namespace pe
