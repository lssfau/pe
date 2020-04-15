//=================================================================================================
/*!
 *  \file pe/core/OpenCLBodyManager.h
 *  \brief Management of the body storage copy on the OpenCL device.
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

#ifndef _PE_CORE_OPENCLBODYMANAGER_H_
#define _PE_CORE_OPENCLBODYMANAGER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <boost/unordered_map.hpp>
#include <gpusolve.hpp>
#include <iostream>
#include <pe/core/OpenCLBodyProperties.h>
#include <pe/core/TimeStep.h>
#include <pe/util/logging/DebugSection.h>
#include <set>
#include <vector>


namespace pe {


//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\defgroup opencl OpenCL
 * \ingroup core
 */
/*!\brief Management class to keep body properties on the OpenCL device in sync with the host body
 *        storage.
 * \ingroup opencl
 *
 * The OpenCLBodyManager is intended to be instantiated and driven by the OpenCL collision system.
 * Whenever bodies are added or removed from the simulation the collision system will pass on that
 * information to the OpenCLBodyManager which will at first keep track of the changes and later on
 * synchronize them with the OpenCL device in one go before collision resolution is started. After
 * the collisions are resolved the OpenCLSolver will trigger the transfer of the results back to
 * the host.
 */
template< typename C >  // Type of the configuration
class OpenCLBodyManager {
   //**Type definitions****************************************************************************
   /*!\name Type definitions */
   //@{
   typedef BodyStorage<C> BS;
   typedef typename Config::BodyID BodyID;
   typedef boost::unordered_map<const size_t, size_t> BodyMap;
   //@}
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   OpenCLBodyManager( BS& bodystorage );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~OpenCLBodyManager();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline size_t getBodyIndex( ConstBodyID body );
   inline OpenCLBodyProperties& getBodyProperties();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void info() const;
   //@}
   //**********************************************************************************************

   //**Synchronization functions*******************************************************************
   /*!\name Utility functions */
   //@{
   inline void bodyAdded( BodyID body );
   inline void bodyRemoved( BodyID body );
   void syncOpenCL();
   void syncHost();
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t size_;                      //!< Number of bodies.
   BodyMap bodies_;                   //!< Maps system ids to offsets.
   BodyMap offsets_;                  //!< Maps offsets to system ids.
   std::vector<size_t> remainingIDs_; //!< Temporary storage for system ids of managed bodies.
   std::vector<bool> active_;         //!< Bodies are alive.
   std::vector<bool> removedBodies_;  //!< Bodies removed from system.
   std::set<size_t> addedBodies_;     //!< Cache for recently added bodies.
   size_t sizeRemoved_;               //!< Number of removed bodies.
   bool syncAll_;                     //!< Full sync host/OpenCL required.
   bool fullSyncRequired_;            //!< Type of next sync to OpenCL.
   std::auto_ptr<OpenCLBodyProperties> props_;  //!< Actual body properties.
   gpusolve::opencl::util::Clock<> clock_;      //!< Clock which is used to measure time spent in the body manager.
   gpusolve::opencl::util::Clock<> syncClock_;  //!< Clock which is used to measure time spent actually in synchronization.
   double time_;                      //!< Time spent in OpenCLBodyManager.
   double syncTime_;                  //!< Time spent on actual synchronization.
   size_t syncCount_;                 //!< Number of synchronizations performed.
   BS& bodystorage_;                  //!< Reference to the central body storage.

   static const size_t removeThreshold_ = 3; //!< If more bodies were removed then a complete synchronization is performed.
   //@}
   //**********************************************************************************************
};




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the OpenCLBodyManager class.
 */
template< typename C >  // Type of the configuration
OpenCLBodyManager<C>::OpenCLBodyManager( BS& bodystorage )
   : size_(0)
   , bodies_()
   , offsets_()
   , remainingIDs_()
   , active_()
   , removedBodies_()
   , addedBodies_()
   , sizeRemoved_(0)
   , syncAll_(true)
   , props_()
   , clock_()
   , syncClock_()
   , time_(0)
   , syncTime_(0)
   , syncCount_(0)
   , bodystorage_( bodystorage )
{
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the OpenCLBodyManager class.
 */
template< typename C >  // Type of the configuration
OpenCLBodyManager<C>::~OpenCLBodyManager() {
   pe_LOG_DEBUG_SECTION( log ) {
      std::cout << "OpenCL overhead: " << time_ << " s" << std::endl;
      std::cout << "\t# of syncs: " << syncCount_ << std::endl;
      std::cout << "\toverhead of syncs: " << syncTime_ << " s (" << syncTime_ / syncCount_;
      std::cout << " s avg)" << std::endl;
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the OpenCL device body storage index corresponding to the given host body.
 *
 * \param body The host body ID.
 * \return The OpenCL device body storage index corresponding to the given host body.
 */
template< typename C >  // Type of the configuration
size_t OpenCLBodyManager<C>::getBodyIndex( ConstBodyID body ) {
   clock_.start();

   BodyMap::const_iterator pos = bodies_.find(body->getSystemID());
   if( pos == bodies_.end() ) {
      std::cerr << "failed to find body #" << body->getSystemID();
      std::cerr << " in BodyManager" << std::endl;
      exit(EXIT_FAILURE);
   }

   clock_.stop();
   time_ += clock_.runTime();

   return pos->second;
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Returns a read-write reference to the OpenCL body storage.
 *
 * \return A read-write reference to the OpenCL body storage.
 */
template< typename C >  // Type of the configuration
OpenCLBodyProperties& OpenCLBodyManager<C>::getBodyProperties() {
   // The OpenCLBodyProperties are dynamically allocated in order to postpone
   // the device selection happening in gpusolve::opencl::OpenCLSystemInterface.
   if( !props_.get() )
      props_.reset( new OpenCLBodyProperties() );

   return *props_;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Prints OpenCL overhead information.
 *
 * \return void
 */
template< typename C >  // Type of the configuration
void OpenCLBodyManager<C>::info() const {
   std::cout << "OpenCL overhead: " << time_ << " s" << std::endl;
}
//*************************************************************************************************




//=================================================================================================
//
//  SYNCHRONIZATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Notifies the body manager that a body was added to the simulation.
 *
 * \param body The host body ID of the added body.
 * \return void
 */
template< typename C >  // Type of the configuration
void OpenCLBodyManager<C>::bodyAdded( BodyID body ) {
   clock_.start();
   size_t sid = body->getSystemID();
   addedBodies_.insert(sid);
   clock_.stop();
   time_ += clock_.runTime();
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Notifies the body manager that a body will be removed from the simulation.
 *
 * \param body The host body ID of the body to be removed.
 * \return void
 */
template< typename C >  // Type of the configuration
void OpenCLBodyManager<C>::bodyRemoved( BodyID body ) {
   clock_.start();
   size_t sid = body->getSystemID();

   bool cached = false;

   // check body cache
   std::set<size_t>::const_iterator cachePos = addedBodies_.find(sid);
   if(cachePos != addedBodies_.end()) {
      addedBodies_.erase(cachePos);
      cached = true;
   }

   if(!cached) {
      // determine index in OpenCLBodyManager
      BodyMap::const_iterator pos = bodies_.find(sid);
      if(pos == bodies_.end()) {
         std::cerr << "failed to find body #" << sid << " in OpenCLBodyManager";
         std::cerr << std::endl;
         exit(EXIT_FAILURE);
      }

      removedBodies_[pos->second] = true;
      ++sizeRemoved_;
   }

   clock_.stop();
   time_ += clock_.runTime();
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Synchronizes host body attributes to the OpenCL device body storage.
 *
 * \return void
 */
template< typename C >  // Type of the configuration
void OpenCLBodyManager<C>::syncOpenCL() {
   OpenCLBodyProperties& props( getBodyProperties() );

   clock_.start();

   size_t bodyCount = size_ + addedBodies_.size();

   if(sizeRemoved_ > removeThreshold_) {
      bodyCount -= sizeRemoved_;
      syncAll_ = true;
   }

   pe_LOG_DEBUG_SECTION( log ) {
      if( syncAll_ )
         std::cout << "Performing complete rebuild of data structures." << std::endl;
   }

   props.resize(bodyCount);
   active_.resize(bodyCount);
   removedBodies_.resize(bodyCount, false);

   // insert newly added bodies
   std::set<size_t>::const_iterator ab = addedBodies_.begin();
   for(; ab != addedBodies_.end(); ++ab) {
      bodies_.insert(BodyMap::value_type(*ab, size_));
      offsets_.insert(BodyMap::value_type(size_, *ab));

      // DEBUG
      // std::cerr << "added body " << *ab << " with index " << size_ << std::endl;

      ++size_;
   }
   addedBodies_.clear();

   // remove gaps in assigned offsets
   if(syncAll_) {
      // assign new offsets to bodies
      remainingIDs_.reserve(size_);
      for(size_t i = 0; i < size_; ++i) {
         if(!removedBodies_[i])
            remainingIDs_.push_back(offsets_[i]);
      }

      for(size_t i = 0; i < remainingIDs_.size(); ++i)
         bodies_[remainingIDs_[i]] = i;

      std::fill(removedBodies_.begin(), removedBodies_.end(), false);
      sizeRemoved_ = 0;
   }

   // enable access to OpenCL data structures
   syncClock_.start();
   props.hostAccess(true);

   for(typename BS::Iterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body) {
      size_t index = 0;
      BodyMap::iterator pos = bodies_.find(body->getSystemID());
      if(pos == bodies_.end()) {
         std::cerr << "failed to find body #" << body->getSystemID() << " in OpenCLBodyManager";
         std::cerr << std::endl;
         exit(EXIT_FAILURE);
      }


      index = pos->second;

      active_[index] = !body->isFixed();

      // global position
      const Vec3& gpos = body->getPosition();
      props.gpos_[index][0] = gpos[0];
      props.gpos_[index][1] = gpos[1];
      props.gpos_[index][2] = gpos[2];
      props.gpos_[index][3] = 0;

      // linear velocity
      const Vec3& v = body->getLinearVel();
      props.v_[index][0] = v[0];
      props.v_[index][1] = v[1];
      props.v_[index][2] = v[2];
      props.v_[index][3] = 0;

      // angular velocity
      const Vec3& w = body->getAngularVel();
      props.w_[index][0] = w[0];
      props.w_[index][1] = w[1];
      props.w_[index][2] = w[2];
      props.w_[index][3] = 0;

      // mass
      props.invMass_[index] = body->getInvMass();

      // inertia
      const Mat3& invI = body->getInvBodyInertia();
      size_t offset = props.invI_.size() / 3;
      for(size_t i = 0; i < 3; ++i) {
         props.invI_[index + i * offset][0] = invI(i, 0);
         props.invI_[index + i * offset][1] = invI(i, 1);
         props.invI_[index + i * offset][2] = invI(i, 2);
         props.invI_[index + i * offset][3] = 0;
      }

      // TODO: status
      props.status_[index] = active_[index];
   }

   // disable access to OpenCL data structures
   props.hostAccess(false);
   syncClock_.stop();
   syncTime_ += syncClock_.runTime();
   ++syncCount_;

   if(syncAll_)
      syncAll_ = false;

   clock_.stop();
   time_ += clock_.runTime();
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Synchronizes device body storage data to the host body storage.
 *
 * \return void
 */
template< typename C >  // Type of the configuration
void OpenCLBodyManager<C>::syncHost() {
   OpenCLBodyProperties& props( getBodyProperties() );

   // enable access to OpenCL data structures
   syncClock_.start();
   props.hostAccess(true);

   for(typename BS::Iterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body) {
      size_t index = 0;
      BodyMap::iterator pos = bodies_.find(body->getSystemID());
      if(pos == bodies_.end()) {
         std::cerr << "failed to find body #" << body->getSystemID() << " in OpenCLBodyManager";
         std::cerr << std::endl;
         exit(EXIT_FAILURE);
      }

      index = pos->second;

      if(!active_[index])
         continue;

      // linear velocity
      Vec3 v(props.v_[index][0], props.v_[index][1], props.v_[index][2]);
      body->setLinearVel(v);

      // angular velocity
      Vec3 w(props.w_[index][0], props.w_[index][1], props.w_[index][2]);
      body->setAngularVel(w);
   }

   // disable access to OpenCL data structures
   props.hostAccess(false);
   syncClock_.stop();
   syncTime_ += syncClock_.runTime();
}
//*************************************************************************************************

} // namespace pe

#endif
