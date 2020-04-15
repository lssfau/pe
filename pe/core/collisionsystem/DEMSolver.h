//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/DEMSolver.h
 *  \brief Specialization of the CollisionSystem class template for the DEM solver
 *
 *  Copyright (C) 2012 Tobias Preclik
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

#ifndef _PE_CORE_COLLISIONSYSTEM_DEMSOLVER_H_
#define _PE_CORE_COLLISIONSYSTEM_DEMSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <pe/core/attachable/Attachable.h>
#include <pe/core/attachable/AttachableStorage.h>
#include <pe/core/batches/BatchVector.h>
#include <pe/core/batches/Generators.h>
#include <pe/core/CollisionSystemID.h>
#include <pe/core/collisionsystem/MPICommunication.h>
#include <pe/core/Configuration.h>
#include <pe/core/contact/Contact.h>
#include <pe/core/contact/ContactVector.h>
#include <pe/core/detection/coarse/Detectors.h>
#include <pe/core/detection/fine/Detectors.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/io/BodySimpleAsciiWriter.h>
#include <pe/core/joint/JointStorage.h>
#include <pe/core/Marshalling.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISection.h>
#include <pe/core/MPISettings.h>
#include <pe/core/MPITag.h>
#include <pe/core/MPITrait.h>
#include <pe/core/ProfilingSection.h>
#include <pe/core/response/DEMSolver.h>
#include <pe/core/response/MPIDecoder.h>
#include <pe/core/response/MPIEncoder.h>
#include <pe/core/response/Solvers.h>
#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/core/rigidbody/MPIRigidBodyTrait.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/notifications/NotificationType.h>
#include <pe/core/notifications/RigidBodyCopyNotification.h>
#include <pe/core/notifications/RigidBodyDeletionNotification.h>
#include <pe/core/notifications/RigidBodyForceNotification.h>
#include <pe/core/notifications/RigidBodyMigrationNotification.h>
#include <pe/core/notifications/RigidBodyRemoteMigrationNotification.h>
#include <pe/core/notifications/RigidBodyRemovalNotification.h>
#include <pe/core/notifications/RigidBodyUpdateNotification.h>
#include <pe/core/RootSection.h>
#include <pe/core/SerialSection.h>
#include <pe/core/Types.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/BaseOf.h>
#include <pe/util/MemoryMeter.h>
#include <pe/util/Logging.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/singleton/Singleton.h>
#include <pe/util/Timing.h>
#include <pe/util/Types.h>
#include <pe/util/Vector.h>
#include <set>




namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the collision system for the DEM solver.
 * \ingroup core
 *
 * This specialization of the CollisionSystem class template adapts the collision system of the
 * rigid body simulation world to the requirements of the DEM solver.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystem< C<CD,FD,BG,response::DEMSolver> >
   : public MPICommunication, public Singleton< CollisionSystem< C<CD,FD,BG,response::DEMSolver> >, logging::Logger >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,BG,response::DEMSolver>      Config;             //!< Type of the configuration.

   typedef BodyStorage<Config>                  BS;                 //!< Type of the body storage.
   typedef typename BS::Iterator                BodyIterator;
   typedef typename Config::BodyType            BodyType;           //!< Type of the rigid bodies.
   typedef typename Config::BodyID              BodyID;             //!< Handle to a rigid body.
   typedef typename Config::ConstBodyID         ConstBodyID;        //!< Handle to a constant rigid body.

   typedef AttachableStorage<Config>            AS;                 //!< Type of the attachable storage.
   typedef typename Config::AttachableType      AttachableType;     //!< Type of the attachables.
   typedef typename Config::AttachableID        AttachableID;       //!< Handle for an attachable.
   typedef typename Config::ConstAttachableID   ConstAttachableID;  //!< Handle for a constant attachable.

   typedef JointStorage<Config>                 JS;                 //!< Type of the joint storage.
   typedef typename Config::JointType           JointType;          //!< Type of the joint.
   typedef typename Config::JointID             JointID;            //!< Handle for a joint.
   typedef typename Config::ConstJointID        ConstJointID;       //!< Handle for a constant joint.

   typedef typename Config::ContactType         ContactType;        //!< Type of the contacts.
   typedef typename Config::ContactID           ContactID;          //!< Handle to a contact.
   typedef typename Config::ConstContactID      ConstContactID;     //!< Handle to a constant contact.
   typedef ContactVector<ContactType,NoDelete>  Contacts;           //!< Container for contacts.

   typedef ProcessStorage<Config>               PS;                 //!< Type of the process storage.
   typedef typename PS::Iterator                ProcessIterator;
   typedef typename Config::ProcessType         ProcessType;        //!< Type of the remote processes.
   typedef typename Config::ProcessID           ProcessID;          //!< Handle for a remote process.
   typedef typename Config::ConstProcessID      ConstProcessID;     //!< Handle for a constant remote process.
   typedef PtrVector<Process,NoDelete>          Processes;          //!< Vector of MPI processes.

   //! Type of the coarse collision detection algorithm.
   /*! The type of the coarse collision detection algorithm is selected by the setting of
       the pe::pe_COARSE_COLLISION_DETECTOR macro. */
   typedef CD<Config>  CoarseDetector;

   //! Type of the fine collision detector.
   /*! The type of the fine collision detection algorithm is selected by the setting of
       the pe::pe_FINE_COLLISION_DETECTOR macro. */
   typedef FD  FineDetector;

   //! Type of the batch generation algorithm.
   /*! The type of the batch generation algorithm is selected by the setting of the
       pe_BATCH_GENERATOR macro. */
   typedef BG<Config>  BatchGenerator;

   //! Type of the collision response algorithm.
   /*! The type of the collision response algorithm is selected by the setting of the
       pe::pe_CONTACT_SOLVER macro. */
   typedef response::DEMSolver<Config>  ContactSolver;
   //**********************************************************************************************

private:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit CollisionSystem();
   //@}
   //**********************************************************************************************

public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   ~CollisionSystem();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline CoarseDetector& getCoarseDetector();
   inline ContactSolver&  getContactSolver();
   inline BS&             getBodyStorage();
   inline BS&             getBodyShadowCopyStorage();
   inline const PS&       getProcessStorage()     const;
   inline const AS&       getAttachableStorage()  const;
   inline const Domain&   getDomain()             const;
   //@}
   //**********************************************************************************************

   //**Query functions*****************************************************************************
   /*!\name Query functions */
   //@{
   inline bool            isSyncRequired()        const;
   inline bool            isSyncRequiredLocally() const;
   inline void            logProfilingSummary()   const;
   //@}
   //**********************************************************************************************

private:
   //**Add/remove functions************************************************************************
   /*!\name Add/remove functions */
   //@{
   inline void            add   ( BodyID body );
   inline void            remove( BodyID body );
   inline BodyIterator    remove( BodyIterator body );
   inline void            removeFromCollisionDetector( BodyID body );
   //@}
   //**********************************************************************************************

   //**Simulation functions************************************************************************
   /*!\name Simulation functions */
   //@{
   void simulationStep( real timestep );
   //@}
   //**********************************************************************************************

   //**Communication functions*********************************************************************
   /*!\name Communication functions */
   //@{
   void synchronizeForces();
   void synchronize();
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void clear();
   inline void clearContacts();
          bool checkUpdateFlags();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   Contacts contacts_;        //!< The currently active contacts of the simulation world.
   CoarseDetector detector_;  //!< The active collision detection algorithm.
   ContactSolver solver_;     //!< The active collision response algorithm.
   Processes lriProcesses_;   //!< Processes having long-range interactions with our bodies.
   BS bodystorage_;           //!< The rigid body storage.
   BS bodystorageShadowCopies_;  //!< The rigid body storage for all shadow copies.
   PS processstorage_;        //!< The process storage.
   Domain domain_;            //!< The local process domain.
   AS attachablestorage_;     //!< The attachable storage.
   JS jointstorage_;          //!< The joint storage.

   MPITag lastSyncTag_;

   //**********************************************************************************************
   /*! \cond PE_INTERNAL */
   /*!\brief Functor for comparing the system ID of two bodies.
    *
    * Returns true if the system ID of the first body is less than the system ID of the second body.
    */
   struct LessSystemID {
      //**Binary function call operator************************************************************
      /*!\name Binary function call operator */
      //@{
      bool operator()( BodyID b1, BodyID b2 ) const {
         return b1->getSystemID() < b2->getSystemID();
      }
      //@}
      //*******************************************************************************************
   };
   /*! \endcond */
   //**********************************************************************************************
   std::set<BodyID, LessSystemID> globalNonfixedBodies_;  //!< Global, non-fixed bodies whose applied forces need to be reduced.
   std::vector<real> reductionBuffer_;                    //!< Buffer for the reduction of forces and torques acting on global non-fixed bodies.
   bool requireSync_;         //!< Flag indicating whether this process requires a synchronization prior to the next time step.

   timing::WcTimer timeCollisionDetection_, timeCollisionResponse_, timeBodySync_, timeBodySyncAssembling_, timeBodySyncCommunicate_, timeBodySyncParsing_, timeForceSync_, timeForceSyncAssembling_, timeForceSyncCommunicate_, timeForceSyncParsing_, timeForceSyncGlobals_, timeIntegration_, timeSimulationStep_;
   std::vector<timing::WcTimer*> timers_;

   MemoryMeter  memCollisionDetection_, memCollisionResponse_, memBodySync_, memBodySyncAssembling_, memBodySyncCommunicate_, memBodySyncParsing_, memForceSync_, memForceSyncAssembling_, memForceSyncCommunicate_, memForceSyncParsing_, memForceSyncGlobals_, memIntegration_, memSimulationStep_;
   std::vector<MemoryMeter*> memoryMeters_;

   TransferMeter sentBodySync_, sentForceSync_;
   std::vector<TransferMeter*> sentMeters_;

   TransferMeter receivedBodySync_, receivedForceSync_;
   std::vector<TransferMeter*> receivedMeters_;
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class World;
   friend class MPISystem;
   template< typename Geometry >
   friend void defineLocalDomain( const Geometry& geometry );
   // Attachables register directly instead of the detour via a manager and thus need friend access to the attachable storage.
   friend GravityID instantiateGravity( id_t sid, pe::BodyID body, const Vec3& gravity );
   friend GravityID attachGravity( pe::BodyID body, const Vec3& gravity );
   friend SpringID instantiateSpring( id_t sid, pe::BodyID body1, const Vec3& anchor1, pe::BodyID body2, const Vec3& anchor2, real stiffness, real damping, real length, bool visible );
   friend SpringID attachSpring( pe::BodyID body1, const Vec3& anchor1, pe::BodyID body2, const Vec3& anchor2, real stiffness, real damping, real length, bool visible );
   friend void detach( pe::AttachableID attachable );
   // Joints register directly instead of the detour via a manager and thus need friend access to the joint storage.
   friend FixedJointID attachFixedJoint( pe::BodyID body1, const Vec3& anchor1, pe::BodyID body2, const Vec3& anchor2, real scale );
   friend void detach( pe::JointID joint );
   // Since the collision system is a private singleton theCollisionSystem() needs friend access to the instance() function.
   friend CollisionSystemID theCollisionSystem();
   pe_BEFRIEND_SINGLETON;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Constructor of the CollisionSystem class.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::CollisionSystem()
   : MPICommunication  ( processstorage_, domain_ )
   , Singleton<CollisionSystem,logging::Logger>()  // Initialization of the Singleton base object
   , contacts_         ( 5000 )                    // The currently active contacts of the simulation world
   , detector_         ( bodystorage_, bodystorageShadowCopies_ )  // The active collision detection algorithm
   , solver_           ( domain_ )                 // The active collision response algorithm
   , bodystorage_      ()
   , bodystorageShadowCopies_ ()
   , processstorage_   ()
   , domain_           ( processstorage_ )
   , attachablestorage_()
   , jointstorage_     ()
   , lastSyncTag_      ( mpitagDEMSynchronizePositionsAndVelocities2 )
   , requireSync_      ( false )
{
   // Registering all timers
   timers_.push_back( &timeCollisionDetection_ );
   timers_.push_back( &timeCollisionResponse_ );
   timers_.push_back( &timeBodySync_ );
   timers_.push_back( &timeBodySyncAssembling_ );
   timers_.push_back( &timeBodySyncCommunicate_ );
   timers_.push_back( &timeBodySyncParsing_ );
   timers_.push_back( &timeForceSync_ );
   timers_.push_back( &timeForceSyncAssembling_ );
   timers_.push_back( &timeForceSyncCommunicate_ );
   timers_.push_back( &timeForceSyncParsing_ );
   timers_.push_back( &timeForceSyncGlobals_ );
   timers_.push_back( &timeIntegration_ );
   timers_.push_back( &timeSimulationStep_ );

   // Registering all memory meters
   memoryMeters_.push_back( &memCollisionDetection_ );
   memoryMeters_.push_back( &memCollisionResponse_ );
   memoryMeters_.push_back( &memBodySync_ );
   memoryMeters_.push_back( &memBodySyncAssembling_ );
   memoryMeters_.push_back( &memBodySyncCommunicate_ );
   memoryMeters_.push_back( &memBodySyncParsing_ );
   memoryMeters_.push_back( &memForceSync_ );
   memoryMeters_.push_back( &memForceSyncAssembling_ );
   memoryMeters_.push_back( &memForceSyncCommunicate_ );
   memoryMeters_.push_back( &memForceSyncParsing_ );
   memoryMeters_.push_back( &memForceSyncGlobals_ );
   memoryMeters_.push_back( &memIntegration_ );
   memoryMeters_.push_back( &memSimulationStep_ );

   // Registering all sent meters
   sentMeters_.push_back( &sentOverall_ );
   sentMeters_.push_back( &sentBodySync_ );
   sentMeters_.push_back( &sentForceSync_ );

   // Registering all received meters
   receivedMeters_.push_back( &receivedOverall_ );
   receivedMeters_.push_back( &receivedBodySync_ );
   receivedMeters_.push_back( &receivedForceSync_ );

   // Logging the successful setup of the collision system
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully initialized the collision system instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor of the CollisionSystem class.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::~CollisionSystem()
{
   // Clearing the collision system
   clear();

   // Logging the successful destruction of the collision system
   pe_LOG_PROGRESS_SECTION( log ) {
      log << "Successfully destroyed the collision system instance";
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the currently active coarse collision detector.
 *
 * \return Reference to the active coarse collision detector.
 *
 * With this function it is possible to access the currently active coarse collision detector.
 * The following code shows an example of how to use this function to acquire a reference to the
 * coarse collision detector:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();
   pe::CoarseDetector& coarseDetector = collisionSystem->getCoarseDetector();
   \endcode
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::CoarseDetector&
   CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::getCoarseDetector()
{
   return detector_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the currently active contact solver.
 *
 * \return Reference to the active contact solver.
 *
 * With this function it is possible to access the currently active contact solver. The
 * following code shows an example of how to use this function to acquire a reference to
 * the contact solver:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();
   pe::ContactSolver& contactSolver = collisionSystem->getContactSolver();
   \endcode
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::ContactSolver&
   CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::getContactSolver()
{
   return solver_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the body storage.
 *
 * \return Reference to the body storage.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::BS&
   CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::getBodyStorage()
{
   return bodystorage_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a reference to the body shadow copy storage.
 *
 * \return Reference to the body shadow copy storage.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::BS&
   CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::getBodyShadowCopyStorage()
{
   return bodystorageShadowCopies_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant reference to the process storage.
 *
 * \return Constant reference to the process storage.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const typename CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::PS&
   CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::getProcessStorage() const
{
   return processstorage_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant reference to the attachable storage.
 *
 * \return Constant reference to the attachable storage.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const typename CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::AS&
   CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::getAttachableStorage() const
{
   return attachablestorage_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant reference to the domain of the local process.
 *
 * \return Constant reference to the domain of the local process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Domain& CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::getDomain() const
{
   return domain_;
}
//*************************************************************************************************




//=================================================================================================
//
//  QUERY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns if a synchronization is required before the next time step.
 *
 * \return True if a synchronization is strictly required, false if a synchronization is possibly
 *         required for correct dynamics but not enforced.
 *
 * Insertion or removal of bodies from the simulation \em can require a subsequent synchronization
 * before performing the next time step. If e.g. no other process obtained a shadow copy of a
 * body to be removed then a synchronization is \em not enforced. However, if a neighbor has a
 * shadow copy a synchronization is required. Changing e.g. velocities or positions can lead to
 * inconsistent descriptions of bodies across process boundaries but synchronization is not
 * enforced. In this case it is the users obligation to synchronize whenever necessary.
 *
 * WARNING: This query function uses an expensive allreduce MPI operation to determine the
 * result!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::isSyncRequired() const
{
#if HAVE_MPI
   // No synchronization ever necessary if we compute on a single process
   if( MPISettings::size() <= 1 )
      return false;

   char requireSync( requireSync_ );
   int  n( 1 );

   MPI_Allreduce( MPI_IN_PLACE, &requireSync, n, MPITrait<char>::getType(), MPI_LOR, MPISettings::comm() );
   return static_cast<bool>( requireSync );
#else
   return false;
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns if a synchronization is required by the local process before the next time step.
 *
 * \return True if a synchronization is strictly required by the local process, false otherwise.
 *
 * Insertion or removal of bodies from the simulation \em can require a subsequent synchronization
 * before performing the next time step. If e.g. no other process obtained a shadow copy of a
 * body to be removed then a synchronization is \em not enforced. However, if a neighbor has a
 * shadow copy a synchronization is required. Changing e.g. velocities or positions can lead to
 * inconsistent descriptions of bodies across process boundaries but synchronization is not
 * enforced. In this case it is the users obligation to synchronize whenever necessary.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::isSyncRequiredLocally() const
{
   return requireSync_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Writes a profiling summary into the log file.
 *
 * \return void
 *
 * In case the profiling mode is enabled and the log level is at least info this function writes a
 * summary of profiling information to the log files. The information is reduced over all previous
 * time steps and in a separate compilation over all time steps and all ranks.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::logProfilingSummary() const
{
   // Logging the profiling results
   pe_PROFILING_SECTION {
      if( logging::loglevel >= logging::info ) {
         std::vector<double> minValues( timers_.size() ), maxValues( timers_.size() ), totalValues( timers_.size() );
         std::vector<size_t> numValues( timers_.size() );
         for( std::size_t i = 0; i < minValues.size(); ++i ) {
            minValues[i]   = timers_[i]->min();
            maxValues[i]   = timers_[i]->max();
            totalValues[i] = timers_[i]->total();
            numValues[i]   = timers_[i]->getCounter();
         }

         pe_LOG_INFO_SECTION( log ) {
            log << "Timing results reduced over all time steps on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              min time     max time     avg time     total time   executions\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------\n"
                << "simulation step        "   << std::setw(10) << minValues[12] << "   " << std::setw(10) << maxValues[12] << "   " << std::setw(10) << totalValues[12] / numValues[12] << " = " << std::setw(10) << totalValues[12] << " / " << std::setw(10) << numValues[12] << "\n"
                << "  col. detection       "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "\n"
                << "  col. response        "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "\n"
                << "  body sync            "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "\n"
                << "    assembling         "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "\n"
                << "    communicate        "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "\n"
                << "    parsing            "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "\n"
                << "  force sync           "   << std::setw(10) << minValues[ 6] << "   " << std::setw(10) << maxValues[ 6] << "   " << std::setw(10) << totalValues[ 6] / numValues[ 6] << " = " << std::setw(10) << totalValues[ 6] << " / " << std::setw(10) << numValues[ 6] << "\n"
                << "    assembling         "   << std::setw(10) << minValues[ 7] << "   " << std::setw(10) << maxValues[ 7] << "   " << std::setw(10) << totalValues[ 7] / numValues[ 7] << " = " << std::setw(10) << totalValues[ 7] << " / " << std::setw(10) << numValues[ 7] << "\n"
                << "    communicate        "   << std::setw(10) << minValues[ 8] << "   " << std::setw(10) << maxValues[ 8] << "   " << std::setw(10) << totalValues[ 8] / numValues[ 8] << " = " << std::setw(10) << totalValues[ 8] << " / " << std::setw(10) << numValues[ 8] << "\n"
                << "    parsing            "   << std::setw(10) << minValues[ 9] << "   " << std::setw(10) << maxValues[ 9] << "   " << std::setw(10) << totalValues[ 9] / numValues[ 9] << " = " << std::setw(10) << totalValues[ 9] << " / " << std::setw(10) << numValues[ 9] << "\n"
                << "    globals            "   << std::setw(10) << minValues[10] << "   " << std::setw(10) << maxValues[10] << "   " << std::setw(10) << totalValues[10] / numValues[10] << " = " << std::setw(10) << totalValues[10] << " / " << std::setw(10) << numValues[10] << "\n"
                << "  time integration     "   << std::setw(10) << minValues[11] << "   " << std::setw(10) << maxValues[11] << "   " << std::setw(10) << totalValues[11] / numValues[11] << " = " << std::setw(10) << totalValues[11] << " / " << std::setw(10) << numValues[11] << "\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
         }

         // Logging the time profiling results reduced over all ranks for MPI parallel simulations
         pe_MPI_SECTION {
            const int          root( MPISettings::root() );
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );

            // Reduce the minimum/maximum/total time measurement and number of measurements of each timer over all time steps and all ranks
            if( rank == root ) {
               MPI_Reduce( MPI_IN_PLACE, &minValues[0],   minValues.size(),   MPITrait<double>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &maxValues[0],   maxValues.size(),   MPITrait<double>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &totalValues[0], totalValues.size(), MPITrait<double>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &numValues[0],   numValues.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }
            else {
               MPI_Reduce( &minValues[0],   0, minValues.size(),   MPITrait<double>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( &maxValues[0],   0, maxValues.size(),   MPITrait<double>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( &totalValues[0], 0, totalValues.size(), MPITrait<double>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( &numValues[0],   0, numValues.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }


            pe_ROOT_SECTION {
               pe_LOG_INFO_SECTION( log ) {
                  log << "Timing results reduced over all time steps and ranks:\n"
                      << "code part              min time     max time     avg time     total time   executions\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------\n" << std::fixed << std::setprecision(4)
                      << "simulation step        "   << std::setw(10) << minValues[12] << "   " << std::setw(10) << maxValues[12] << "   " << std::setw(10) << totalValues[12] / numValues[12] << " = " << std::setw(10) << totalValues[12] << " / " << std::setw(10) << numValues[12] << "\n"
                      << "  col. detection       "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "\n"
                      << "  col. response        "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "\n"
                      << "  body sync            "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "\n"
                      << "    assembling         "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "\n"
                      << "    communicate        "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "\n"
                      << "    parsing            "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "\n"
                      << "  force sync           "   << std::setw(10) << minValues[ 6] << "   " << std::setw(10) << maxValues[ 6] << "   " << std::setw(10) << totalValues[ 6] / numValues[ 6] << " = " << std::setw(10) << totalValues[ 6] << " / " << std::setw(10) << numValues[ 6] << "\n"
                      << "    assembling         "   << std::setw(10) << minValues[ 7] << "   " << std::setw(10) << maxValues[ 7] << "   " << std::setw(10) << totalValues[ 7] / numValues[ 7] << " = " << std::setw(10) << totalValues[ 7] << " / " << std::setw(10) << numValues[ 7] << "\n"
                      << "    communicate        "   << std::setw(10) << minValues[ 8] << "   " << std::setw(10) << maxValues[ 8] << "   " << std::setw(10) << totalValues[ 8] / numValues[ 8] << " = " << std::setw(10) << totalValues[ 8] << " / " << std::setw(10) << numValues[ 8] << "\n"
                      << "    parsing            "   << std::setw(10) << minValues[ 9] << "   " << std::setw(10) << maxValues[ 9] << "   " << std::setw(10) << totalValues[ 9] / numValues[ 9] << " = " << std::setw(10) << totalValues[ 9] << " / " << std::setw(10) << numValues[ 9] << "\n"
                      << "    globals            "   << std::setw(10) << minValues[10] << "   " << std::setw(10) << maxValues[10] << "   " << std::setw(10) << totalValues[10] / numValues[10] << " = " << std::setw(10) << totalValues[10] << " / " << std::setw(10) << numValues[10] << "\n"
                      << "  time integration     "   << std::setw(10) << minValues[11] << "   " << std::setw(10) << maxValues[11] << "   " << std::setw(10) << totalValues[11] / numValues[11] << " = " << std::setw(10) << totalValues[11] << " / " << std::setw(10) << numValues[11] << "\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
               }
            }
         }

         std::vector<int64_t> minAllocValues( memoryMeters_.size() ), maxAllocValues( memoryMeters_.size() ), totalAllocValues( memoryMeters_.size() ), minInUseValues( memoryMeters_.size() ), maxInUseValues( memoryMeters_.size() ), totalInUseValues( memoryMeters_.size() );
         std::vector<size_t> numAllocValues( memoryMeters_.size() );
         for( std::size_t i = 0; i < memoryMeters_.size(); ++i ) {
            minAllocValues[i]   = memoryMeters_[i]->minAllocation();
            maxAllocValues[i]   = memoryMeters_[i]->maxAllocation();
            totalAllocValues[i] = memoryMeters_[i]->totalAllocation();
            minInUseValues[i]   = memoryMeters_[i]->minInUse();
            maxInUseValues[i]   = memoryMeters_[i]->maxInUse();
            totalInUseValues[i] = memoryMeters_[i]->totalInUse();
            numAllocValues[i]   = memoryMeters_[i]->getCounter();
         }

         pe_LOG_INFO_SECTION( log ) {
            log << "Memory metering results reduced over all time steps on current process:\n"
                << "code part              min alloc    max alloc    sum alloc    min in use   max in use   sum in use   executions\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------   ----------\n" << std::fixed << std::setprecision(4)
                << "simulation step        "   << std::setw(10) << minAllocValues[12] << "   " << std::setw(10) << maxAllocValues[12] << "   " << std::setw(10) << totalAllocValues[12] << "   " << std::setw(10) << minInUseValues[12] << "   " << std::setw(10) << maxInUseValues[12] << "   " << std::setw(10) << totalInUseValues[12] << "   " << std::setw(10) << numAllocValues[12] << "\n"
                << "  collision detection  "   << std::setw(10) << minAllocValues[ 0] << "   " << std::setw(10) << maxAllocValues[ 0] << "   " << std::setw(10) << totalAllocValues[ 0] << "   " << std::setw(10) << minInUseValues[ 0] << "   " << std::setw(10) << maxInUseValues[ 0] << "   " << std::setw(10) << totalInUseValues[ 0] << "   " << std::setw(10) << numAllocValues[ 0] << "\n"
                << "  collision response   "   << std::setw(10) << minAllocValues[ 1] << "   " << std::setw(10) << maxAllocValues[ 1] << "   " << std::setw(10) << totalAllocValues[ 1] << "   " << std::setw(10) << minInUseValues[ 1] << "   " << std::setw(10) << maxInUseValues[ 1] << "   " << std::setw(10) << totalInUseValues[ 1] << "   " << std::setw(10) << numAllocValues[ 1] << "\n"
                << "  body sync            "   << std::setw(10) << minAllocValues[ 2] << "   " << std::setw(10) << maxAllocValues[ 2] << "   " << std::setw(10) << totalAllocValues[ 2] << "   " << std::setw(10) << minInUseValues[ 2] << "   " << std::setw(10) << maxInUseValues[ 2] << "   " << std::setw(10) << totalInUseValues[ 2] << "   " << std::setw(10) << numAllocValues[ 2] << "\n"
                << "    assembling         "   << std::setw(10) << minAllocValues[ 3] << "   " << std::setw(10) << maxAllocValues[ 3] << "   " << std::setw(10) << totalAllocValues[ 3] << "   " << std::setw(10) << minInUseValues[ 3] << "   " << std::setw(10) << maxInUseValues[ 3] << "   " << std::setw(10) << totalInUseValues[ 3] << "   " << std::setw(10) << numAllocValues[ 3] << "\n"
                << "    communicate        "   << std::setw(10) << minAllocValues[ 4] << "   " << std::setw(10) << maxAllocValues[ 4] << "   " << std::setw(10) << totalAllocValues[ 4] << "   " << std::setw(10) << minInUseValues[ 4] << "   " << std::setw(10) << maxInUseValues[ 4] << "   " << std::setw(10) << totalInUseValues[ 4] << "   " << std::setw(10) << numAllocValues[ 4] << "\n"
                << "    parsing            "   << std::setw(10) << minAllocValues[ 5] << "   " << std::setw(10) << maxAllocValues[ 5] << "   " << std::setw(10) << totalAllocValues[ 5] << "   " << std::setw(10) << minInUseValues[ 5] << "   " << std::setw(10) << maxInUseValues[ 5] << "   " << std::setw(10) << totalInUseValues[ 5] << "   " << std::setw(10) << numAllocValues[ 5] << "\n"
                << "  force sync           "   << std::setw(10) << minAllocValues[ 6] << "   " << std::setw(10) << maxAllocValues[ 6] << "   " << std::setw(10) << totalAllocValues[ 6] << "   " << std::setw(10) << minInUseValues[ 6] << "   " << std::setw(10) << maxInUseValues[ 6] << "   " << std::setw(10) << totalInUseValues[ 6] << "   " << std::setw(10) << numAllocValues[ 6] << "\n"
                << "    assembling         "   << std::setw(10) << minAllocValues[ 7] << "   " << std::setw(10) << maxAllocValues[ 7] << "   " << std::setw(10) << totalAllocValues[ 7] << "   " << std::setw(10) << minInUseValues[ 7] << "   " << std::setw(10) << maxInUseValues[ 7] << "   " << std::setw(10) << totalInUseValues[ 7] << "   " << std::setw(10) << numAllocValues[ 7] << "\n"
                << "    communicate        "   << std::setw(10) << minAllocValues[ 8] << "   " << std::setw(10) << maxAllocValues[ 8] << "   " << std::setw(10) << totalAllocValues[ 8] << "   " << std::setw(10) << minInUseValues[ 8] << "   " << std::setw(10) << maxInUseValues[ 8] << "   " << std::setw(10) << totalInUseValues[ 8] << "   " << std::setw(10) << numAllocValues[ 8] << "\n"
                << "    parsing            "   << std::setw(10) << minAllocValues[ 9] << "   " << std::setw(10) << maxAllocValues[ 9] << "   " << std::setw(10) << totalAllocValues[ 9] << "   " << std::setw(10) << minInUseValues[ 9] << "   " << std::setw(10) << maxInUseValues[ 9] << "   " << std::setw(10) << totalInUseValues[ 9] << "   " << std::setw(10) << numAllocValues[ 9] << "\n"
                << "    globals            "   << std::setw(10) << minAllocValues[10] << "   " << std::setw(10) << maxAllocValues[10] << "   " << std::setw(10) << totalAllocValues[10] << "   " << std::setw(10) << minInUseValues[10] << "   " << std::setw(10) << maxInUseValues[10] << "   " << std::setw(10) << totalInUseValues[10] << "   " << std::setw(10) << numAllocValues[10] << "\n"
                << "  time integration     "   << std::setw(10) << minAllocValues[11] << "   " << std::setw(10) << maxAllocValues[11] << "   " << std::setw(10) << totalAllocValues[11] << "   " << std::setw(10) << minInUseValues[11] << "   " << std::setw(10) << maxInUseValues[11] << "   " << std::setw(10) << totalInUseValues[11] << "   " << std::setw(10) << numAllocValues[11] << "\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------   ----------\n";
         }

         // Logging the memory profiling results reduced over all ranks for MPI parallel simulations
         pe_MPI_SECTION {
            const int          root( MPISettings::root() );
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );

            // Reduce the minimum/maximum/total memory allocation measurement (from system and by application) and number of measurements of each memory meter over all time steps and all ranks
            if( rank == root ) {
               MPI_Reduce( MPI_IN_PLACE, &minAllocValues[0],   minAllocValues.size(),   MPITrait<int64_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &maxAllocValues[0],   maxAllocValues.size(),   MPITrait<int64_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &totalAllocValues[0], totalAllocValues.size(), MPITrait<int64_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &minInUseValues[0],   minInUseValues.size(),   MPITrait<int64_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &maxInUseValues[0],   maxInUseValues.size(),   MPITrait<int64_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &totalInUseValues[0], totalInUseValues.size(), MPITrait<int64_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &numAllocValues[0],   numAllocValues.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }
            else {
               MPI_Reduce( &minAllocValues[0],   0, minAllocValues.size(),   MPITrait<int64_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( &maxAllocValues[0],   0, maxAllocValues.size(),   MPITrait<int64_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( &totalAllocValues[0], 0, totalAllocValues.size(), MPITrait<int64_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( &minInUseValues[0],   0, minInUseValues.size(),   MPITrait<int64_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( &maxInUseValues[0],   0, maxInUseValues.size(),   MPITrait<int64_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( &totalInUseValues[0], 0, totalInUseValues.size(), MPITrait<int64_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( &numAllocValues[0],   0, numAllocValues.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }

            pe_ROOT_SECTION {
               pe_LOG_INFO_SECTION( log ) {
                  log << "Memory metering results reduced over all time steps and ranks:\n"
                      << "code part              min alloc    max alloc    sum alloc    min in use   max in use   sum in use   executions\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------   ----------\n" << std::fixed << std::setprecision(4)
                      << "simulation step        "   << std::setw(10) << minAllocValues[12] << "   " << std::setw(10) << maxAllocValues[12] << "   " << std::setw(10) << totalAllocValues[12] << "   " << std::setw(10) << minInUseValues[12] << "   " << std::setw(10) << maxInUseValues[12] << "   " << std::setw(10) << totalInUseValues[12] << "   " << std::setw(10) << numAllocValues[12] << "\n"
                      << "  collision detection  "   << std::setw(10) << minAllocValues[ 0] << "   " << std::setw(10) << maxAllocValues[ 0] << "   " << std::setw(10) << totalAllocValues[ 0] << "   " << std::setw(10) << minInUseValues[ 0] << "   " << std::setw(10) << maxInUseValues[ 0] << "   " << std::setw(10) << totalInUseValues[ 0] << "   " << std::setw(10) << numAllocValues[ 0] << "\n"
                      << "  collision response   "   << std::setw(10) << minAllocValues[ 1] << "   " << std::setw(10) << maxAllocValues[ 1] << "   " << std::setw(10) << totalAllocValues[ 1] << "   " << std::setw(10) << minInUseValues[ 1] << "   " << std::setw(10) << maxInUseValues[ 1] << "   " << std::setw(10) << totalInUseValues[ 1] << "   " << std::setw(10) << numAllocValues[ 1] << "\n"
                      << "  body sync            "   << std::setw(10) << minAllocValues[ 2] << "   " << std::setw(10) << maxAllocValues[ 2] << "   " << std::setw(10) << totalAllocValues[ 2] << "   " << std::setw(10) << minInUseValues[ 2] << "   " << std::setw(10) << maxInUseValues[ 2] << "   " << std::setw(10) << totalInUseValues[ 2] << "   " << std::setw(10) << numAllocValues[ 2] << "\n"
                      << "    assembling         "   << std::setw(10) << minAllocValues[ 3] << "   " << std::setw(10) << maxAllocValues[ 3] << "   " << std::setw(10) << totalAllocValues[ 3] << "   " << std::setw(10) << minInUseValues[ 3] << "   " << std::setw(10) << maxInUseValues[ 3] << "   " << std::setw(10) << totalInUseValues[ 3] << "   " << std::setw(10) << numAllocValues[ 3] << "\n"
                      << "    communicate        "   << std::setw(10) << minAllocValues[ 4] << "   " << std::setw(10) << maxAllocValues[ 4] << "   " << std::setw(10) << totalAllocValues[ 4] << "   " << std::setw(10) << minInUseValues[ 4] << "   " << std::setw(10) << maxInUseValues[ 4] << "   " << std::setw(10) << totalInUseValues[ 4] << "   " << std::setw(10) << numAllocValues[ 4] << "\n"
                      << "    parsing            "   << std::setw(10) << minAllocValues[ 5] << "   " << std::setw(10) << maxAllocValues[ 5] << "   " << std::setw(10) << totalAllocValues[ 5] << "   " << std::setw(10) << minInUseValues[ 5] << "   " << std::setw(10) << maxInUseValues[ 5] << "   " << std::setw(10) << totalInUseValues[ 5] << "   " << std::setw(10) << numAllocValues[ 5] << "\n"
                      << "  force sync           "   << std::setw(10) << minAllocValues[ 6] << "   " << std::setw(10) << maxAllocValues[ 6] << "   " << std::setw(10) << totalAllocValues[ 6] << "   " << std::setw(10) << minInUseValues[ 6] << "   " << std::setw(10) << maxInUseValues[ 6] << "   " << std::setw(10) << totalInUseValues[ 6] << "   " << std::setw(10) << numAllocValues[ 6] << "\n"
                      << "    assembling         "   << std::setw(10) << minAllocValues[ 7] << "   " << std::setw(10) << maxAllocValues[ 7] << "   " << std::setw(10) << totalAllocValues[ 7] << "   " << std::setw(10) << minInUseValues[ 7] << "   " << std::setw(10) << maxInUseValues[ 7] << "   " << std::setw(10) << totalInUseValues[ 7] << "   " << std::setw(10) << numAllocValues[ 7] << "\n"
                      << "    communicate        "   << std::setw(10) << minAllocValues[ 8] << "   " << std::setw(10) << maxAllocValues[ 8] << "   " << std::setw(10) << totalAllocValues[ 8] << "   " << std::setw(10) << minInUseValues[ 8] << "   " << std::setw(10) << maxInUseValues[ 8] << "   " << std::setw(10) << totalInUseValues[ 8] << "   " << std::setw(10) << numAllocValues[ 8] << "\n"
                      << "    parsing            "   << std::setw(10) << minAllocValues[ 9] << "   " << std::setw(10) << maxAllocValues[ 9] << "   " << std::setw(10) << totalAllocValues[ 9] << "   " << std::setw(10) << minInUseValues[ 9] << "   " << std::setw(10) << maxInUseValues[ 9] << "   " << std::setw(10) << totalInUseValues[ 9] << "   " << std::setw(10) << numAllocValues[ 9] << "\n"
                      << "    globals            "   << std::setw(10) << minAllocValues[10] << "   " << std::setw(10) << maxAllocValues[10] << "   " << std::setw(10) << totalAllocValues[10] << "   " << std::setw(10) << minInUseValues[10] << "   " << std::setw(10) << maxInUseValues[10] << "   " << std::setw(10) << totalInUseValues[10] << "   " << std::setw(10) << numAllocValues[10] << "\n"
                      << "  time integration     "   << std::setw(10) << minAllocValues[11] << "   " << std::setw(10) << maxAllocValues[11] << "   " << std::setw(10) << totalAllocValues[11] << "   " << std::setw(10) << minInUseValues[11] << "   " << std::setw(10) << maxInUseValues[11] << "   " << std::setw(10) << totalInUseValues[11] << "   " << std::setw(10) << numAllocValues[11] << "\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------   ----------   ----------\n";
               }
            }
         }

         std::vector<size_t> sentMinBytes( sentMeters_.size() ), sentMaxBytes( sentMeters_.size() ), sentTotalBytes( sentMeters_.size() ), sentNumTransfers( sentMeters_.size() );
         for( std::size_t i = 0; i < sentMeters_.size(); ++i ) {
            sentMinBytes[i]     = sentMeters_[i]->minBytes();
            sentMaxBytes[i]     = sentMeters_[i]->maxBytes();
            sentTotalBytes[i]   = sentMeters_[i]->totalBytes();
            sentNumTransfers[i] = sentMeters_[i]->getNumberOfTransfers();
         }

         pe_LOG_INFO_SECTION( log ) {
            log << "Outgoing transfer results reduced over all time steps on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              min bytes    max bytes    avg bytes    total bytes  transfers\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------\n"
                << "collision system       "   << std::setw(10) << sentMinBytes[0] << "   " << std::setw(10) << sentMaxBytes[0] << "   " << std::setw(10) << (sentNumTransfers[0] == 0 ? 0 : sentTotalBytes[0] / sentNumTransfers[0]) << " = " << std::setw(10) << sentTotalBytes[0] << " / " << std::setw(10) << sentNumTransfers[0] << "\n"
                << "  body sync            "   << std::setw(10) << sentMinBytes[1] << "   " << std::setw(10) << sentMaxBytes[1] << "   " << std::setw(10) << (sentNumTransfers[1] == 0 ? 0 : sentTotalBytes[1] / sentNumTransfers[1]) << " = " << std::setw(10) << sentTotalBytes[1] << " / " << std::setw(10) << sentNumTransfers[1] << "\n"
                << "  force sync           "   << std::setw(10) << sentMinBytes[2] << "   " << std::setw(10) << sentMaxBytes[2] << "   " << std::setw(10) << (sentNumTransfers[2] == 0 ? 0 : sentTotalBytes[2] / sentNumTransfers[2]) << " = " << std::setw(10) << sentTotalBytes[2] << " / " << std::setw(10) << sentNumTransfers[2] << "\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
         }

         // Logging the outgoing transfer profiling results reduced over all ranks for MPI parallel simulations
         pe_MPI_SECTION {
            const int          root( MPISettings::root() );
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );

            // Reduce the minimum/maximum/total bytes transfered and number of transfers over all time steps and all ranks
            if( rank == root ) {
               MPI_Reduce( MPI_IN_PLACE, &sentMinBytes[0],     sentMinBytes.size(),     MPITrait<size_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &sentMaxBytes[0],     sentMaxBytes.size(),     MPITrait<size_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &sentTotalBytes[0],   sentTotalBytes.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &sentNumTransfers[0], sentNumTransfers.size(), MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }
            else {
               MPI_Reduce( &sentMinBytes[0],     0, sentMinBytes.size(),     MPITrait<size_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( &sentMaxBytes[0],     0, sentMaxBytes.size(),     MPITrait<size_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( &sentTotalBytes[0],   0, sentTotalBytes.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( &sentNumTransfers[0], 0, sentNumTransfers.size(), MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }

            pe_ROOT_SECTION {
               pe_LOG_INFO_SECTION( log ) {
                  log << "Outgoing transfer results reduced over all time steps and ranks:\n" << std::fixed << std::setprecision(4)
                      << "code part              min bytes    max bytes    avg bytes    total bytes  transfers\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------\n"
                      << "collision system       "   << std::setw(10) << sentMinBytes[0] << "   " << std::setw(10) << sentMaxBytes[0] << "   " << std::setw(10) << (sentNumTransfers[0] == 0 ? 0 : sentTotalBytes[0] / sentNumTransfers[0]) << " = " << std::setw(10) << sentTotalBytes[0] << " / " << std::setw(10) << sentNumTransfers[0] << "\n"
                      << "  body sync            "   << std::setw(10) << sentMinBytes[1] << "   " << std::setw(10) << sentMaxBytes[1] << "   " << std::setw(10) << (sentNumTransfers[1] == 0 ? 0 : sentTotalBytes[1] / sentNumTransfers[1]) << " = " << std::setw(10) << sentTotalBytes[1] << " / " << std::setw(10) << sentNumTransfers[1] << "\n"
                      << "  force sync           "   << std::setw(10) << sentMinBytes[2] << "   " << std::setw(10) << sentMaxBytes[2] << "   " << std::setw(10) << (sentNumTransfers[2] == 0 ? 0 : sentTotalBytes[2] / sentNumTransfers[2]) << " = " << std::setw(10) << sentTotalBytes[2] << " / " << std::setw(10) << sentNumTransfers[2] << "\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
               }
            }
         }

         std::vector<size_t> receivedMinBytes( receivedMeters_.size() ), receivedMaxBytes( receivedMeters_.size() ), receivedTotalBytes( receivedMeters_.size() ), receivedNumTransfers( receivedMeters_.size() );
         for( std::size_t i = 0; i < receivedMeters_.size(); ++i ) {
            receivedMinBytes[i]     = receivedMeters_[i]->minBytes();
            receivedMaxBytes[i]     = receivedMeters_[i]->maxBytes();
            receivedTotalBytes[i]   = receivedMeters_[i]->totalBytes();
            receivedNumTransfers[i] = receivedMeters_[i]->getNumberOfTransfers();
         }

         pe_LOG_INFO_SECTION( log ) {
            log << "Incoming transfer results reduced over all time steps on current process:\n" << std::fixed << std::setprecision(4)
                << "code part              min bytes    max bytes    avg bytes    total bytes  transfers\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------\n"
                << "collision system       "   << std::setw(10) << receivedMinBytes[0] << "   " << std::setw(10) << receivedMaxBytes[0] << "   " << std::setw(10) << (receivedNumTransfers[0] == 0 ? 0 : receivedTotalBytes[0] / receivedNumTransfers[0]) << " = " << std::setw(10) << receivedTotalBytes[0] << " / " << std::setw(10) << receivedNumTransfers[0] << "\n"
                << "  body sync            "   << std::setw(10) << receivedMinBytes[1] << "   " << std::setw(10) << receivedMaxBytes[1] << "   " << std::setw(10) << (receivedNumTransfers[1] == 0 ? 0 : receivedTotalBytes[1] / receivedNumTransfers[1]) << " = " << std::setw(10) << receivedTotalBytes[1] << " / " << std::setw(10) << receivedNumTransfers[1] << "\n"
                << "  force sync           "   << std::setw(10) << receivedMinBytes[2] << "   " << std::setw(10) << receivedMaxBytes[2] << "   " << std::setw(10) << (receivedNumTransfers[2] == 0 ? 0 : receivedTotalBytes[2] / receivedNumTransfers[2]) << " = " << std::setw(10) << receivedTotalBytes[2] << " / " << std::setw(10) << receivedNumTransfers[2] << "\n"
                << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
         }

         // Logging the incoming transfer profiling results reduced over all ranks for MPI parallel simulations
         pe_MPI_SECTION {
            const int          root( MPISettings::root() );
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );

            // Reduce the minimum/maximum/total bytes transfered and number of transfers over all time steps and all ranks
            if( rank == root ) {
               MPI_Reduce( MPI_IN_PLACE, &receivedMinBytes[0],     receivedMinBytes.size(),     MPITrait<size_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &receivedMaxBytes[0],     receivedMaxBytes.size(),     MPITrait<size_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &receivedTotalBytes[0],   receivedTotalBytes.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &receivedNumTransfers[0], receivedNumTransfers.size(), MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }
            else {
               MPI_Reduce( &receivedMinBytes[0],     0, receivedMinBytes.size(),     MPITrait<size_t>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( &receivedMaxBytes[0],     0, receivedMaxBytes.size(),     MPITrait<size_t>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( &receivedTotalBytes[0],   0, receivedTotalBytes.size(),   MPITrait<size_t>::getType(), MPI_SUM, root, comm );
               MPI_Reduce( &receivedNumTransfers[0], 0, receivedNumTransfers.size(), MPITrait<size_t>::getType(), MPI_SUM, root, comm );
            }

            pe_ROOT_SECTION {
               pe_LOG_INFO_SECTION( log ) {
                  log << "Incoming transfer results reduced over all time steps and ranks:\n" << std::fixed << std::setprecision(4)
                      << "code part              min bytes    max bytes    avg bytes    total bytes  transfers\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------\n"
                      << "collision system       "   << std::setw(10) << receivedMinBytes[0] << "   " << std::setw(10) << receivedMaxBytes[0] << "   " << std::setw(10) << (receivedNumTransfers[0] == 0 ? 0 : receivedTotalBytes[0] / receivedNumTransfers[0]) << " = " << std::setw(10) << receivedTotalBytes[0] << " / " << std::setw(10) << receivedNumTransfers[0] << "\n"
                      << "  body sync            "   << std::setw(10) << receivedMinBytes[1] << "   " << std::setw(10) << receivedMaxBytes[1] << "   " << std::setw(10) << (receivedNumTransfers[1] == 0 ? 0 : receivedTotalBytes[1] / receivedNumTransfers[1]) << " = " << std::setw(10) << receivedTotalBytes[1] << " / " << std::setw(10) << receivedNumTransfers[1] << "\n"
                      << "  force sync           "   << std::setw(10) << receivedMinBytes[2] << "   " << std::setw(10) << receivedMaxBytes[2] << "   " << std::setw(10) << (receivedNumTransfers[2] == 0 ? 0 : receivedTotalBytes[2] / receivedNumTransfers[2]) << " = " << std::setw(10) << receivedTotalBytes[2] << " / " << std::setw(10) << receivedNumTransfers[2] << "\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
               }
            }
         }
      }
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  ADD/REMOVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Adding a rigid body to the collision system.
 *
 * \param body The rigid body to be added to the collision system.
 * \return void
 *
 * This function adds the rigid body to the collision detector and the central body storage.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::add( BodyID body )
{
   pe_INTERNAL_ASSERT( !body->isRemote(), "Trying to add remote body to central body storage." );

   // Keep track of global non-fixed bodies.
   if( body->isGlobal() && !body->isFixed() ) {
      globalNonfixedBodies_.insert( body );
   }

   // Unless the body is marked as remote we will set the owner to our rank here. For global bodies ownership is meaningless.
   if( !body->isGlobal() && !body->isRemote() ) {
      if( MPISettings::size() > 1 )
         // TODO This can be done better: If the body is completely inside the local domain we should not enforce a sync.
         requireSync_ = true;

      body->setOwner( MPISettings::rank(), 0 );
   }

   detector_.add( body );
   bodystorage_.add( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the collision system.
 *
 * \param body The rigid body to be removed from the collision system.
 * \return void
 *
 * This function removes the rigid body from the collision detector and the central body storage
 * but does not destroy it.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::remove( BodyID body )
{
   pe_INTERNAL_ASSERT( !body->isRemote(), "Trying to remove remote body from central body storage." );

   // Keep track of global non-fixed bodies.
   if( body->isGlobal() && !body->isFixed() ) {
      bool found( globalNonfixedBodies_.erase( body ) > 0 );

      UNUSED( found );
      pe_INTERNAL_ASSERT( found, "Removing global non-fixed body which has not been registered." );
   }

   if( !body->isGlobal() && !body->isRemote() && body->sizeProcesses() != 0 ) {
      // Notify registered processes (intersecting or interacting) of body removal since they possess a shadow copy.
      for( ProcessIterator it = body->beginProcesses(); it != body->endProcesses(); ++it ) {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "__Notify registered process " << it->getRank() << " of deletion of body " << body->getSystemID() << ".\n";
         }
         marshal( it->getSendBuffer(), notificationType<RigidBodyDeletionNotification>() );
         marshal( it->getSendBuffer(), RigidBodyDeletionNotification( *body ) );
      }

      requireSync_ = true;
   }

   detector_.remove( body );
   bodystorage_.remove( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the collision system.
 *
 * \param body An iterator pointing to the rigid body to be removed from the collision system.
 * \return An iterator pointing to the rigid body after the erase body.
 *
 * This function removes the rigid body from the collision detector and the central body storage
 * but does not destroy it.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::BodyIterator CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::remove( BodyIterator body )
{
   pe_INTERNAL_ASSERT( !body->isRemote(), "Trying to remove remote body from central body storage." );

   // Keep track of global non-fixed bodies.
   if( body->isGlobal() && !body->isFixed() ) {
      bool found( globalNonfixedBodies_.erase( *body ) > 0 );

      UNUSED( found );
      pe_INTERNAL_ASSERT( found, "Removing global non-fixed body which has not been registered." );
   }

   if( !body->isGlobal() && !body->isRemote() && body->sizeProcesses() != 0 ) {
      // Notify registered processes (intersecting or interacting) of body removal since they possess a shadow copy.
      for( ProcessIterator it = body->beginProcesses(); it != body->endProcesses(); ++it ) {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "__Notify registered process " << it->getRank() << " of deletion of body " << body->getSystemID() << ".\n";
         }
         marshal( it->getSendBuffer(), notificationType<RigidBodyDeletionNotification>() );
         marshal( it->getSendBuffer(), RigidBodyDeletionNotification( *( *body ) ) );
      }

      requireSync_ = true;
   }

   detector_.remove( *body );
   return bodystorage_.remove( body );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Removing a rigid body from the collision detector.
 *
 * \param body The rigid body to be removed from the collision detector.
 * \return void
 *
 * This function removes the rigid body from the collision detector.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::removeFromCollisionDetector( BodyID body )
{
   detector_.remove( body );
}
//*************************************************************************************************





//=================================================================================================
//
//  SIMULATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of the discrete element solver simulationStep() function.
 *
 * \param timestep Size of the time step.
 * \return void
 *
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::simulationStep( real dt )
{
   pe_USER_ASSERT( !requireSync_, "Simulation requires synchronization before continuing." );

   pe_PROFILING_SECTION {
#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeSimulationStep_.start();
      memSimulationStep_.start();
      timeCollisionDetection_.start();
      memCollisionDetection_.start();
   }

   // Detect all collisions
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Detecting contacts...\n";
   }

   detector_.findContacts( contacts_ );

   pe_PROFILING_SECTION {
      timeCollisionDetection_.end();
      memCollisionDetection_.stop();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeCollisionResponse_.start();
      memCollisionResponse_.start();
   }

   // Compute collision response forces and synchronize them
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Resolving contacts...\n";
   }

   real maxOverlap_( 0.0 );
   for( typename Contacts::Iterator c=contacts_.begin(); c!=contacts_.end(); ++c ) {
      const real overlap( solver_.resolveContact( *c ) );
      if( overlap > maxOverlap_ )
         maxOverlap_ = overlap;
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Maximum overlap is " << maxOverlap_ << ".\n";
   }

   clearContacts();

   pe_PROFILING_SECTION {
      timeCollisionResponse_.end();
      memCollisionResponse_.stop();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif
   }

   synchronizeForces();

   pe_PROFILING_SECTION {
      timeIntegration_.start();
      memIntegration_.start();
   }

   // Updating the positions and velocities of all locally owned and global rigid bodies (but not shadow copies).
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Time integration starts...\n";
   }

   for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "Time integration of body with system id " << body->getSystemID() << "\n";
         log << *body << "\n";
      }

      body->move( dt );
   }

   // Reset forces of shadow copies
   for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body ) {
      body->clearContacts();
      body->resetForce();
   }

   pe_PROFILING_SECTION {
      timeIntegration_.end();
      memIntegration_.stop();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif
   }

   synchronize();

   pe_PROFILING_SECTION {
#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeSimulationStep_.end();
      memSimulationStep_.stop();
   }
}
//*************************************************************************************************




//=================================================================================================
//
//  COMMUNICATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Synchronization of all forces and torques among the connected remote MPI processes.
 *
 * \return void
 *
 * This function synchronizes all applied forces and torques of rigid bodies on the local process
 * with all connected MPI processes. It must be called before performing the time integration.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::synchronizeForces()
{
#if HAVE_MPI
   // Skip synchronization if we compute on a single process
   if( MPISettings::size() <= 1 )
      return;

   pe_PROFILING_SECTION {
      timeForceSync_.start();
      memForceSync_.start();
      timeForceSyncAssembling_.start();
      memForceSyncAssembling_.start();
   }

   // Sending local force contributions of shadow copies to owner.
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Assembling of force synchronization message starts...\n";
   }

   for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "__Processing shadow copy of body " << body->getSystemID() << ".\n";
      }

      if( !body->hasForce() ) {
         // If we did not apply any forces do not send anything.
         continue;
      }

      ProcessID process( body->getOwnerHandle() );
      Process::SendBuff& buffer( process->getSendBuffer() );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "__Sending force contribution " << body->getForce() << ", " << body->getTorque() << " of body " << body->getSystemID() << " to owner process " << process->getRank() << ".\n";
      }
      marshal( buffer, notificationType<RigidBodyForceNotification>() );
      marshal( buffer, RigidBodyForceNotification( *(*body) ) );
   }

   pe_PROFILING_SECTION {
      timeForceSyncAssembling_.end();
      memForceSyncAssembling_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         sentForceSync_.transfered( process->getSendBuffer().size() );

      MPI_Barrier( MPI_COMM_WORLD );

      timeForceSyncCommunicate_.start();
      memForceSyncCommunicate_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Communication of force synchronization messages starts...\n";
   }

   communicate( mpitagDEMSynchronizeForces );

   pe_PROFILING_SECTION {
      timeForceSyncCommunicate_.end();
      memForceSyncCommunicate_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedForceSync_.transfered( process->getRecvBuffer().size() );

      MPI_Barrier( MPI_COMM_WORLD );

      timeForceSyncParsing_.start();
      memForceSyncParsing_.start();
   }

   // Receiving force and torque contributions
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Parsing of force synchronization response starts...\n";
   }

   for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process ) {
      Process::RecvBuff& buffer( process->getRecvBuffer() );
      NotificationType notificationType;

      // Receiving rigid body force/torque contribution notifications [DN] from neighbors (N) and distant processes (D)
      while( !buffer.isEmpty() ) {
         unmarshal( buffer, notificationType );

         switch( notificationType ) {
            case rigidBodyForceNotification: {
               RigidBodyForceNotification::Parameters objparam;
               unmarshal( buffer, objparam );

               BodyID b( *bodystorage_.find( objparam.sid_ ) );
               pe_INTERNAL_ASSERT( !b->isRemote(), "Update notification must only concern local bodies." );

               b->addForce( objparam.f_ );
               b->addTorque( objparam.tau_ );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body force contribution from neighboring process with rank " << process->getRank() << ":\nf = " << objparam.f_ << "\ntau = " << objparam.tau_ << "\nf_total = " << b->getForce() << "\ntau_total = " << b->getTorque() << "\n";
               }
               break;
            }
            default:
               throw std::runtime_error( "Received invalid notification type." );
         }
      }
   }

   pe_PROFILING_SECTION {
      timeForceSyncParsing_.end();
      memForceSyncParsing_.stop();

      MPI_Barrier( MPI_COMM_WORLD );

      timeForceSyncGlobals_.start();
      memForceSyncGlobals_.start();
   }

   if( globalNonfixedBodies_.size() > 0 ) {
      size_t i;
      reductionBuffer_.resize( globalNonfixedBodies_.size() * 6 );

      i = 0;
      for( typename std::set<BodyID,LessSystemID>::iterator it = globalNonfixedBodies_.begin(); it != globalNonfixedBodies_.end(); ++it ) {
         const Vec3 f( (*it)->getForce() ), tau( (*it)->getTorque() );

         reductionBuffer_[i++] = f[0];
         reductionBuffer_[i++] = f[1];
         reductionBuffer_[i++] = f[2];
         reductionBuffer_[i++] = tau[0];
         reductionBuffer_[i++] = tau[1];
         reductionBuffer_[i++] = tau[2];
      }

      MPI_Allreduce( MPI_IN_PLACE, &reductionBuffer_[0], reductionBuffer_.size(), MPITrait<real>::getType(), MPI_SUM, MPISettings::comm() );

      i = 0;
      for( typename std::set<BodyID,LessSystemID>::iterator it = globalNonfixedBodies_.begin(); it != globalNonfixedBodies_.end(); ++it, i += 6 ) {
         (*it)->setForce ( Vec3( reductionBuffer_[i], reductionBuffer_[i + 1], reductionBuffer_[i + 2] ) );
         (*it)->setTorque( Vec3( reductionBuffer_[i + 3], reductionBuffer_[i + 4], reductionBuffer_[i + 5] ) );
      }
   }

   pe_PROFILING_SECTION {
      timeForceSyncGlobals_.end();
      memForceSyncGlobals_.stop();

      MPI_Barrier( MPI_COMM_WORLD );

      timeForceSync_.end();
      memForceSync_.stop();
   }
#endif
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Synchronization of all rigid bodies among the connected remote MPI processes.
 *
 * \return void
 *
 * This function synchronizes all rigid bodies on the local process with all connected MPI
 * processes. It must be called by all processes if body properties (such as positions and
 * velocities) were changed, bodies were added to or removed from the simulation.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::synchronize()
{
#if HAVE_MPI
   // Note: MPI buffers can already be partly filled by removal notifications!

   // Skip synchronization if we compute on a single process
   if( MPISettings::size() <= 1 )
      return;

   pe_PROFILING_SECTION {
      timeBodySync_.start();
      memBodySync_.start();

      timeBodySyncAssembling_.start();
      memBodySyncAssembling_.start();
   }

#ifndef NDEBUG
   // Reset debug flags of all remote bodies.
   for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body ) {
      pe_INTERNAL_ASSERT( body->isRemote(), "Body storage for shadow copies contains local bodies." );
      body->debugFlag_ = false;
   }
#endif

   const int myRank( MPISettings::rank() );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Assembling of body synchronization message starts...\n";
   }

   // position update
   for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ) {
      pe_INTERNAL_ASSERT( !body->isRemote(), "Central body storage contains remote bodies." );

      if( body->isGlobal() ) {
         ++body;
         continue;
      }

      const Vec3 gpos( body->getPosition() );
      BodyID     b   ( *body );

      // Note: At this point we know that the body was locally owned before the position update.

      if( body->getOwnerRank() != myRank ) {
         pe_LOG_ERROR_SECTION( log ) {
            log << "Found remote body with sid " << body->getSystemID() << " in central storage:\n" << *body << "\n";
         }
      }

      pe_INTERNAL_ASSERT( body->getOwnerRank() == myRank, "Owner field in local body storage does not contain the own process rank." );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "Processing local body " << b->getSystemID() << ".\n";
      }

      if( domain_.ownsPoint( gpos ) ) {
         // Body still is locally owned after position update.
         pe_LOG_DEBUG_SECTION( log ) {
            log << "Owner of body " << b->getSystemID() << " is still process " << body->getOwnerRank() << ".\n";
         }

         std::vector<ProcessID> updateNotifications, copyNotifications;

         // Update (nearest) neighbor processes.
         for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process ) {
            if( process->intersectsWith( b ) ) {
               // The body is needed by the process.

               if( body->isRegistered( *process ) ) {
                  // Postpone update notification
                  updateNotifications.push_back( *process );
               }
               else {
                  // Postpone copy notification
                  copyNotifications.push_back( *process );
                  body->registerProcess( *process );
               }
            }
            else if( body->isRegistered( *process ) ) {
               // In case the rigid body no longer intersects the remote process nor interacts with it but is registered,
               // send removal notification.
               Process::SendBuff& buffer( process->getSendBuffer() );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Sending removal notification for body " << b->getSystemID() << " to process " << process->getRank() << ".\n";
               }

               marshal( buffer, notificationType<RigidBodyRemovalNotification>() );
               marshal( buffer, RigidBodyRemovalNotification( *b ) );

               body->deregisterProcess( *process );
            }
         }

         for( typename std::vector<ProcessID>::iterator it = updateNotifications.begin(); it != updateNotifications.end(); ++it ) {
            Process::SendBuff& buffer( (*it)->getSendBuffer() );

            pe_LOG_DEBUG_SECTION( log ) {
               log << "Sending update notification for body " << b->getSystemID() << " to process " << (*it)->getRank() << ".\n";
            }

            marshal( buffer, notificationType< RigidBodyUpdateNotification<Config> >() );
            marshal( buffer, RigidBodyUpdateNotification<Config>( *b ) );
         }

         for( typename std::vector<ProcessID>::iterator it = copyNotifications.begin(); it != copyNotifications.end(); ++it ) {
            Process::SendBuff& buffer( (*it)->getSendBuffer() );

            pe_LOG_DEBUG_SECTION( log ) {
               log << "Sending shadow copy notification for body " << b->getSystemID() << " to process " << (*it)->getRank() << ".\n";
            }

            marshal( buffer, notificationType< RigidBodyCopyNotification<Config> >() );
            marshal( buffer, RigidBodyCopyNotification<Config>( *b ) );
         }

         // Update remote processes (no intersections possible; (long-range) interactions only).
         // TODO iterate over all processes attached bodies are owned by (skipping nearest neighbors)
         // depending on registration send update or copy
      }
      else {
         // Body is no longer locally owned (body is about to be migrated).
         std::pair<int, ProcessID> owner( findOwner( gpos ) );
         pe_INTERNAL_ASSERT( owner.first != myRank, "Internal owner logic is broken." );

         pe_LOG_DEBUG_SECTION( log ) {
            log << "Local body " << b->getSystemID() << " is no longer on process " << body->getOwnerRank() << " but on process " << owner.first << ".\n";
         }

         if( owner.first < 0 ) {
            // No owner found: Outflow condition.
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Sending deletion notifications for body " << body->getSystemID() << " due to outflow.\n";
            }

            // Registered processes receive removal notification in the remove() routine.
            //todelete.push_back( *body );
            body = remove( body );
            delete b;

            // Note: Attached shadow copies are not deleted here. Instead we rely on the deferred deletion since we no
            // longer need the shadow copy: The owner of an attached shadow copy will receive a deletion notification, detach
            // the attachable, delete the shadow copy of the deleted body and send us a removal notification of the body
            // of which we own a shadow copy in the next position update since (probably) we no longer require the body but
            // are still part of its registration list.
            continue;
         }
         else {
            // New owner found among neighbors.
            pe_INTERNAL_ASSERT( owner.second != NULL, "Migration is restricted to neighboring processes." );

            // --->8--- compare snippet above!
            std::vector<ProcessID> updateNotifications, copyNotifications;

            // Update (nearest) neighbor processes.
            for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process ) {
               if( process->intersectsWith( *body ) || owner.second == *process ) { // Make sure that new owner requires the body!
                  // The body is needed by the process.

                  if( body->isRegistered( *process ) ) {
                     // Postpone update notifications
                     updateNotifications.push_back( *process );
                  }
                  else {
                     // Postpone copy notifications
                     copyNotifications.push_back( *process );
                     body->registerProcess( *process );
                  }
               }
               else if( body->isRegistered( *process ) ) {
                  // In case the rigid body no longer intersects the remote process nor interacts with it but is registered,
                  // send removal notification.
                  Process::SendBuff& buffer( process->getSendBuffer() );

                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Sending removal notification for body " << b->getSystemID() << " to process " << process->getRank() << ".\n";
                  }

                  marshal( buffer, notificationType<RigidBodyRemovalNotification>() );
                  marshal( buffer, RigidBodyRemovalNotification( *b ) );

                  body->deregisterProcess( *process );
               }
            }

            for( typename std::vector<ProcessID>::iterator it = updateNotifications.begin(); it != updateNotifications.end(); ++it ) {
               Process::SendBuff& buffer( (*it)->getSendBuffer() );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Sending update notification for body " << b->getSystemID() << " to process " << (*it)->getRank() << ".\n";
               }

               marshal( buffer, notificationType< RigidBodyUpdateNotification<Config> >() );
               marshal( buffer, RigidBodyUpdateNotification<Config>( *b ) );
            }

            for( typename std::vector<ProcessID>::iterator it = copyNotifications.begin(); it != copyNotifications.end(); ++it ) {
               Process::SendBuff& buffer( (*it)->getSendBuffer() );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Sending shadow copy notification for body " << b->getSystemID() << " to process " << (*it)->getRank() << ".\n";
               }

               marshal( buffer, notificationType< RigidBodyCopyNotification<Config> >() );
               marshal( buffer, RigidBodyCopyNotification<Config>( *b ) );
            }
            // --->8--- compare snippet above!

            ProcessID p( owner.second );

            pe_LOG_DEBUG_SECTION( log ) {
               log << "Sending migration notification for body " << b->getSystemID() << " to process " << p->getRank() << ".\n";
               log << "Process registration list before migration: [";
               for( ProcessIterator it = b->beginProcesses(); it != b->endProcesses(); ++it ) {
                  if( it != b->beginProcesses() )
                     log << ", ";
                  log << it->getRank();
               }
               log << "]\n";
            }

            // Set new owner and transform to shadow copy
            b->setOwner( p->getRank(), p );
            b->setRemote( true );
#ifndef NDEBUG
            b->debugFlag_ = true;
#endif

            // Move body to shadow copy storage.
            body = bodystorage_.remove( body );
            bodystorageShadowCopies_.add( b );
            b->manager_ = 0;

            // Note: We cannot determine here whether we require the body since we do not have up to date shadow copies.
            // However, we will most likely have to keep the body since it typically still intersects the process subdomain.

            // Correct registration list (exclude new owner and us - the old owner) and
            // notify registered processes (except for new owner) of (remote) migration since they possess a shadow copy.
            b->deregisterProcess(  p );

            for( ProcessIterator it = b->beginProcesses(); it != b->endProcesses(); ++it ) {
               Process::SendBuff& buffer( it->getSendBuffer() );

               marshal( buffer, notificationType<RigidBodyRemoteMigrationNotification>() );
               marshal( buffer, RigidBodyRemoteMigrationNotification( *b, p->getRank() ) );
            }

            // Send migration notification to new owner
            {
               Process::SendBuff& buffer( p->getSendBuffer() );

               marshal( buffer, notificationType<RigidBodyMigrationNotification>() );
               marshal( buffer, RigidBodyMigrationNotification( *b ) );

               // Note: The new owner misses shadow copies of all attached bodies. Since we do not have an up to date view
               // we need to relay them later.
            }

            continue;
         }
      }

      ++body;
   }

   pe_PROFILING_SECTION {
      timeBodySyncAssembling_.end();
      memBodySyncAssembling_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         sentBodySync_.transfered( process->getSendBuffer().size() );

      MPI_Barrier( MPI_COMM_WORLD );

      timeBodySyncCommunicate_.start();
      memBodySyncCommunicate_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Communication of body synchronization messages starts...\n";
   }

   // TODO send message as soon as it is ready, receive non-blocking and wait here for reception to complete
   // FIXME: If synchronize is called twice that is once by the user after a timestep for example this can break things due to a race condition (see bug #5); using different tags as a workaround
   if( lastSyncTag_ == mpitagDEMSynchronizePositionsAndVelocities2 ) {
      communicate( mpitagDEMSynchronizePositionsAndVelocities );
      lastSyncTag_ = mpitagDEMSynchronizePositionsAndVelocities;
   }
   else {
      communicate( mpitagDEMSynchronizePositionsAndVelocities2 );
      lastSyncTag_ = mpitagDEMSynchronizePositionsAndVelocities2;
   }

   pe_PROFILING_SECTION {
      timeBodySyncCommunicate_.end();
      memBodySyncCommunicate_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedBodySync_.transfered( process->getRecvBuffer().size() );

      MPI_Barrier( MPI_COMM_WORLD );

      timeBodySyncParsing_.start();
      memBodySyncParsing_.start();
   }

   // Receiving the updates for the remote rigid bodies from the connected processes
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Parsing of body synchronization response starts...\n";
   }

   for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process ) {
      Process::RecvBuff& buffer( process->getRecvBuffer() );
      NotificationType notificationType;

      // receiving shadow copies [N], shadow copy updates [DN], (local) migrations [N], (remote) migrations [D], deletions [DN] and removal notifications [DN] from neighbors (N) and distant processes (D)
      while( !buffer.isEmpty() ) {
         unmarshal( buffer, notificationType );

         switch( notificationType ) {
            case rigidBodyCopyNotification: {
               BodyID obj;
               typename RigidBodyCopyNotification<Config>::Parameters objparam;
               unmarshal( buffer, objparam );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received " << objparam.geomType_ << " copy notification from neighboring process with rank " << process->getRank() << ".\n";
               }

               switch( objparam.geomType_ ) {
                  case sphereType: {
                     Sphere::Parameters subobjparam;
                     unmarshal( buffer, subobjparam, false );
                     obj = instantiateSphere( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - process->getOffset(), subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_, false );
                     obj->setLinearVel( subobjparam.v_ );
                     obj->setAngularVel( subobjparam.w_ );
                     obj->setOwner( process->getRank(), *process );
                     break;
                  }
                  case boxType: {
                     Box::Parameters subobjparam;
                     unmarshal( buffer, subobjparam, false );
                     obj = instantiateBox( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - process->getOffset(), subobjparam.rpos_, subobjparam.q_, subobjparam.lengths_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_, false );
                     obj->setLinearVel( subobjparam.v_ );
                     obj->setAngularVel( subobjparam.w_ );
                     obj->setOwner( process->getRank(), *process );
                     break;
                  }
                  case capsuleType: {
                     Capsule::Parameters subobjparam;
                     unmarshal( buffer, subobjparam, false );
                     obj = instantiateCapsule( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - process->getOffset(), subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.length_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_, false );
                     obj->setLinearVel( subobjparam.v_ );
                     obj->setAngularVel( subobjparam.w_ );
                     obj->setOwner( process->getRank(), *process );
                     break;
                  }
                  case cylinderType: {
                     Cylinder::Parameters subobjparam;
                     unmarshal( buffer, subobjparam, false );
                     obj = instantiateCylinder( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - process->getOffset(), subobjparam.rpos_, subobjparam.q_, subobjparam.radius_, subobjparam.length_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_, false );
                     obj->setLinearVel( subobjparam.v_ );
                     obj->setAngularVel( subobjparam.w_ );
                     obj->setOwner( process->getRank(), *process );
                     break;
                  }
                  case planeType: {
                     Plane::Parameters subobjparam;
                     unmarshal( buffer, subobjparam, false );

                     pe_INTERNAL_ASSERT( subobjparam.fixed_ == true, "Cannot instantiate unfixed planes." );

                     obj = instantiatePlane( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - process->getOffset(), subobjparam.rpos_, subobjparam.q_, subobjparam.material_, subobjparam.visible_, false );
                     obj->setLinearVel( subobjparam.v_ );
                     obj->setAngularVel( subobjparam.w_ );
                     obj->setOwner( process->getRank(), *process );
                     break;
                  }
                  case triangleMeshType: {
                     TriangleMesh::Parameters subobjparam;
                     unmarshal( buffer, subobjparam, false );
                     obj = instantiateTriangleMesh( subobjparam.sid_, subobjparam.uid_, subobjparam.gpos_ - process->getOffset(), subobjparam.rpos_, subobjparam.q_, subobjparam.vertices_, subobjparam.faceIndices_, subobjparam.material_, subobjparam.visible_, subobjparam.fixed_, false );
                     obj->setLinearVel( subobjparam.v_ );
                     obj->setAngularVel( subobjparam.w_ );
                     obj->setOwner( process->getRank(), *process );
                     break;
                  }
                  case unionType: {
                     Union::Parameters subobjparam;
                     unmarshal( buffer, subobjparam, false );

                     obj = instantiateUnion( subobjparam, process->getOffset(), true, false );
                     obj->setLinearVel( subobjparam.v_ );
                     obj->setAngularVel( subobjparam.w_ );
                     obj->setOwner( process->getRank(), *process );
                     break;
                  }
                  default: {
                     pe_LOG_DEBUG_SECTION( log ) {
                        log << "Encountered invalid geometry type " << (int)objparam.geomType_ << ".\n";
                     }
                     throw std::runtime_error( "Unknown geometry type" );
                  }
               }

               bodystorageShadowCopies_.add( obj );
               detector_.add( obj );

               // reconstruct list of shadow copy holders
               for( std::size_t i = 0; i < objparam.reglist_.size(); ++i ) {
                  if( objparam.reglist_[i] == myRank )
                     continue;

                  ProcessIterator it( processstorage_.find( objparam.reglist_[i] ) );

                  if( it != processstorage_.end() )
                     obj->registerProcess( *it );
                  else
                     throw std::runtime_error( "Registering distant processes is not yet implemented." );
               }

#ifndef NDEBUG
               obj->debugFlag_ = true;
#endif

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Processed " << objparam.geomType_ << " copy notification:\n" << obj << "\n";
               }

               break;
            }
            case rigidBodyUpdateNotification: {
               typename RigidBodyUpdateNotification<Config>::Parameters objparam;
               unmarshal( buffer, objparam );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body update notification for body " << objparam.sid_ << " from neighboring process with rank " << process->getRank() << ":\nv = " << objparam.v_ << "\nw = " << objparam.w_ << "\nposition = " << objparam.gpos_ << "\nquaternion = " << objparam.q_ << "\n";
               }

               BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );

               pe_INTERNAL_ASSERT( process->getRank() == b->getOwnerRank(), "Update notifications must be sent by owner." );
               pe_INTERNAL_ASSERT( b->isRemote(), "Update notification must only concern shadow copies." );

#ifndef NDEBUG
               b->debugFlag_ = true;
#endif

               b->setPosition   ( objparam.gpos_ - process->getOffset() );
               b->setOrientation( objparam.q_    );
               b->setLinearVel  ( objparam.v_    );
               b->setAngularVel ( objparam.w_    );

               // reconstruct list of shadow copy holders
               b->clearProcesses();
               for( std::size_t i = 0; i < objparam.reglist_.size(); ++i ) {
                  if( objparam.reglist_[i] == myRank )
                     continue;

                  ProcessIterator it( processstorage_.find( objparam.reglist_[i] ) );

                  if( it != processstorage_.end() )
                     b->registerProcess( *it );
                  else
                     throw std::runtime_error( "Registering distant processes is not yet implemented." );
               }

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Processed rigid body update notification:\n" << b << "\n";
               }

               break;
            }
            case rigidBodyMigrationNotification: {
               RigidBodyMigrationNotification::Parameters objparam;
               unmarshal( buffer, objparam );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body migration notification for body " << objparam.sid_ << " from neighboring process with rank " << process->getRank() << ".\n";
               }

               BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );

               bodystorageShadowCopies_.remove( b );
               bodystorage_.add( b );
               b->manager_ = theDefaultManager();

               pe_INTERNAL_ASSERT( process->getRank() == b->getOwnerRank(), "Migration notifications must be sent by previous owner." );
               pe_INTERNAL_ASSERT( b->isRemote(), "Bodies in migration notifications must be available as shadow copies in local process." );
               pe_INTERNAL_ASSERT( domain_.ownsPoint( b->getPosition() ), "Receiving body migration even though we do not own it." );

               b->setOwner( myRank, 0 );
               b->setRemote( false );
               b->registerProcess( *process );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Processed rigid body migration notification:\n" << b << "\n";
               }

               break;
            }
            case rigidBodyRemoteMigrationNotification: {
               RigidBodyRemoteMigrationNotification::Parameters objparam;
               unmarshal( buffer, objparam );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body remote migration notification for body " << objparam.sid_ << " from neighboring process with rank " << process->getRank() << " (previous owner):\nnew owner = " << objparam.to_ << "\n";
               }

               BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );

               pe_INTERNAL_ASSERT( process->getRank() == b->getOwnerRank(), "Remote migration notifications must be sent by previous owner." );
               pe_INTERNAL_ASSERT( b->isRemote(), "Bodies in remote migration notifications must be available as shadow copies in local process." );
               pe_INTERNAL_ASSERT( objparam.to_ != myRank, "Bodies in remote migration notifications may not migrate to local process." );

               ProcessIterator it( processstorage_.find( objparam.to_ ) );
               if( it == processstorage_.end() )
                  // Distant process (TODO)
                  b->setOwner( objparam.to_, 0 );
               else
                  // Neighboring process
                  b->setOwner( objparam.to_, *it );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Processed rigid body remote migration notification:\n" << b << "\n";
               }

               break;
            }
            case rigidBodyRemovalNotification: {
               RigidBodyRemovalNotification::Parameters objparam;
               unmarshal( buffer, objparam );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body removal notification for body " << objparam.sid_ << " from neighboring process with rank " << process->getRank() << " (owner).\n";
               }

               // Remove shadow copy as prompted.
               BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );

               // TODO assert that we indeed do not need the shadow copy anymore
               pe_INTERNAL_ASSERT( b->getOwnerRank() == process->getRank(), "Only owner is allowed to send removal notifications." );

               bodystorageShadowCopies_.remove( b );
               detector_.remove( b );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Processed rigid body removal notification:\n" << b << "\n";
               }

               delete b;

               break;
            }
            case rigidBodyDeletionNotification: {
               RigidBodyDeletionNotification::Parameters objparam;
               unmarshal( buffer, objparam );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body deletion notification for body " << objparam.sid_ << " from neighboring process with rank " << process->getRank() << " (owner).\n";
               }

               // Remove invalid shadow copy.
               BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );

               pe_INTERNAL_ASSERT( b->getOwnerRank() == process->getRank(), "Only owner is allowed to send deletion notifications." );

               bodystorageShadowCopies_.remove( b );
               detector_.remove( b );

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Processed rigid body deletion notification:\n" << b << "\n";
               }

               delete b;

               break;
            }
            default:
               throw std::runtime_error( "Received invalid notification type." );
         }
      }
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Parsing of body synchronization response ended.\n";
   }

#ifndef NDEBUG
   // Assert that all remote bodies are up to date.
   for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body ) {
      pe_INTERNAL_ASSERT( body->isRemote(), "Body storage for shadow copies contains local bodies." );
      if( !body->debugFlag_ ) {
         pe_LOG_ERROR_SECTION( log ) {
            log << "Body with owner " << body->getOwnerRank() << " is not up-to-date:\n" << *body << "\n";
         }
         pe_INTERNAL_ASSERT( false, "Shadow copy is not up-to-date." );
      }
   }
#endif

   requireSync_ = false;

   pe_PROFILING_SECTION {
      timeBodySyncParsing_.end();
      memBodySyncParsing_.stop();

      MPI_Barrier( MPI_COMM_WORLD );

      timeBodySync_.end();
      memBodySync_.stop();
   }

   // HOWTO IDENTIFY THE BODIES TO RELAY?
   // - foreach migration away from us:
   //    - relay the bodies attached to the migrated body (even though we do not know whether the new owner actually needs them)
   //      or relay deletion notifications

   // HOWTO GET RID OF ATTACHED SHADOW COPIES?
   // when receiving a (remote) migration notice, we _add_ the new owner to the registration list since the shadow copy of our body will be relayed for us
   // the next time we will probably send a removal notice if we find that the body is no longer required by the old owner

   // TODO delete all marked deletions (including some of the migrations)
   // deletion involves deletion of no longer needed (attached) shadow copies

#endif
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Clears the collision system.
 *
 * \return void
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::clear()
{
   detector_.clear();

   for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body )
      delete *body;
   bodystorage_.clear();

   for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body )
      delete *body;
   bodystorageShadowCopies_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Clearing the contacts and batches of the simulation world.
 *
 * \return void.
 *
 * This function destroys all currently active contacts of the simulation world and resets the
 * generated batches.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolver> >::clearContacts()
{
   typename Contacts::Iterator       begin( contacts_.begin() );
   typename Contacts::Iterator const end  ( contacts_.end()   );

   for( ; begin!=end; ++begin )
      delete *begin;

   contacts_.clear();
}
//*************************************************************************************************


} // namespace pe

#endif

