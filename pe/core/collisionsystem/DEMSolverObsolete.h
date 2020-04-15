//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/DEMSolverObsolete.h
 *  \brief Specialization of the CollisionSystem class template for the discrete element solver
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

#ifndef _PE_CORE_COLLISIONSYSTEM_DEMSOLVEROBSOLETE_H_
#define _PE_CORE_COLLISIONSYSTEM_DEMSOLVEROBSOLETE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <pe/core/attachable/Attachable.h>
#include <pe/core/attachable/AttachableStorage.h>
#include <pe/core/batches/BatchVector.h>
#include <pe/core/batches/Generators.h>
#include <pe/core/rigidbody/BodyStorage.h>
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
#include <pe/core/joint/JointStorage.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISection.h>
#include <pe/core/MPISettings.h>
#include <pe/core/MPITag.h>
#include <pe/core/MPITrait.h>
#include <pe/core/ProfilingSection.h>
#include <pe/core/response/MPIDecoder.h>
#include <pe/core/response/MPIEncoder.h>
#include <pe/core/response/Solvers.h>
#include <pe/core/rigidbody/MPIRigidBodyTrait.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/RootSection.h>
#include <pe/core/SerialSection.h>
#include <pe/core/Types.h>
#include <pe/util/Assert.h>
#include <pe/util/constraints/BaseOf.h>
#include <pe/util/Logging.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/singleton/Singleton.h>
#include <pe/util/Timing.h>
#include <pe/util/Types.h>
#include <pe/util/Vector.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Specialization of the collision system for the discrete element solver.
 * \ingroup core
 *
 * This specialization of the CollisionSystem class template adapts the collision system of the
 * rigid body simulation world to the requirements of the discrete element solver. In
 * contrast to other collision response algorithms, the discrete element solver doesn't
 * require the setup of contact batches. Like for the FFD solver, the setting for the batch generation process
 * (see pe::pe_BATCH_GENERATOR) is simply ignored.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >
   : public MPICommunication, public Singleton< CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >, logging::Logger >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,BG,response::DEMSolverObsolete>      Config;             //!< Type of the configuration.

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

   typedef MPIEncoder<Config>                   Encoder;            //!< Type of the body encoder.
   typedef MPIDecoder<Config>                   Decoder;            //!< Type of the body decoder.

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
   typedef response::DEMSolverObsolete<Config>  ContactSolver;
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
   inline size_t          getNumContacts()       const;
   inline real            getMaxOverlap()        const;
   inline CoarseDetector& getCoarseDetector();
   inline ContactSolver&  getContactSolver();
   inline const BS&       getBodyStorage()       const;
   inline const PS&       getProcessStorage()    const;
   inline const AS&       getAttachableStorage() const;
   inline const Domain&   getDomain()            const;
   //@}
   //**********************************************************************************************

   //**Query functions*****************************************************************************
   /*!\name Query functions */
   //@{
   inline bool            isSyncRequired()       const;
   inline void            logProfilingSummary()  const;
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
   void synchronize();

   void firstCommunication ();
   void secondCommunication();
   //@}
   //**********************************************************************************************

   //**Send functions****************************************************************************
   /*!\name Send functions */
   //@{
   inline void sendBody    ( ProcessID p, ConstBodyID     b );
   inline void sendSphere  ( ProcessID p, ConstSphereID   s );
   inline void sendBox     ( ProcessID p, ConstBoxID      b );
   inline void sendCapsule ( ProcessID p, ConstCapsuleID  c );
   inline void sendCylinder( ProcessID p, ConstCylinderID c );
   inline void sendUnion   ( ProcessID p, ConstUnionID    u );
   inline void updateBody  ( ProcessID p, ConstBodyID     b );
   inline void sendForce   ( ProcessID p, ConstBodyID     b );
   //@}
   //**********************************************************************************************

   //**Receive functions****************************************************************************
   /*!\name Receive functions */
   //@{
   BodyID recvBody  ( ProcessID p );
   void   recvForces( ProcessID p );
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
   size_t numContacts_;       //!< The number of contacts in the last time step.
   real maxOverlap_;          //!< The maximum overlap in the past time steps.
   Contacts contacts_;        //!< The currently active contacts of the simulation world.
   CoarseDetector detector_;  //!< The active collision detection algorithm.
   ContactSolver solver_;     //!< The active collision response algorithm.
   BS bodystorage_;           //!< The rigid body storage.
   Decoder decoder_;          //!< The body decoder.
   PS processstorage_;        //!< The process storage.
   Domain domain_;            //!< The local process domain.
   AS attachablestorage_;     //!< The attachable storage.
   JS jointstorage_;          //!< The joint storage.

   timing::WcTimer timeCollisionDetection_, timeCollisionResponse_, timeBodySync_, timeBodySyncAssembling_, timeBodySyncCommunicate_, timeBodySyncParsing_, timeResync_, timeForceSync_, timeForceSyncAssembling_, timeForceSyncCommunicate_, timeForceSyncParsing_, timeIntegration_, timeSimulationStep_;
   std::vector<timing::WcTimer*> timers_;

   TransferMeter sentBodySync_, sentForceSync_, sentResync_;
   std::vector<TransferMeter*> sentMeters_;

   TransferMeter receivedBodySync_, receivedForceSync_, receivedResync_;
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
CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::CollisionSystem()
   : MPICommunication ( processstorage_, domain_ )
   , Singleton<CollisionSystem,logging::Logger>()  // Initialization of the Singleton base object
   , numContacts_      (    0 )                    // The number of time steps in the last time step
   , maxOverlap_       (    0 )                    // The maximum overlap in the past time steps
   , contacts_         ( 5000 )                    // The currently active contacts of the simulation world
   , detector_         ( bodystorage_ )            // The active collision detection algorithm
   , solver_           ( domain_ )                 // The active collision response algorithm
   , bodystorage_      ()
   , decoder_          ( bodystorage_, attachablestorage_ )
   , processstorage_   ()
   , domain_           ( processstorage_ )
   , attachablestorage_()
   , jointstorage_     ()
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
   timers_.push_back( &timeIntegration_ );
   timers_.push_back( &timeSimulationStep_ );
   timers_.push_back( &timeResync_ );

   // Registering all sent meters
   sentMeters_.push_back( &sentOverall_ );
   sentMeters_.push_back( &sentBodySync_ );
   sentMeters_.push_back( &sentForceSync_ );
   sentMeters_.push_back( &sentResync_ );

   // Registering all received meters
   receivedMeters_.push_back( &receivedOverall_ );
   receivedMeters_.push_back( &receivedBodySync_ );
   receivedMeters_.push_back( &receivedForceSync_ );
   receivedMeters_.push_back( &receivedResync_ );

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
CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::~CollisionSystem()
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
/*!\brief Returns the number of contacts used during the last time step.
 *
 * \return The number of contacts of the last time step.
 *
 * This function returns the total number of contacts that were used during the collision
 * resolution process of the last time step.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getNumContacts() const
{
   return numContacts_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Get the maximum detected overlap.
 *
 * \return The maximum overlap between two rigid bodies in the last time step.
 *
 * This function returns the maximum overlap between two rigid bodies in the last time step.
 * If the overlap between two bodies gets too large, this could affect the numerical stability
 * of the simulation. Therefore the time step size should be reduced in case the maximum overlap
 * appears to be too large. In many cases this leads to smaller overlaps.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
real CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getMaxOverlap() const
{
   return maxOverlap_;
}
//*************************************************************************************************


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
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::CoarseDetector&
   CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getCoarseDetector()
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
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::ContactSolver&
   CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getContactSolver()
{
   return solver_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a constant reference to the body storage.
 *
 * \return Constant reference to the body storage.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const typename CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::BS&
   CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getBodyStorage() const
{
   return bodystorage_;
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
inline const typename CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::PS&
   CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getProcessStorage() const
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
inline const typename CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::AS&
   CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getAttachableStorage() const
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
inline const Domain& CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::getDomain() const
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
 * The DEM solver currently does not enforce synchronization and thus always returns false.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline bool CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::isSyncRequired() const
{
   return false;
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
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::logProfilingSummary() const
{
   // Logging the profiling results
   pe_PROFILING_SECTION {
      if( logging::loglevel >= logging::info ) {
         // Store the minimum/maximum/total time measurement and number of measurements of each timer over all time steps
         std::vector<double> minValues( timers_.size() ), maxValues( timers_.size() ), totalValues( timers_.size() );
         std::vector<size_t> numValues( timers_.size() );
         for( std::size_t i = 0; i < timers_.size(); ++i ) {
            minValues[i]   = timers_[i]->min();
            maxValues[i]   = timers_[i]->max();
            totalValues[i] = timers_[i]->total();
            numValues[i]   = timers_[i]->getCounter();
         }

         pe_LOG_INFO_SECTION( log ) {
            log << "Timing results reduced over all time steps on current process:\n" << std::fixed << std::setprecision(4)
                     << "code part              min time     max time     avg time     total time   executions\n"
                     << "--------------------   ----------   ----------   ----------   ----------   ----------\n"
                     << "simulation step        "   << std::setw(10) << minValues[11] << "   " << std::setw(10) << maxValues[11] << "   " << std::setw(10) << totalValues[11] / numValues[11] << " = " << std::setw(10) << totalValues[11] << " / " << std::setw(10) << numValues[11] << "\n"
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
                     << "  time integration     "   << std::setw(10) << minValues[10] << "   " << std::setw(10) << maxValues[10] << "   " << std::setw(10) << totalValues[10] / numValues[10] << " = " << std::setw(10) << totalValues[10] << " / " << std::setw(10) << numValues[10] << "\n"
                     << "resync                 "   << std::setw(10) << minValues[12] << "   " << std::setw(10) << maxValues[12] << "   " << std::setw(10) << totalValues[12] / numValues[12] << " = " << std::setw(10) << totalValues[12] << " / " << std::setw(10) << numValues[12] << "\n"
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
                      << "simulation step        "   << std::setw(10) << minValues[11] << "   " << std::setw(10) << maxValues[11] << "   " << std::setw(10) << totalValues[11] / numValues[11] << " = " << std::setw(10) << totalValues[11] << " / " << std::setw(10) << numValues[11] << "\n"
                      << "  col. detection       "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "\n"
                      << "  col. response        "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "\n"
                      << "  body sync            "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "\n"
                      << "     assembling        "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "\n"
                      << "     communicate       "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "\n"
                      << "     parsing           "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "\n"
                      << "  force sync           "   << std::setw(10) << minValues[ 6] << "   " << std::setw(10) << maxValues[ 6] << "   " << std::setw(10) << totalValues[ 6] / numValues[ 6] << " = " << std::setw(10) << totalValues[ 6] << " / " << std::setw(10) << numValues[ 6] << "\n"
                      << "     assembling        "   << std::setw(10) << minValues[ 7] << "   " << std::setw(10) << maxValues[ 7] << "   " << std::setw(10) << totalValues[ 7] / numValues[ 7] << " = " << std::setw(10) << totalValues[ 7] << " / " << std::setw(10) << numValues[ 7] << "\n"
                      << "     communicate       "   << std::setw(10) << minValues[ 8] << "   " << std::setw(10) << maxValues[ 8] << "   " << std::setw(10) << totalValues[ 8] / numValues[ 8] << " = " << std::setw(10) << totalValues[ 8] << " / " << std::setw(10) << numValues[ 8] << "\n"
                      << "     parsing           "   << std::setw(10) << minValues[ 9] << "   " << std::setw(10) << maxValues[ 9] << "   " << std::setw(10) << totalValues[ 9] / numValues[ 9] << " = " << std::setw(10) << totalValues[ 9] << " / " << std::setw(10) << numValues[ 9] << "\n"
                      << "  time integration     "   << std::setw(10) << minValues[10] << "   " << std::setw(10) << maxValues[10] << "   " << std::setw(10) << totalValues[10] / numValues[10] << " = " << std::setw(10) << totalValues[10] << " / " << std::setw(10) << numValues[10] << "\n"
                      << "resync                 "   << std::setw(10) << minValues[12] << "   " << std::setw(10) << maxValues[12] << "   " << std::setw(10) << totalValues[12] / numValues[12] << " = " << std::setw(10) << totalValues[12] << " / " << std::setw(10) << numValues[12] << "\n"
                      << "--------------------   ----------   ----------   ----------   ----------   ----------\n";
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
                << "  resync               "   << std::setw(10) << sentMinBytes[3] << "   " << std::setw(10) << sentMaxBytes[3] << "   " << std::setw(10) << (sentNumTransfers[3] == 0 ? 0 : sentTotalBytes[3] / sentNumTransfers[3]) << " = " << std::setw(10) << sentTotalBytes[3] << " / " << std::setw(10) << sentNumTransfers[3] << "\n"
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
                      << "  resync               "   << std::setw(10) << sentMinBytes[3] << "   " << std::setw(10) << sentMaxBytes[3] << "   " << std::setw(10) << (sentNumTransfers[3] == 0 ? 0 : sentTotalBytes[3] / sentNumTransfers[3]) << " = " << std::setw(10) << sentTotalBytes[3] << " / " << std::setw(10) << sentNumTransfers[3] << "\n"
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
                << "  resync               "   << std::setw(10) << receivedMinBytes[3] << "   " << std::setw(10) << receivedMaxBytes[3] << "   " << std::setw(10) << (receivedNumTransfers[3] == 0 ? 0 : receivedTotalBytes[3] / receivedNumTransfers[3]) << " = " << std::setw(10) << receivedTotalBytes[3] << " / " << std::setw(10) << receivedNumTransfers[3] << "\n"
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
                      << "  resync               "   << std::setw(10) << receivedMinBytes[3] << "   " << std::setw(10) << receivedMaxBytes[3] << "   " << std::setw(10) << (receivedNumTransfers[3] == 0 ? 0 : receivedTotalBytes[3] / receivedNumTransfers[3]) << " = " << std::setw(10) << receivedTotalBytes[3] << " / " << std::setw(10) << receivedNumTransfers[3] << "\n"
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
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::add( BodyID body )
{
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
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::remove( BodyID body )
{
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
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::BodyIterator CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::remove( BodyIterator body )
{
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
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::removeFromCollisionDetector( BodyID body )
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
void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::simulationStep( real timestep )
{
   // Early exit if no rigid bodies are specified
   pe_SERIAL_SECTION {
      if( getBodyStorage().isEmpty() ) return;
   }

   pe_PROFILING_SECTION {
      timeSimulationStep_.start();
      timeCollisionDetection_.start();
   }

   // Finding all contacts
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Detecting contacts...\n";
   }

   detector_.findContacts( contacts_ );
   numContacts_ = contacts_.size();

   pe_PROFILING_SECTION {
      timeCollisionDetection_.end();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeCollisionResponse_.start();
   }

   // Resolving all contacts
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Resolving contacts...\n";
   }

   for( typename Contacts::Iterator c=contacts_.begin(); c!=contacts_.end(); ++c )
   {
      const real overlap( solver_.resolveContact( *c ) );
      if( overlap > maxOverlap_ )
         maxOverlap_ = overlap;
   }

   // Clearing the contacts
   clearContacts();

   pe_PROFILING_SECTION {
      timeCollisionResponse_.end();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif
   }

   // Synchronizing the forces acting on the rigid bodies
   pe_MPI_SECTION {
      firstCommunication();
   }

   pe_PROFILING_SECTION {
      timeIntegration_.start();
   }

   // Updating the positions and velocities of all local rigid bodies.
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Time integration starts...\n";
   }

   for( BodyIterator body=bodystorage_.begin(); body!=bodystorage_.end(); ++body ) {
      body->move( timestep );
   }

   pe_PROFILING_SECTION {
      timeIntegration_.end();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif
   }

   pe_MPI_SECTION
   {
      // Updating all rigid bodies that cross a process boundary
      secondCommunication();

      // Updating the remote flag and destroying non-local rigid bodies
      for( BodyIterator body=bodystorage_.begin(); body!=bodystorage_.end(); )
      {
         // Checking that the forces on remote bodies have been reset
         pe_INTERNAL_ASSERT( !body->isRemote() || !body->hasForce(), "Invalid force on remote body detected" );

         // Ignoring global rigid bodies
         if( body->isGlobal() ) {
            ++body;
            continue;
         }

         // Setting the remote flag of the rigid body
         body->setRemote( !domain_.ownsPoint( body->getPosition() ) );

         // Removing the rigid body in case it is no longer contained in the local domain
         // In case the forces acting on the body are non-zero, the rigid body has been completely
         // moved from this process to the neighboring process in a single time step. In this case
         // the deletion of the body is postponed to the next time step to guarantee a force
         // synchronization.
         if( body->isRemote() && !body->hasForce() && !domain_.intersectsWith( *body ) ) {
            BodyID bodyid( *body );
            body = remove( body );
            delete bodyid;              // Destroying the rigid body
         }
         else ++body;
      }
   }

   pe_PROFILING_SECTION {
      timeSimulationStep_.end();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif
   }

#if 0
   // Logging the profiling results for this time step
   pe_PROFILING_SECTION {
      if( logging::loglevel >= logging::progress ) {
         pe_MPI_SECTION {
            const int          root( MPISettings::root() );
            const MPI_Comm     comm( MPISettings::comm() );
            const int          rank( MPISettings::rank() );

            // Store the last time measurement of each timer into multiple fields and reduce them with minimum, maximum and sum operations
            std::vector<double> minValues( timers_.size() ), maxValues( timers_.size() ), totalValues( timers_.size() );
            for( std::size_t i = 0; i < minValues.size(); ++i )
               minValues[i] = maxValues[i] = totalValues[i] = timers_[i]->last();

            if( rank == root ) {
               MPI_Reduce( MPI_IN_PLACE, &minValues[0],   minValues.size(),   MPITrait<double>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &maxValues[0],   maxValues.size(),   MPITrait<double>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( MPI_IN_PLACE, &totalValues[0], totalValues.size(), MPITrait<double>::getType(), MPI_SUM, root, comm );
            }
            else {
               MPI_Reduce( &minValues[0],   0, minValues.size(),   MPITrait<double>::getType(), MPI_MIN, root, comm );
               MPI_Reduce( &maxValues[0],   0, maxValues.size(),   MPITrait<double>::getType(), MPI_MAX, root, comm );
               MPI_Reduce( &totalValues[0], 0, totalValues.size(), MPITrait<double>::getType(), MPI_SUM, root, comm );
            }

            pe_ROOT_SECTION {
               pe_LOG_PROGRESS_SECTION( log ) {
                  log << "Timing results for current time step reduced over all ranks:\n"
                           << "code part              min time     max time     total time\n"
                           << "--------------------   ----------   ----------   ----------\n" << std::fixed << std::setprecision(4)
                           << "simulation step        "   << std::setw(10) << minValues[11] << "   " << std::setw(10) << maxValues[11] << "   " << std::setw(10) << totalValues[11] << "\n"
                           << "  col. detection       "   << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] << "\n"
                           << "  col. response        "   << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] << "\n"
                           << "  body sync            "   << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] << "\n"
                           << "    assembling         "   << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] << "\n"
                           << "    communicate        "   << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] << "\n"
                           << "    parsing            "   << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] << "\n"
                           << "  force sync           "   << std::setw(10) << minValues[ 6] << "   " << std::setw(10) << maxValues[ 6] << "   " << std::setw(10) << totalValues[ 6] << "\n"
                           << "    assembling         "   << std::setw(10) << minValues[ 7] << "   " << std::setw(10) << maxValues[ 7] << "   " << std::setw(10) << totalValues[ 7] << "\n"
                           << "    communicate        "   << std::setw(10) << minValues[ 8] << "   " << std::setw(10) << maxValues[ 8] << "   " << std::setw(10) << totalValues[ 8] << "\n"
                           << "    parsing            "   << std::setw(10) << minValues[ 9] << "   " << std::setw(10) << maxValues[ 9] << "   " << std::setw(10) << totalValues[ 9] << "\n"
                           << "  time integration     "   << std::setw(10) << minValues[10] << "   " << std::setw(10) << maxValues[10] << "   " << std::setw(10) << totalValues[10] << "\n"
                           << "resync                 "   << std::setw(10) << minValues[12] << "   " << std::setw(10) << maxValues[12] << "   " << std::setw(10) << totalValues[12] << "\n"
                           << "--------------------   ----------   ----------   ----------\n";
               }
            }
         }
      }
   }
#endif
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
 *
 * This function synchronizes all rigid bodies on the local process with all connected MPI
 * processes.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::synchronize()
{
   pe_PROFILING_SECTION {
      timeResync_.start();
   }

   pe_INTERNAL_ASSERT( !processstorage_.isEmpty(), "Synchronization without connected processes" );

   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   // Synchronizing the rigid bodies contained in the local domain of the simulation world:
   //  1.) Infinite rigid bodies can be ignored.
   //  2.) Remote rigid bodies are destroyed since their position might have changed. In case they
   //      are still contained in this process, they are resent during the MPI communication.
   //  3.) Sending all local rigid bodies to all processes they are (partially) contained in.
   //  4.) Updating the remote flag of the rigid bodies.
   //  5.) Removing all rigid bodies that are no longer contained in the local domain.
   for( BodyIterator body=bodystorage_.begin(); body!=bodystorage_.end(); )
   {
      // Ignoring global rigid bodies
      if( body->isGlobal() ) {
         ++body;
         continue;
      }

      // Destroying the rigid body in case it is not contained in the local domain
      if( body->isRemote() ) {
         BodyID bodyid( *body );
         body = remove( body );
         delete bodyid;              // Destroying the rigid body
         continue;
      }

      // Sending the rigid body to all processes it is (partially) contained in
      for( ProcessIterator process=pbegin; process!=pend; ++process ) {
         if( process->intersectsWith( *body ) ) {
            sendBody( *process, *body );
            body->registerProcess( *process );
         }
         else {
            body->deregisterProcess( *process );
         }
      }

      // Setting the remote flag of the rigid body
      body->setRemote( !domain_.ownsPoint( body->getPosition() ) );

      // Removing the rigid body in case it is no longer contained in the local domain
      if( body->isRemote() && !domain_.intersectsWith( *body ) ) {
         BodyID bodyid( *body );
         body = remove( body );
         delete bodyid;              // Destroying the rigid body
      }
      else ++body;
   }

   pe_PROFILING_SECTION {
      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         sentResync_.transfered( process->getSendBuffer().size() );
   }

   // Performing the MPI communication
   communicate( mpitagDEMObsoleteSynchronizeComplete );

   pe_PROFILING_SECTION {
      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedResync_.transfered( process->getRecvBuffer().size() );
   }

   // Receiving the rigid bodies contained in this process
   for( ProcessIterator process1=pbegin; process1!=pend; ++process1 )
   {
      while( process1->hasData() )
      {
         // Receiving the rigid body
         BodyID body = recvBody( *process1 );

         // Setting the remote flag of the rigid body
         body->setRemote( !domain_.ownsPoint( body->getPosition() ) );

         // Registering the processes the rigid body is (partially) contained in
         for( ProcessIterator process2=pbegin; process2!=pend; ++process2 ) {
            if( process2->intersectsWith( body ) )
               body->registerProcess( *process2 );
         }
      }
   }

   pe_PROFILING_SECTION {
      timeResync_.end();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief First MPI communication during a single time step of the DEM solver.
 *
 * \return void
 *
 * This communication synchronizes the total forces acting on the rigid bodies.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::firstCommunication()
{
   // Sending local force contributions of shadow copies to owner.
   pe_PROFILING_SECTION {
      timeForceSync_.start();
      timeForceSyncAssembling_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Assembling of force synchronization message starts...\n";
   }

   const BodyIterator bbegin( bodystorage_.begin() );
   const BodyIterator bend  ( bodystorage_.end()   );

   for( BodyIterator body=bbegin; body!=bend; ++body )
   {
      // Don't send a fixed rigid body (including infinite rigid bodies)
      if( body->isFixed() ) continue;

      // Don't send the forces of a local rigid body
      if( !body->isRemote() ) {
         body->clearProcesses();
         continue;
      }

      // Checking the finiteness of the rigid body
      pe_INTERNAL_ASSERT( body->isFinite(), "Invalid infinite rigid body detected" );

      // Checking the number of registered processes
      // Every remote rigid body must have at least one registered process (the managing process)
      pe_INTERNAL_ASSERT( body->hasProcesses(), "Invalid number of registered processes" );

      // Sending the rigid body to all registered processes
      const MPIRigidBodyTrait::ProcessIterator pbegin( body->beginProcesses() );
      const MPIRigidBodyTrait::ProcessIterator pend  ( body->endProcesses()   );

      // Send force to the body's origin process (since we don't know the origin we send it to all neighboring processes)
      for( MPIRigidBodyTrait::ProcessIterator process=pbegin; process!=pend; ++process ) {
         sendForce( *process, *body );
      }

      // Deregistering all processes
      body->clearProcesses();
   }

   pe_PROFILING_SECTION {
      timeForceSyncAssembling_.end();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         sentForceSync_.transfered( process->getSendBuffer().size() );

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeForceSyncCommunicate_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Force synchronization starts...\n";
   }

   communicate( mpitagDEMObsoleteSynchronizeForces );

   pe_PROFILING_SECTION {
      timeForceSyncCommunicate_.end();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedForceSync_.transfered( process->getRecvBuffer().size() );

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeForceSyncParsing_.start();
   }

   // Receiving force and torque contributions
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Parsing of force synchronization response starts...\n";
   }

   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   for( ProcessIterator process=pbegin; process!=pend; ++process ) {
      recvForces( *process );
   }

   pe_PROFILING_SECTION {
      timeForceSyncParsing_.end();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeForceSync_.end();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Second MPI communication during a single time step of the DEM solver.
 *
 * \return void
 *
 * This communication updates all remote rigid bodies on all processes and sends all rigid
 * bodies that cross a new process boundary.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::secondCommunication()
{
   pe_PROFILING_SECTION {
      timeBodySync_.start();
      timeBodySyncAssembling_.start();
   }

   const BodyIterator bbegin( bodystorage_.begin() );
   const BodyIterator bend  ( bodystorage_.end()   );

   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   // Updating all remote rigid bodies on the remote processes
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Assembling of body synchronization message starts...\n";
   }

   for( BodyIterator body=bbegin; body!=bend; ++body )
   {
      // Setting the update flag of the rigid body
      body->setUpdated( false );

      // Don't send a fixed or remote rigid body
      if( body->isFixed() || body->isRemote() ) continue;

      for( ProcessIterator process=pbegin; process!=pend; ++process )
      {
         // In case the rigid body is contained in the remote process, it is either updated
         // (in case it is already registered) or completely sent (in case it is not registered)
         if( process->intersectsWith( *body ) )
         {
            if( body->isRegistered( *process ) ) {
               updateBody( *process, *body );
            }
            else {
               sendBody( *process, *body );
               body->registerProcess( *process );
            }
         }

         // In case the rigid body is not contained in the remote process but registered,
         // update it one last time and deregister the remote process
         else if( body->isRegistered( *process ) ) {
            updateBody( *process, *body );
            body->deregisterProcess( *process );
         }
      }
   }

   pe_PROFILING_SECTION {
      timeBodySyncAssembling_.end();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         sentBodySync_.transfered( process->getSendBuffer().size() );

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeBodySyncCommunicate_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Body synchronization starts...\n";
   }

   communicate( mpitagDEMObsoleteSynchronizePositionsAndVelocities );

   pe_PROFILING_SECTION {
      timeBodySyncCommunicate_.end();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedBodySync_.transfered( process->getRecvBuffer().size() );

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeBodySyncParsing_.start();
   }

   // Receiving the updates for the remote rigid bodies from the connected processes
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Parsing of body synchronization response starts...\n";
   }

   for( ProcessIterator process=pbegin; process!=pend; ++process )
   {
      while( process->hasData() )
      {
         // Receiving the rigid body
         BodyID body = recvBody( *process );
         pe_INTERNAL_ASSERT( body->isRemote() , "Invalid local rigid body detected" );
         pe_INTERNAL_ASSERT( body->isUpdated(), "Invalid update flag detected"      );

         // Deregistering all processes
         body->clearProcesses();

         // Registering all connected processes the body is contained in
         for( ProcessIterator p=pbegin; p!=pend; ++p ) {
            if( p->intersectsWith( body ) )
               body->registerProcess( *p );
         }
      }
   }

   // Checking the update flags of the remote rigid bodies
   pe_INTERNAL_ASSERT( checkUpdateFlags(), "Invalid update flags detected" );

   pe_PROFILING_SECTION {
      timeBodySyncParsing_.end();

#if HAVE_MPI
      MPI_Barrier( MPI_COMM_WORLD );
#endif

      timeBodySync_.end();
   }
}
//*************************************************************************************************






//=================================================================================================
//
//  SEND FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Encoding a rigid body for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
 * \param b The rigid body to be send to the remote MPI process.
 * \return void
 * \exception std::runtime_error Unknown geometry type.
 *
 * This function encodes the given rigid body for the send operation to the remote MPI
 * process. Note that it is invalid to send an infinite, fixed or remote rigid body!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::sendBody( ProcessID p, ConstBodyID b )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending " << b->getType() << " " << b->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << b->getPosition() << "\n"
          << "         Linear velocity  = " << b->getLinearVel() << "\n"
          << "         Angular velocity = " << b->getAngularVel();
   }

   // Encoding the rigid body
   Encoder::encodeBody( p->getSendBuffer(), b, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a sphere primitive for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
 * \param s The sphere to be send to the remote MPI process.
 * \return void
 *
 * This function encodes the given sphere primitive for the send operation to the remote MPI
 * process. Note that it is invalid to send an infinite, fixed or remote sphere primitive!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::sendSphere( ProcessID p, ConstSphereID s )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending sphere " << s->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << s->getPosition() << "\n"
          << "         Linear velocity  = " << s->getLinearVel() << "\n"
          << "         Angular velocity = " << s->getAngularVel();
   }

   // Encoding the sphere primitive
   Encoder::encodeSphere( p->getSendBuffer(), s, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a box primitive for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
 * \param b The box to be send to the remote MPI process.
 * \return void
 *
 * This function encodes the given box primitive for the send operation to the remote MPI
 * process. Note that it is invalid to send an infinite, fixed or remote box primitive!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::sendBox( ProcessID p, ConstBoxID b )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending box " << b->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << b->getPosition() << "\n"
          << "         Linear velocity  = " << b->getLinearVel() << "\n"
          << "         Angular velocity = " << b->getAngularVel();
   }

   // Encoding the box primitive
   Encoder::encodeBox( p->getSendBuffer(), b, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a capsule primitive for the send operation to the remote MPI process.
 *
 * \param c The capsule to be send to the remote MPI process.
 * \return void
 *
 * This function encodes the given capsule primitive for the send operation to the remote MPI
 * process. Note that it is invalid to send an infinite, fixed or remote capsule primitive!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::sendCapsule( ProcessID p, ConstCapsuleID c )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending capsule " << c->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << c->getPosition() << "\n"
          << "         Linear velocity  = " << c->getLinearVel() << "\n"
          << "         Angular velocity = " << c->getAngularVel();
   }

   // Encoding the capsule primitive
   Encoder::encodeCapsule( p->getSendBuffer(), c, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a cylinder primitive for the send operation to the remote MPI process.
 *
 * \param c The cylinder to be send to the remote MPI process.
 * \return void
 *
 * This function encodes the given cylinder primitive for the send operation to the remote MPI
 * process. Note that it is invalid to send an infinite, fixed or remote cylinder primitive!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::sendCylinder( ProcessID p, ConstCylinderID c )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending cylinder " << c->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << c->getPosition() << "\n"
          << "         Linear velocity  = " << c->getLinearVel() << "\n"
          << "         Angular velocity = " << c->getAngularVel();
   }

   // Encoding the cylinder primitive
   Encoder::encodeCylinder( p->getSendBuffer(), c, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a union compound geometry for the send operation to the remote MPI process.
 *
 * \param u The union to be send to the remote MPI process.
 * \return void
 *
 * This function encodes the given union for the send operation to the remote MPI process.
 * Note that it is invalid to send an infinite, fixed or remote union!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::sendUnion( ProcessID p, ConstUnionID u )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending union " << u->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << u->getPosition() << "\n"
          << "         Linear velocity  = " << u->getLinearVel() << "\n"
          << "         Angular velocity = " << u->getAngularVel();
   }

   // Encoding the union compound geometry
   Encoder::encodeUnion( p->getSendBuffer(), u, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Updating a local rigid body on the remote MPI process.
 *
 * \param b The rigid body to be updated on the remote MPI process.
 * \return void
 *
 * This function encodes the given rigid body for the send operation to the remote MPI
 * process. Note that it is invalid to send an infinite, fixed or remote sphere primitive!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::updateBody( ProcessID p, ConstBodyID b )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending update for " << b->getType() << " " << b->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << b->getPosition() << "\n"
          << "         Linear velocity  = " << b->getLinearVel() << "\n"
          << "         Angular velocity = " << b->getAngularVel();
   }

   // Encoding rigid body update
   Encoder::encodeUpdate( p->getSendBuffer(), b, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding the forces acting on the given body for the send operation to the remote MPI process.
 *
 * \param b The rigid body.
 * \return void
 *
 * This function encodes the force and torque acting on the given given rigid body for the send
 * operation to the remote MPI process. Note that it is invalid to send an infinite or fixed
 * rigid body!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::sendForce( ProcessID p, ConstBodyID b )
{
   // Checking the the finiteness and the mobility of the rigid body
   pe_INTERNAL_ASSERT( b->isFinite(), "Invalid infinite rigid body detected" );
   pe_INTERNAL_ASSERT( !b->isFixed(), "Invalid fixed rigid body detected"    );
   pe_INTERNAL_ASSERT( b->isRemote(), "Invalid local rigid body detected"    );

   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending force for " << b->getType() << " " << b->getSystemID() << " to process " << p->getRank();
   }

   // Encoding the acting force and torque
   p->getSendBuffer() << b->getSystemID();  // Sending the system ID of the rigid body
   marshal( p->getSendBuffer(), b->getForce() );     // Sending the acting force
   marshal( p->getSendBuffer(), b->getTorque() );    // Sending the acting torque
}
//*************************************************************************************************




//=================================================================================================
//
//  RECEIVE FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Receiving a single rigid body from the remote MPI process.
 *
 * \return The rigid body received from the remote MPI process.
 * \exception std::runtime_error Received invalid message tag.
 *
 * This function decodes a single rigid body from the message recevied from the remote MPI
 * process. In case a new, unknown rigid body is received, it is locally instantiated on this
 * process. Rigid bodies that are already known on this process are updated according to the
 * received informations.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::BodyID
   CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::recvBody( ProcessID p )
{
   // Decoding the rigid body
   const BodyID body = decoder_.decodeBody( p->getRecvBuffer() );

   // Logging the receive operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Receiving " << body->getType() << " " << body->getSystemID() << " from process " << p->getRank() << "\n"
          << "         Global position  = " << body->getPosition() << "\n"
          << "         Linear velocity  = " << body->getLinearVel() << "\n"
          << "         Angular velocity = " << body->getAngularVel();
   }

   return body;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding all forces and torques received from the remote MPI process.
 *
 * \return void
 *
 * This function decodes all forces and torques received from the remote MPI process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::recvForces( ProcessID p )
{
   const BodyIterator begin( bodystorage_.begin() );
   const BodyIterator end  ( bodystorage_.end()   );

   id_t sid;
   Vec3 f, t;

   // Extracting the received forces
   while( p->hasData() )
   {
      p->getRecvBuffer() >> sid;  // Extracting the unique system-specific ID
      unmarshal( p->getRecvBuffer(), f );    // Extracting the acting force
      unmarshal( p->getRecvBuffer(), t );    // Extracting the acting torque

      // Searching for the rigid body
      BodyIterator body( bodystorage_.find( sid ) );
      if( body == end || body->isRemote() ) continue;

      // Logging the receive operation
      pe_LOG_DETAIL_SECTION( log ) {
         log << "      Receiving force for " << body->getType() << " " << body->getSystemID() << " from process " << p->getRank();
      }

      // Adding the forces to the rigid body
      body->addForce ( f );
      body->addTorque( t );

      // Registering the process with the rigid body
      body->registerProcess( p );
   }
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
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::clear()
{
   detector_.clear();

   for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body )
      delete *body;
   bodystorage_.clear();
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
inline void CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::clearContacts()
{
   typename Contacts::Iterator       begin( contacts_.begin() );
   typename Contacts::Iterator const end  ( contacts_.end()   );

   for( ; begin!=end; ++begin )
      delete *begin;

   contacts_.clear();
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
inline bool CollisionSystem< C<CD,FD,BG,response::DEMSolverObsolete> >::checkUpdateFlags()
{
   bool error( false );

   const BodyIterator bbegin( bodystorage_.begin() );
   const BodyIterator bend  ( bodystorage_.end()   );

   for( BodyIterator body=bbegin; body!=bend; ++body ) {
      if( body->isRemote() && !body->isFixed() && !body->isUpdated() ) {
         if( !error ) {
            std::cerr << pe_RED << "\n **** Unmanaged rigid body detected ****\n" << pe_OLDCOLOR;
         }
         std::cerr << " Body: user-ID = " << body->getID() << ", system-ID = " << body->getSystemID()
                   << ", process " << MPISettings::rank() << " of " << MPISettings::size() << "\n";
         error = true;
      }
   }

   return !error;
}
//*************************************************************************************************

} // namespace pe

#endif
