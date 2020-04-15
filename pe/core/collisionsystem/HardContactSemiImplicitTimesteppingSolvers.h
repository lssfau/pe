//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/HardContactSemiImplicitTimesteppingSolvers.h
 *  \brief Specialization of the CollisionSystem class template for the hard contact solvers
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

#ifndef _PE_CORE_COLLISIONSYSTEM_HARDCONTACTSEMIIMPLICITTIMESTEPPINGSOLVERS_H_
#define _PE_CORE_COLLISIONSYSTEM_HARDCONTACTSEMIIMPLICITTIMESTEPPINGSOLVERS_H_


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
#include <pe/core/response/HardContactSemiImplicitTimesteppingSolvers.h>
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
#include <pe/core/notifications/RigidBodyVelocityUpdateNotification.h>
#include <pe/core/notifications/RigidBodyVelocityCorrectionNotification.h>
#include <pe/core/RootSection.h>
#include <pe/core/SerialSection.h>
#include <pe/core/Types.h>
#include <pe/math/Matrix2x2.h>
#include <pe/math/Vector2.h>
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
/*!\brief Specialization of the collision system for the hard contact solvers.
 * \ingroup core
 *
 * This specialization of the CollisionSystem class template adapts the collision system of the
 * rigid body simulation world to the requirements of the hard contact solvers.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >
   : public MPICommunication, public Singleton< CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >, logging::Logger >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers>      Config;             //!< Type of the configuration.

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
   typedef response::HardContactSemiImplicitTimesteppingSolvers<Config>  ContactSolver;

   enum RelaxationModel {
      InelasticFrictionlessContact,
      ApproximateInelasticCoulombContactByDecoupling,
      ApproximateInelasticCoulombContactByOrthogonalProjections,
      InelasticCoulombContactByDecoupling,
      InelasticCoulombContactByOrthogonalProjections,
      InelasticGeneralizedMaximumDissipationContact
   };
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
   inline real            getMaximumPenetration() const;
   inline size_t          getNumberOfContacts()   const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void            setRelaxationParameter( real f );
   inline void            setMaxIterations( size_t n );
   inline void            setRelaxationModel( RelaxationModel relaxationModel );
   inline void            setErrorReductionParameter( real erp );
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

   //**Helper functions*****************************************************************************
   /*!\name Helper functions */
   //@{
   inline void            clearShadowCopies();
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
   void simulationStep( real dt );
   void resolveContacts( const Contacts& contacts, real dt );
   real relaxInelasticFrictionlessContacts( real dtinv );
   real relaxApproximateInelasticCoulombContactsByDecoupling( real dtinv );
   real relaxInelasticCoulombContactsByDecoupling( real dtinv );
   real relaxInelasticCoulombContactsByOrthogonalProjections( real dtinv, bool approximate );
   real relaxInelasticGeneralizedMaximumDissipationContacts( real dtinv );
   //@}
   //**********************************************************************************************

   //**Communication functions*********************************************************************
   /*!\name Communication functions */
   //@{
   void synchronize();
   void synchronizeVelocities();
   //@}
   //**********************************************************************************************

   //**Time-integration functions******************************************************************
   /*!\name Time-integration functions */
   //@{
   void initializeVelocityCorrections( BodyID body, Vec3& dv, Vec3& dw, real dt ) const;
   void integratePositions( BodyID body, Vec3 v, Vec3 w, real dt ) const;
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

   real erp_;                 //!< The error reduction parameter (0 <= erp_ <= 1).
   size_t maxIterations_;     //!< Maximum number of iterations.
   size_t iteration_;
   size_t maxSubIterations_;  //!< Maximum number of iterations of iterative solvers in the one-contact problem.
   real abortThreshold_;      //!< If L-infinity iterate difference drops below this threshold the iteration is aborted.
   RelaxationModel relaxationModel_; //!< The method used to relax unilateral contacts
   real relaxationParam_;     //!< Parameter specifying underrelaxation of velocity corrections for boundary bodies.
   real maximumPenetration_;
   size_t numContacts_;
   MPITag lastSyncTag_;

   // bodies
   std::vector<Vec3> v_, w_, dv_, dw_;

   // contacts
   std::vector<bool> contactsMask_;
   std::vector<Vec3> r1_, r2_;
   std::vector<BodyID> body1_, body2_;
   std::vector<Vec3> n_, t_, o_;
   std::vector<real> dist_;
   std::vector<real> mu_;
   std::vector<Mat3> diag_nto_;
   std::vector<Mat3> diag_nto_inv_;
   std::vector<Mat2> diag_to_inv_;
   std::vector<real> diag_n_inv_;
   std::vector<Vec3> p_;

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

   timing::WcTimer timeSimulationStep_, timeCollisionDetection_, timeCollisionResponse_, timeCollisionResponseContactFiltering_, timeCollisionResponseContactCaching_, timeCollisionResponseBodyCaching_, timeCollisionResponseSolving_, timeCollisionResponseIntegration_, timeBodySync_, timeBodySyncAssembling_, timeBodySyncCommunicate_, timeBodySyncParsing_, timeVelocitiesSync_, timeVelocitiesSyncCorrectionsAssembling_, timeVelocitiesSyncCorrectionsCommunicate_, timeVelocitiesSyncCorrectionsParsing_, timeVelocitiesSyncUpdatesAssembling_, timeVelocitiesSyncUpdatesCommunicate_, timeVelocitiesSyncUpdatesParsing_, timeVelocitiesSyncGlobals_;
   std::vector<timing::WcTimer*> timers_;

   MemoryMeter  memSimulationStep_, memCollisionDetection_, memCollisionResponse_, memCollisionResponseContactFiltering_, memCollisionResponseContactCaching_, memCollisionResponseBodyCaching_, memCollisionResponseSolving_, memCollisionResponseIntegration_, memBodySync_, memBodySyncAssembling_, memBodySyncCommunicate_, memBodySyncParsing_, memVelocitiesSync_, memVelocitiesSyncCorrectionsAssembling_, memVelocitiesSyncCorrectionsCommunicate_, memVelocitiesSyncCorrectionsParsing_, memVelocitiesSyncUpdatesAssembling_, memVelocitiesSyncUpdatesCommunicate_, memVelocitiesSyncUpdatesParsing_, memVelocitiesSyncGlobals_;
   std::vector<MemoryMeter*> memoryMeters_;

   TransferMeter sentBodySync_, sentVelocitiesSyncCorrections_, sentVelocitiesSyncUpdates_;
   std::vector<TransferMeter*> sentMeters_;

   TransferMeter receivedBodySync_, receivedVelocitiesSyncCorrections_, receivedVelocitiesSyncUpdates_;
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
CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::CollisionSystem()
   : MPICommunication  ( processstorage_, domain_ )
   , Singleton<CollisionSystem,logging::Logger>()  // Initialization of the Singleton base object
   , contacts_         ( 5000 )                    // The currently active contacts of the simulation world
   , detector_         ( bodystorage_, bodystorageShadowCopies_ )  // The active collision detection algorithm
   , solver_           ()
   , bodystorage_      ()
   , bodystorageShadowCopies_ ()
   , processstorage_   ()
   , domain_           ( processstorage_ )
   , attachablestorage_()
   , jointstorage_     ()
   , erp_              ( real(0.7) )
   , maxIterations_    ( 100 )
   , iteration_        ( 0 )
   , maxSubIterations_ ( 20 )
   , abortThreshold_   ( real(1e-7) )
   , relaxationModel_  ( ApproximateInelasticCoulombContactByDecoupling )
   , relaxationParam_  ( real(0.9) )
   , maximumPenetration_ ( real(0.0) )
   , numContacts_      ( 0 )
   , lastSyncTag_      ( mpitagHCTSSynchronizePositionsAndVelocities2 )
   , requireSync_      ( false )
{
   // Registering all timers
   timers_.push_back( &timeSimulationStep_ );
   timers_.push_back( &timeCollisionDetection_ );
   timers_.push_back( &timeCollisionResponse_ );
   timers_.push_back( &timeCollisionResponseContactFiltering_ );
   timers_.push_back( &timeCollisionResponseContactCaching_ );
   timers_.push_back( &timeCollisionResponseBodyCaching_ );
   timers_.push_back( &timeCollisionResponseSolving_ );
   timers_.push_back( &timeCollisionResponseIntegration_ );
   timers_.push_back( &timeBodySync_ );
   timers_.push_back( &timeBodySyncAssembling_ );
   timers_.push_back( &timeBodySyncCommunicate_ );
   timers_.push_back( &timeBodySyncParsing_ );
   timers_.push_back( &timeVelocitiesSync_ );
   timers_.push_back( &timeVelocitiesSyncCorrectionsAssembling_ );
   timers_.push_back( &timeVelocitiesSyncCorrectionsCommunicate_ );
   timers_.push_back( &timeVelocitiesSyncCorrectionsParsing_ );
   timers_.push_back( &timeVelocitiesSyncUpdatesAssembling_ );
   timers_.push_back( &timeVelocitiesSyncUpdatesCommunicate_ );
   timers_.push_back( &timeVelocitiesSyncUpdatesParsing_ );
   timers_.push_back( &timeVelocitiesSyncGlobals_ );

   // Registering all memory meters
   memoryMeters_.push_back( &memSimulationStep_ );
   memoryMeters_.push_back( &memCollisionDetection_ );
   memoryMeters_.push_back( &memCollisionResponse_ );
   memoryMeters_.push_back( &memCollisionResponseContactFiltering_ );
   memoryMeters_.push_back( &memCollisionResponseContactCaching_ );
   memoryMeters_.push_back( &memCollisionResponseBodyCaching_ );
   memoryMeters_.push_back( &memCollisionResponseSolving_ );
   memoryMeters_.push_back( &memCollisionResponseIntegration_ );
   memoryMeters_.push_back( &memBodySync_ );
   memoryMeters_.push_back( &memBodySyncAssembling_ );
   memoryMeters_.push_back( &memBodySyncCommunicate_ );
   memoryMeters_.push_back( &memBodySyncParsing_ );
   memoryMeters_.push_back( &memVelocitiesSync_ );
   memoryMeters_.push_back( &memVelocitiesSyncCorrectionsAssembling_ );
   memoryMeters_.push_back( &memVelocitiesSyncCorrectionsCommunicate_ );
   memoryMeters_.push_back( &memVelocitiesSyncCorrectionsParsing_ );
   memoryMeters_.push_back( &memVelocitiesSyncUpdatesAssembling_ );
   memoryMeters_.push_back( &memVelocitiesSyncUpdatesCommunicate_ );
   memoryMeters_.push_back( &memVelocitiesSyncUpdatesParsing_ );
   memoryMeters_.push_back( &memVelocitiesSyncGlobals_ );

   // Registering all sent meters
   sentMeters_.push_back( &sentOverall_ );
   sentMeters_.push_back( &sentBodySync_ );
   sentMeters_.push_back( &sentVelocitiesSyncCorrections_ );
   sentMeters_.push_back( &sentVelocitiesSyncUpdates_ );

   // Registering all received meters
   receivedMeters_.push_back( &receivedOverall_ );
   receivedMeters_.push_back( &receivedBodySync_ );
   receivedMeters_.push_back( &receivedVelocitiesSyncCorrections_ );
   receivedMeters_.push_back( &receivedVelocitiesSyncUpdates_ );

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
CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::~CollisionSystem()
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
inline typename CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::CoarseDetector&
   CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getCoarseDetector()
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
inline typename CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::ContactSolver&
   CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getContactSolver()
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
inline typename CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::BS&
   CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getBodyStorage()
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
inline typename CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::BS&
   CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getBodyShadowCopyStorage()
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
inline const typename CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::PS&
   CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getProcessStorage() const
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
inline const typename CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::AS&
   CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getAttachableStorage() const
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
inline const Domain& CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getDomain() const
{
   return domain_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum penetration depth found in the last collision detection.
 *
 * \return The maximum penetration depth found in the last collision detection.
 *
 * Only contacts treated on the local process are considered.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline real CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getMaximumPenetration() const
{
   return maximumPenetration_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of contacts found by the last collision detection.
 *
 * \return The number of contacts found by the last collision detection.
 *
 * Only contacts treated on the local process are counted.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::getNumberOfContacts() const
{
   return numContacts_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets the relaxation parameter for boundary bodies.
 *
 * \param f The relaxation parameter.
 * \return void
 *
 * The iterative solvers are a mixture of non-linear Gauss-Seidel and Jacobi solvers. This might
 * require underrelaxation. The parameter must be positive. Note that for dilute systems the
 * solver might need stronger underrelaxation (smaller \a f) than for dense systems.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::setRelaxationParameter( real f )
{
   pe_INTERNAL_ASSERT( f > 0, "Relaxation parameter must be positive." );

   relaxationParam_ = f;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the maximum number of iterations performed by the iterative solver.
 *
 * \param n The maximum  number of iterations.
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::setMaxIterations( size_t n )
{
   maxIterations_ = n;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the relaxation model used by the iterative solver.
 *
 * \param relaxationModel The relaxation model to be used by the iterative solver.
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::setRelaxationModel( RelaxationModel relaxationModel )
{
   relaxationModel_ = relaxationModel;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the error reduction parameter.
 *
 * \param erp The error reduction parameter.
 * \return void
 *
 * If body shapes overlap by x at a contact then the contact resolution aims to remove erp*x of the
 * overlap. Thus the error reduction parameter must be between 0 and 1. 0 corresponds to no error
 * reduction and is the default. 1 corresponds to full error reduction. Note that error reduction
 * (constraint stabilization) introduces additional energy to the system.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::setErrorReductionParameter( real erp )
{
   pe_INTERNAL_ASSERT( erp >= 0 && erp <= 1, "Error reduction parameter out of range." );

   erp_ = erp;
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
inline bool CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::isSyncRequired() const
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
inline bool CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::isSyncRequiredLocally() const
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::logProfilingSummary() const
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
                << "simulation step        " << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "\n"
                << "  col. detection       " << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "\n"
                << "  col. response        " << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "\n"
                << "    contact filter     " << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "\n"
                << "    contact caching    " << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "\n"
                << "    body caching       " << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "\n"
                << "    solving            " << std::setw(10) << minValues[ 6] << "   " << std::setw(10) << maxValues[ 6] << "   " << std::setw(10) << totalValues[ 6] / numValues[ 6] << " = " << std::setw(10) << totalValues[ 6] << " / " << std::setw(10) << numValues[ 6] << "\n"
                << "    velocities sync    " << std::setw(10) << minValues[12] << "   " << std::setw(10) << maxValues[12] << "   " << std::setw(10) << totalValues[12] / numValues[12] << " = " << std::setw(10) << totalValues[12] << " / " << std::setw(10) << numValues[12] << "\n"
                << "      corr. assem.     " << std::setw(10) << minValues[13] << "   " << std::setw(10) << maxValues[13] << "   " << std::setw(10) << totalValues[13] / numValues[13] << " = " << std::setw(10) << totalValues[13] << " / " << std::setw(10) << numValues[13] << "\n"
                << "      corr. comm.      " << std::setw(10) << minValues[14] << "   " << std::setw(10) << maxValues[14] << "   " << std::setw(10) << totalValues[14] / numValues[14] << " = " << std::setw(10) << totalValues[14] << " / " << std::setw(10) << numValues[14] << "\n"
                << "      corr. pars.      " << std::setw(10) << minValues[15] << "   " << std::setw(10) << maxValues[15] << "   " << std::setw(10) << totalValues[15] / numValues[15] << " = " << std::setw(10) << totalValues[15] << " / " << std::setw(10) << numValues[15] << "\n"
                << "      upd. assem.      " << std::setw(10) << minValues[16] << "   " << std::setw(10) << maxValues[16] << "   " << std::setw(10) << totalValues[16] / numValues[16] << " = " << std::setw(10) << totalValues[16] << " / " << std::setw(10) << numValues[16] << "\n"
                << "      upd. comm.       " << std::setw(10) << minValues[17] << "   " << std::setw(10) << maxValues[17] << "   " << std::setw(10) << totalValues[17] / numValues[17] << " = " << std::setw(10) << totalValues[17] << " / " << std::setw(10) << numValues[17] << "\n"
                << "      upd. pars.       " << std::setw(10) << minValues[18] << "   " << std::setw(10) << maxValues[18] << "   " << std::setw(10) << totalValues[18] / numValues[18] << " = " << std::setw(10) << totalValues[18] << " / " << std::setw(10) << numValues[18] << "\n"
                << "      sync globals     " << std::setw(10) << minValues[19] << "   " << std::setw(10) << maxValues[19] << "   " << std::setw(10) << totalValues[19] / numValues[19] << " = " << std::setw(10) << totalValues[19] << " / " << std::setw(10) << numValues[19] << "\n"
                << "    integration        " << std::setw(10) << minValues[ 7] << "   " << std::setw(10) << maxValues[ 7] << "   " << std::setw(10) << totalValues[ 7] / numValues[ 7] << " = " << std::setw(10) << totalValues[ 7] << " / " << std::setw(10) << numValues[ 7] << "\n"
                << "  body sync            " << std::setw(10) << minValues[ 8] << "   " << std::setw(10) << maxValues[ 8] << "   " << std::setw(10) << totalValues[ 8] / numValues[ 8] << " = " << std::setw(10) << totalValues[ 8] << " / " << std::setw(10) << numValues[ 8] << "\n"
                << "    assembling         " << std::setw(10) << minValues[ 9] << "   " << std::setw(10) << maxValues[ 9] << "   " << std::setw(10) << totalValues[ 9] / numValues[ 9] << " = " << std::setw(10) << totalValues[ 9] << " / " << std::setw(10) << numValues[ 9] << "\n"
                << "    communicate        " << std::setw(10) << minValues[10] << "   " << std::setw(10) << maxValues[10] << "   " << std::setw(10) << totalValues[10] / numValues[10] << " = " << std::setw(10) << totalValues[10] << " / " << std::setw(10) << numValues[10] << "\n"
                << "    parsing            " << std::setw(10) << minValues[11] << "   " << std::setw(10) << maxValues[11] << "   " << std::setw(10) << totalValues[11] / numValues[11] << " = " << std::setw(10) << totalValues[11] << " / " << std::setw(10) << numValues[11] << "\n"
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
                      << "simulation step        " << std::setw(10) << minValues[ 0] << "   " << std::setw(10) << maxValues[ 0] << "   " << std::setw(10) << totalValues[ 0] / numValues[ 0] << " = " << std::setw(10) << totalValues[ 0] << " / " << std::setw(10) << numValues[ 0] << "\n"
                      << "  col. detection       " << std::setw(10) << minValues[ 1] << "   " << std::setw(10) << maxValues[ 1] << "   " << std::setw(10) << totalValues[ 1] / numValues[ 1] << " = " << std::setw(10) << totalValues[ 1] << " / " << std::setw(10) << numValues[ 1] << "\n"
                      << "  col. response        " << std::setw(10) << minValues[ 2] << "   " << std::setw(10) << maxValues[ 2] << "   " << std::setw(10) << totalValues[ 2] / numValues[ 2] << " = " << std::setw(10) << totalValues[ 2] << " / " << std::setw(10) << numValues[ 2] << "\n"
                      << "    contact filter     " << std::setw(10) << minValues[ 3] << "   " << std::setw(10) << maxValues[ 3] << "   " << std::setw(10) << totalValues[ 3] / numValues[ 3] << " = " << std::setw(10) << totalValues[ 3] << " / " << std::setw(10) << numValues[ 3] << "\n"
                      << "    contact caching    " << std::setw(10) << minValues[ 4] << "   " << std::setw(10) << maxValues[ 4] << "   " << std::setw(10) << totalValues[ 4] / numValues[ 4] << " = " << std::setw(10) << totalValues[ 4] << " / " << std::setw(10) << numValues[ 4] << "\n"
                      << "    body caching       " << std::setw(10) << minValues[ 5] << "   " << std::setw(10) << maxValues[ 5] << "   " << std::setw(10) << totalValues[ 5] / numValues[ 5] << " = " << std::setw(10) << totalValues[ 5] << " / " << std::setw(10) << numValues[ 5] << "\n"
                      << "    solving            " << std::setw(10) << minValues[ 6] << "   " << std::setw(10) << maxValues[ 6] << "   " << std::setw(10) << totalValues[ 6] / numValues[ 6] << " = " << std::setw(10) << totalValues[ 6] << " / " << std::setw(10) << numValues[ 6] << "\n"
                      << "    velocities sync    " << std::setw(10) << minValues[12] << "   " << std::setw(10) << maxValues[12] << "   " << std::setw(10) << totalValues[12] / numValues[12] << " = " << std::setw(10) << totalValues[12] << " / " << std::setw(10) << numValues[12] << "\n"
                      << "      corr. assem.     " << std::setw(10) << minValues[13] << "   " << std::setw(10) << maxValues[13] << "   " << std::setw(10) << totalValues[13] / numValues[13] << " = " << std::setw(10) << totalValues[13] << " / " << std::setw(10) << numValues[13] << "\n"
                      << "      corr. comm.      " << std::setw(10) << minValues[14] << "   " << std::setw(10) << maxValues[14] << "   " << std::setw(10) << totalValues[14] / numValues[14] << " = " << std::setw(10) << totalValues[14] << " / " << std::setw(10) << numValues[14] << "\n"
                      << "      corr. pars.      " << std::setw(10) << minValues[15] << "   " << std::setw(10) << maxValues[15] << "   " << std::setw(10) << totalValues[15] / numValues[15] << " = " << std::setw(10) << totalValues[15] << " / " << std::setw(10) << numValues[15] << "\n"
                      << "      upd. assem.      " << std::setw(10) << minValues[16] << "   " << std::setw(10) << maxValues[16] << "   " << std::setw(10) << totalValues[16] / numValues[16] << " = " << std::setw(10) << totalValues[16] << " / " << std::setw(10) << numValues[16] << "\n"
                      << "      upd. comm.       " << std::setw(10) << minValues[17] << "   " << std::setw(10) << maxValues[17] << "   " << std::setw(10) << totalValues[17] / numValues[17] << " = " << std::setw(10) << totalValues[17] << " / " << std::setw(10) << numValues[17] << "\n"
                      << "      upd. pars.       " << std::setw(10) << minValues[18] << "   " << std::setw(10) << maxValues[18] << "   " << std::setw(10) << totalValues[18] / numValues[18] << " = " << std::setw(10) << totalValues[18] << " / " << std::setw(10) << numValues[18] << "\n"
                      << "      sync globals     " << std::setw(10) << minValues[19] << "   " << std::setw(10) << maxValues[19] << "   " << std::setw(10) << totalValues[19] / numValues[19] << " = " << std::setw(10) << totalValues[19] << " / " << std::setw(10) << numValues[19] << "\n"
                      << "    integration        " << std::setw(10) << minValues[ 7] << "   " << std::setw(10) << maxValues[ 7] << "   " << std::setw(10) << totalValues[ 7] / numValues[ 7] << " = " << std::setw(10) << totalValues[ 7] << " / " << std::setw(10) << numValues[ 7] << "\n"
                      << "  body sync            " << std::setw(10) << minValues[ 8] << "   " << std::setw(10) << maxValues[ 8] << "   " << std::setw(10) << totalValues[ 8] / numValues[ 8] << " = " << std::setw(10) << totalValues[ 8] << " / " << std::setw(10) << numValues[ 8] << "\n"
                      << "    assembling         " << std::setw(10) << minValues[ 9] << "   " << std::setw(10) << maxValues[ 9] << "   " << std::setw(10) << totalValues[ 9] / numValues[ 9] << " = " << std::setw(10) << totalValues[ 9] << " / " << std::setw(10) << numValues[ 9] << "\n"
                      << "    communicate        " << std::setw(10) << minValues[10] << "   " << std::setw(10) << maxValues[10] << "   " << std::setw(10) << totalValues[10] / numValues[10] << " = " << std::setw(10) << totalValues[10] << " / " << std::setw(10) << numValues[10] << "\n"
                      << "    parsing            " << std::setw(10) << minValues[11] << "   " << std::setw(10) << maxValues[11] << "   " << std::setw(10) << totalValues[11] / numValues[11] << " = " << std::setw(10) << totalValues[11] << " / " << std::setw(10) << numValues[11] << "\n"
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
                << "simulation step        " << std::setw(10) << minAllocValues[ 0] << "   " << std::setw(10) << maxAllocValues[ 0] << "   " << std::setw(10) << totalAllocValues[ 0] << "   " << std::setw(10) << minInUseValues[ 0] << "   " << std::setw(10) << maxInUseValues[ 0] << "   " << std::setw(10) << totalInUseValues[ 0] << "   " << std::setw(10) << numAllocValues[ 0] << "\n"
                << "  col. detection       " << std::setw(10) << minAllocValues[ 1] << "   " << std::setw(10) << maxAllocValues[ 1] << "   " << std::setw(10) << totalAllocValues[ 1] << "   " << std::setw(10) << minInUseValues[ 1] << "   " << std::setw(10) << maxInUseValues[ 1] << "   " << std::setw(10) << totalInUseValues[ 1] << "   " << std::setw(10) << numAllocValues[ 1] << "\n"
                << "  col. response        " << std::setw(10) << minAllocValues[ 2] << "   " << std::setw(10) << maxAllocValues[ 2] << "   " << std::setw(10) << totalAllocValues[ 2] << "   " << std::setw(10) << minInUseValues[ 2] << "   " << std::setw(10) << maxInUseValues[ 2] << "   " << std::setw(10) << totalInUseValues[ 2] << "   " << std::setw(10) << numAllocValues[ 2] << "\n"
                << "    contact filter     " << std::setw(10) << minAllocValues[ 3] << "   " << std::setw(10) << maxAllocValues[ 3] << "   " << std::setw(10) << totalAllocValues[ 3] << "   " << std::setw(10) << minInUseValues[ 3] << "   " << std::setw(10) << maxInUseValues[ 3] << "   " << std::setw(10) << totalInUseValues[ 3] << "   " << std::setw(10) << numAllocValues[ 3] << "\n"
                << "    contact caching    " << std::setw(10) << minAllocValues[ 4] << "   " << std::setw(10) << maxAllocValues[ 4] << "   " << std::setw(10) << totalAllocValues[ 4] << "   " << std::setw(10) << minInUseValues[ 4] << "   " << std::setw(10) << maxInUseValues[ 4] << "   " << std::setw(10) << totalInUseValues[ 4] << "   " << std::setw(10) << numAllocValues[ 4] << "\n"
                << "    body caching       " << std::setw(10) << minAllocValues[ 5] << "   " << std::setw(10) << maxAllocValues[ 5] << "   " << std::setw(10) << totalAllocValues[ 5] << "   " << std::setw(10) << minInUseValues[ 5] << "   " << std::setw(10) << maxInUseValues[ 5] << "   " << std::setw(10) << totalInUseValues[ 5] << "   " << std::setw(10) << numAllocValues[ 5] << "\n"
                << "    solving            " << std::setw(10) << minAllocValues[ 6] << "   " << std::setw(10) << maxAllocValues[ 6] << "   " << std::setw(10) << totalAllocValues[ 6] << "   " << std::setw(10) << minInUseValues[ 6] << "   " << std::setw(10) << maxInUseValues[ 6] << "   " << std::setw(10) << totalInUseValues[ 6] << "   " << std::setw(10) << numAllocValues[ 6] << "\n"
                << "    velocities sync    " << std::setw(10) << minAllocValues[12] << "   " << std::setw(10) << maxAllocValues[12] << "   " << std::setw(10) << totalAllocValues[12] << "   " << std::setw(10) << minInUseValues[12] << "   " << std::setw(10) << maxInUseValues[12] << "   " << std::setw(10) << totalInUseValues[12] << "   " << std::setw(10) << numAllocValues[12] << "\n"
                << "      corr. assem.     " << std::setw(10) << minAllocValues[13] << "   " << std::setw(10) << maxAllocValues[13] << "   " << std::setw(10) << totalAllocValues[13] << "   " << std::setw(10) << minInUseValues[13] << "   " << std::setw(10) << maxInUseValues[13] << "   " << std::setw(10) << totalInUseValues[13] << "   " << std::setw(10) << numAllocValues[13] << "\n"
                << "      corr. comm.      " << std::setw(10) << minAllocValues[14] << "   " << std::setw(10) << maxAllocValues[14] << "   " << std::setw(10) << totalAllocValues[14] << "   " << std::setw(10) << minInUseValues[14] << "   " << std::setw(10) << maxInUseValues[14] << "   " << std::setw(10) << totalInUseValues[14] << "   " << std::setw(10) << numAllocValues[14] << "\n"
                << "      corr. pars.      " << std::setw(10) << minAllocValues[15] << "   " << std::setw(10) << maxAllocValues[15] << "   " << std::setw(10) << totalAllocValues[15] << "   " << std::setw(10) << minInUseValues[15] << "   " << std::setw(10) << maxInUseValues[15] << "   " << std::setw(10) << totalInUseValues[15] << "   " << std::setw(10) << numAllocValues[15] << "\n"
                << "      upd. assem.      " << std::setw(10) << minAllocValues[16] << "   " << std::setw(10) << maxAllocValues[16] << "   " << std::setw(10) << totalAllocValues[16] << "   " << std::setw(10) << minInUseValues[16] << "   " << std::setw(10) << maxInUseValues[16] << "   " << std::setw(10) << totalInUseValues[16] << "   " << std::setw(10) << numAllocValues[16] << "\n"
                << "      upd. comm.       " << std::setw(10) << minAllocValues[17] << "   " << std::setw(10) << maxAllocValues[17] << "   " << std::setw(10) << totalAllocValues[17] << "   " << std::setw(10) << minInUseValues[17] << "   " << std::setw(10) << maxInUseValues[17] << "   " << std::setw(10) << totalInUseValues[17] << "   " << std::setw(10) << numAllocValues[17] << "\n"
                << "      upd. pars.       " << std::setw(10) << minAllocValues[18] << "   " << std::setw(10) << maxAllocValues[18] << "   " << std::setw(10) << totalAllocValues[18] << "   " << std::setw(10) << minInUseValues[18] << "   " << std::setw(10) << maxInUseValues[18] << "   " << std::setw(10) << totalInUseValues[18] << "   " << std::setw(10) << numAllocValues[18] << "\n"
                << "      sync globals     " << std::setw(10) << minAllocValues[19] << "   " << std::setw(10) << maxAllocValues[19] << "   " << std::setw(10) << totalAllocValues[19] << "   " << std::setw(10) << minInUseValues[19] << "   " << std::setw(10) << maxInUseValues[19] << "   " << std::setw(10) << totalInUseValues[19] << "   " << std::setw(10) << numAllocValues[19] << "\n"
                << "    integration        " << std::setw(10) << minAllocValues[ 7] << "   " << std::setw(10) << maxAllocValues[ 7] << "   " << std::setw(10) << totalAllocValues[ 7] << "   " << std::setw(10) << minInUseValues[ 7] << "   " << std::setw(10) << maxInUseValues[ 7] << "   " << std::setw(10) << totalInUseValues[ 7] << "   " << std::setw(10) << numAllocValues[ 7] << "\n"
                << "  body sync            " << std::setw(10) << minAllocValues[ 8] << "   " << std::setw(10) << maxAllocValues[ 8] << "   " << std::setw(10) << totalAllocValues[ 8] << "   " << std::setw(10) << minInUseValues[ 8] << "   " << std::setw(10) << maxInUseValues[ 8] << "   " << std::setw(10) << totalInUseValues[ 8] << "   " << std::setw(10) << numAllocValues[ 8] << "\n"
                << "    assembling         " << std::setw(10) << minAllocValues[ 9] << "   " << std::setw(10) << maxAllocValues[ 9] << "   " << std::setw(10) << totalAllocValues[ 9] << "   " << std::setw(10) << minInUseValues[ 9] << "   " << std::setw(10) << maxInUseValues[ 9] << "   " << std::setw(10) << totalInUseValues[ 9] << "   " << std::setw(10) << numAllocValues[ 9] << "\n"
                << "    communication      " << std::setw(10) << minAllocValues[10] << "   " << std::setw(10) << maxAllocValues[10] << "   " << std::setw(10) << totalAllocValues[10] << "   " << std::setw(10) << minInUseValues[10] << "   " << std::setw(10) << maxInUseValues[10] << "   " << std::setw(10) << totalInUseValues[10] << "   " << std::setw(10) << numAllocValues[10] << "\n"
                << "    parsing            " << std::setw(10) << minAllocValues[11] << "   " << std::setw(10) << maxAllocValues[11] << "   " << std::setw(10) << totalAllocValues[11] << "   " << std::setw(10) << minInUseValues[11] << "   " << std::setw(10) << maxInUseValues[11] << "   " << std::setw(10) << totalInUseValues[11] << "   " << std::setw(10) << numAllocValues[11] << "\n"
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
                      << "simulation step        " << std::setw(10) << minAllocValues[ 0] << "   " << std::setw(10) << maxAllocValues[ 0] << "   " << std::setw(10) << totalAllocValues[ 0] << "   " << std::setw(10) << minInUseValues[ 0] << "   " << std::setw(10) << maxInUseValues[ 0] << "   " << std::setw(10) << totalInUseValues[ 0] << "   " << std::setw(10) << numAllocValues[ 0] << "\n"
                      << "  col. detection       " << std::setw(10) << minAllocValues[ 1] << "   " << std::setw(10) << maxAllocValues[ 1] << "   " << std::setw(10) << totalAllocValues[ 1] << "   " << std::setw(10) << minInUseValues[ 1] << "   " << std::setw(10) << maxInUseValues[ 1] << "   " << std::setw(10) << totalInUseValues[ 1] << "   " << std::setw(10) << numAllocValues[ 1] << "\n"
                      << "  col. response        " << std::setw(10) << minAllocValues[ 2] << "   " << std::setw(10) << maxAllocValues[ 2] << "   " << std::setw(10) << totalAllocValues[ 2] << "   " << std::setw(10) << minInUseValues[ 2] << "   " << std::setw(10) << maxInUseValues[ 2] << "   " << std::setw(10) << totalInUseValues[ 2] << "   " << std::setw(10) << numAllocValues[ 2] << "\n"
                      << "    contact filter     " << std::setw(10) << minAllocValues[ 3] << "   " << std::setw(10) << maxAllocValues[ 3] << "   " << std::setw(10) << totalAllocValues[ 3] << "   " << std::setw(10) << minInUseValues[ 3] << "   " << std::setw(10) << maxInUseValues[ 3] << "   " << std::setw(10) << totalInUseValues[ 3] << "   " << std::setw(10) << numAllocValues[ 3] << "\n"
                      << "    contact caching    " << std::setw(10) << minAllocValues[ 4] << "   " << std::setw(10) << maxAllocValues[ 4] << "   " << std::setw(10) << totalAllocValues[ 4] << "   " << std::setw(10) << minInUseValues[ 4] << "   " << std::setw(10) << maxInUseValues[ 4] << "   " << std::setw(10) << totalInUseValues[ 4] << "   " << std::setw(10) << numAllocValues[ 4] << "\n"
                      << "    body caching       " << std::setw(10) << minAllocValues[ 5] << "   " << std::setw(10) << maxAllocValues[ 5] << "   " << std::setw(10) << totalAllocValues[ 5] << "   " << std::setw(10) << minInUseValues[ 5] << "   " << std::setw(10) << maxInUseValues[ 5] << "   " << std::setw(10) << totalInUseValues[ 5] << "   " << std::setw(10) << numAllocValues[ 5] << "\n"
                      << "    solving            " << std::setw(10) << minAllocValues[ 6] << "   " << std::setw(10) << maxAllocValues[ 6] << "   " << std::setw(10) << totalAllocValues[ 6] << "   " << std::setw(10) << minInUseValues[ 6] << "   " << std::setw(10) << maxInUseValues[ 6] << "   " << std::setw(10) << totalInUseValues[ 6] << "   " << std::setw(10) << numAllocValues[ 6] << "\n"
                      << "    velocities sync    " << std::setw(10) << minAllocValues[12] << "   " << std::setw(10) << maxAllocValues[12] << "   " << std::setw(10) << totalAllocValues[12] << "   " << std::setw(10) << minInUseValues[12] << "   " << std::setw(10) << maxInUseValues[12] << "   " << std::setw(10) << totalInUseValues[12] << "   " << std::setw(10) << numAllocValues[12] << "\n"
                      << "      corr. assem.     " << std::setw(10) << minAllocValues[13] << "   " << std::setw(10) << maxAllocValues[13] << "   " << std::setw(10) << totalAllocValues[13] << "   " << std::setw(10) << minInUseValues[13] << "   " << std::setw(10) << maxInUseValues[13] << "   " << std::setw(10) << totalInUseValues[13] << "   " << std::setw(10) << numAllocValues[13] << "\n"
                      << "      corr. comm.      " << std::setw(10) << minAllocValues[14] << "   " << std::setw(10) << maxAllocValues[14] << "   " << std::setw(10) << totalAllocValues[14] << "   " << std::setw(10) << minInUseValues[14] << "   " << std::setw(10) << maxInUseValues[14] << "   " << std::setw(10) << totalInUseValues[14] << "   " << std::setw(10) << numAllocValues[14] << "\n"
                      << "      corr. pars.      " << std::setw(10) << minAllocValues[15] << "   " << std::setw(10) << maxAllocValues[15] << "   " << std::setw(10) << totalAllocValues[15] << "   " << std::setw(10) << minInUseValues[15] << "   " << std::setw(10) << maxInUseValues[15] << "   " << std::setw(10) << totalInUseValues[15] << "   " << std::setw(10) << numAllocValues[15] << "\n"
                      << "      upd. assem.      " << std::setw(10) << minAllocValues[16] << "   " << std::setw(10) << maxAllocValues[16] << "   " << std::setw(10) << totalAllocValues[16] << "   " << std::setw(10) << minInUseValues[16] << "   " << std::setw(10) << maxInUseValues[16] << "   " << std::setw(10) << totalInUseValues[16] << "   " << std::setw(10) << numAllocValues[16] << "\n"
                      << "      upd. comm.       " << std::setw(10) << minAllocValues[17] << "   " << std::setw(10) << maxAllocValues[17] << "   " << std::setw(10) << totalAllocValues[17] << "   " << std::setw(10) << minInUseValues[17] << "   " << std::setw(10) << maxInUseValues[17] << "   " << std::setw(10) << totalInUseValues[17] << "   " << std::setw(10) << numAllocValues[17] << "\n"
                      << "      upd. pars.       " << std::setw(10) << minAllocValues[18] << "   " << std::setw(10) << maxAllocValues[18] << "   " << std::setw(10) << totalAllocValues[18] << "   " << std::setw(10) << minInUseValues[18] << "   " << std::setw(10) << maxInUseValues[18] << "   " << std::setw(10) << totalInUseValues[18] << "   " << std::setw(10) << numAllocValues[18] << "\n"
                      << "      sync globals     " << std::setw(10) << minAllocValues[19] << "   " << std::setw(10) << maxAllocValues[19] << "   " << std::setw(10) << totalAllocValues[19] << "   " << std::setw(10) << minInUseValues[19] << "   " << std::setw(10) << maxInUseValues[19] << "   " << std::setw(10) << totalInUseValues[19] << "   " << std::setw(10) << numAllocValues[19] << "\n"
                      << "    integration        " << std::setw(10) << minAllocValues[ 7] << "   " << std::setw(10) << maxAllocValues[ 7] << "   " << std::setw(10) << totalAllocValues[ 7] << "   " << std::setw(10) << minInUseValues[ 7] << "   " << std::setw(10) << maxInUseValues[ 7] << "   " << std::setw(10) << totalInUseValues[ 7] << "   " << std::setw(10) << numAllocValues[ 7] << "\n"
                      << "  body sync            " << std::setw(10) << minAllocValues[ 8] << "   " << std::setw(10) << maxAllocValues[ 8] << "   " << std::setw(10) << totalAllocValues[ 8] << "   " << std::setw(10) << minInUseValues[ 8] << "   " << std::setw(10) << maxInUseValues[ 8] << "   " << std::setw(10) << totalInUseValues[ 8] << "   " << std::setw(10) << numAllocValues[ 8] << "\n"
                      << "    assembling         " << std::setw(10) << minAllocValues[ 9] << "   " << std::setw(10) << maxAllocValues[ 9] << "   " << std::setw(10) << totalAllocValues[ 9] << "   " << std::setw(10) << minInUseValues[ 9] << "   " << std::setw(10) << maxInUseValues[ 9] << "   " << std::setw(10) << totalInUseValues[ 9] << "   " << std::setw(10) << numAllocValues[ 9] << "\n"
                      << "    communication      " << std::setw(10) << minAllocValues[10] << "   " << std::setw(10) << maxAllocValues[10] << "   " << std::setw(10) << totalAllocValues[10] << "   " << std::setw(10) << minInUseValues[10] << "   " << std::setw(10) << maxInUseValues[10] << "   " << std::setw(10) << totalInUseValues[10] << "   " << std::setw(10) << numAllocValues[10] << "\n"
                      << "    parsing            " << std::setw(10) << minAllocValues[11] << "   " << std::setw(10) << maxAllocValues[11] << "   " << std::setw(10) << totalAllocValues[11] << "   " << std::setw(10) << minInUseValues[11] << "   " << std::setw(10) << maxInUseValues[11] << "   " << std::setw(10) << totalInUseValues[11] << "   " << std::setw(10) << numAllocValues[11] << "\n"
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
                << "collision system       " << std::setw(10) << sentMinBytes[0] << "   " << std::setw(10) << sentMaxBytes[0] << "   " << std::setw(10) << (sentNumTransfers[0] == 0 ? 0 : sentTotalBytes[0] / sentNumTransfers[0]) << " = " << std::setw(10) << sentTotalBytes[0] << " / " << std::setw(10) << sentNumTransfers[0] << "\n"
                << "  body sync            " << std::setw(10) << sentMinBytes[1] << "   " << std::setw(10) << sentMaxBytes[1] << "   " << std::setw(10) << (sentNumTransfers[1] == 0 ? 0 : sentTotalBytes[1] / sentNumTransfers[1]) << " = " << std::setw(10) << sentTotalBytes[1] << " / " << std::setw(10) << sentNumTransfers[1] << "\n"
                << "  vel. sync corr.      " << std::setw(10) << sentMinBytes[2] << "   " << std::setw(10) << sentMaxBytes[2] << "   " << std::setw(10) << (sentNumTransfers[2] == 0 ? 0 : sentTotalBytes[2] / sentNumTransfers[2]) << " = " << std::setw(10) << sentTotalBytes[2] << " / " << std::setw(10) << sentNumTransfers[2] << "\n"
                << "  vel. sync upd.       " << std::setw(10) << sentMinBytes[3] << "   " << std::setw(10) << sentMaxBytes[3] << "   " << std::setw(10) << (sentNumTransfers[3] == 0 ? 0 : sentTotalBytes[3] / sentNumTransfers[3]) << " = " << std::setw(10) << sentTotalBytes[3] << " / " << std::setw(10) << sentNumTransfers[3] << "\n"
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
                      << "collision system       " << std::setw(10) << sentMinBytes[0] << "   " << std::setw(10) << sentMaxBytes[0] << "   " << std::setw(10) << (sentNumTransfers[0] == 0 ? 0 : sentTotalBytes[0] / sentNumTransfers[0]) << " = " << std::setw(10) << sentTotalBytes[0] << " / " << std::setw(10) << sentNumTransfers[0] << "\n"
                      << "  body sync            " << std::setw(10) << sentMinBytes[1] << "   " << std::setw(10) << sentMaxBytes[1] << "   " << std::setw(10) << (sentNumTransfers[1] == 0 ? 0 : sentTotalBytes[1] / sentNumTransfers[1]) << " = " << std::setw(10) << sentTotalBytes[1] << " / " << std::setw(10) << sentNumTransfers[1] << "\n"
                      << "  vel. sync corr.      " << std::setw(10) << sentMinBytes[2] << "   " << std::setw(10) << sentMaxBytes[2] << "   " << std::setw(10) << (sentNumTransfers[2] == 0 ? 0 : sentTotalBytes[2] / sentNumTransfers[2]) << " = " << std::setw(10) << sentTotalBytes[2] << " / " << std::setw(10) << sentNumTransfers[2] << "\n"
                      << "  vel. sync upd.       " << std::setw(10) << sentMinBytes[3] << "   " << std::setw(10) << sentMaxBytes[3] << "   " << std::setw(10) << (sentNumTransfers[3] == 0 ? 0 : sentTotalBytes[3] / sentNumTransfers[3]) << " = " << std::setw(10) << sentTotalBytes[3] << " / " << std::setw(10) << sentNumTransfers[3] << "\n"
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
                << "collision system       " << std::setw(10) << receivedMinBytes[0] << "   " << std::setw(10) << receivedMaxBytes[0] << "   " << std::setw(10) << (receivedNumTransfers[0] == 0 ? 0 : receivedTotalBytes[0] / receivedNumTransfers[0]) << " = " << std::setw(10) << receivedTotalBytes[0] << " / " << std::setw(10) << receivedNumTransfers[0] << "\n"
                << "  body sync            " << std::setw(10) << receivedMinBytes[1] << "   " << std::setw(10) << receivedMaxBytes[1] << "   " << std::setw(10) << (receivedNumTransfers[1] == 0 ? 0 : receivedTotalBytes[1] / receivedNumTransfers[1]) << " = " << std::setw(10) << receivedTotalBytes[1] << " / " << std::setw(10) << receivedNumTransfers[1] << "\n"
                << "  vel. sync corr.      " << std::setw(10) << receivedMinBytes[2] << "   " << std::setw(10) << receivedMaxBytes[2] << "   " << std::setw(10) << (receivedNumTransfers[2] == 0 ? 0 : receivedTotalBytes[2] / receivedNumTransfers[2]) << " = " << std::setw(10) << receivedTotalBytes[2] << " / " << std::setw(10) << receivedNumTransfers[2] << "\n"
                << "  vel. sync upd.       " << std::setw(10) << receivedMinBytes[3] << "   " << std::setw(10) << receivedMaxBytes[3] << "   " << std::setw(10) << (receivedNumTransfers[3] == 0 ? 0 : receivedTotalBytes[3] / receivedNumTransfers[3]) << " = " << std::setw(10) << receivedTotalBytes[3] << " / " << std::setw(10) << receivedNumTransfers[3] << "\n"
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
                      << "collision system       " << std::setw(10) << receivedMinBytes[0] << "   " << std::setw(10) << receivedMaxBytes[0] << "   " << std::setw(10) << (receivedNumTransfers[0] == 0 ? 0 : receivedTotalBytes[0] / receivedNumTransfers[0]) << " = " << std::setw(10) << receivedTotalBytes[0] << " / " << std::setw(10) << receivedNumTransfers[0] << "\n"
                      << "  body sync            " << std::setw(10) << receivedMinBytes[1] << "   " << std::setw(10) << receivedMaxBytes[1] << "   " << std::setw(10) << (receivedNumTransfers[1] == 0 ? 0 : receivedTotalBytes[1] / receivedNumTransfers[1]) << " = " << std::setw(10) << receivedTotalBytes[1] << " / " << std::setw(10) << receivedNumTransfers[1] << "\n"
                      << "  vel. sync corr.      " << std::setw(10) << receivedMinBytes[2] << "   " << std::setw(10) << receivedMaxBytes[2] << "   " << std::setw(10) << (receivedNumTransfers[2] == 0 ? 0 : receivedTotalBytes[2] / receivedNumTransfers[2]) << " = " << std::setw(10) << receivedTotalBytes[2] << " / " << std::setw(10) << receivedNumTransfers[2] << "\n"
                      << "  vel. sync upd.       " << std::setw(10) << receivedMinBytes[3] << "   " << std::setw(10) << receivedMaxBytes[3] << "   " << std::setw(10) << (receivedNumTransfers[3] == 0 ? 0 : receivedTotalBytes[3] / receivedNumTransfers[3]) << " = " << std::setw(10) << receivedTotalBytes[3] << " / " << std::setw(10) << receivedNumTransfers[3] << "\n"
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
//  HELPER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Arranges that shadow copies are retransmitted on next synchronization.
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::clearShadowCopies()
{
   for( BodyIterator it = bodystorage_.begin(); it != bodystorage_.end(); ++it ) {
      BodyID body( *it );

      for( ProcessIterator itt = body->beginProcesses(); itt != body->endProcesses(); ++itt ) {
         marshal( itt->getSendBuffer(), notificationType<RigidBodyRemovalNotification>() );
         marshal( itt->getSendBuffer(), RigidBodyRemovalNotification( *body ) );
      }

      body->clearProcesses();
   }

   requireSync_ = true;
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::add( BodyID body )
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::remove( BodyID body )
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
            log << "Notify registered process " << it->getRank() << " of deletion of body " << body->getSystemID() << ".\n";
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
inline typename CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::BodyIterator CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::remove( BodyIterator body )
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
            log << "Notify registered process " << it->getRank() << " of deletion of body " << body->getSystemID() << ".\n";
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::removeFromCollisionDetector( BodyID body )
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
void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::simulationStep( real dt )
{
   //pe_LOG_WARNING_SECTION( log ) {
   //   log << "New timestep!\n";
   //}
   pe_USER_ASSERT( !requireSync_, "Simulation requires synchronization before continuing." );

   pe_PROFILING_SECTION {
#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
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
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif
   }

   // Compute collision response forces and synchronize them
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Resolving contacts...\n";
   }

   resolveContacts( contacts_, dt );

   clearContacts();

   synchronize();

   pe_PROFILING_SECTION {
#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif

      timeSimulationStep_.end();
      memSimulationStep_.stop();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Resolves the given contacts.
 *
 * \param contacts The vector of contacts.
 * \param dt The time step.
 * \return void
 *
 * TODO
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::resolveContacts( const Contacts& contacts, real dt )
{
   const real dtinv( real(1) / dt );
   const int myRank( MPISettings::rank() );

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Resolving the " << contacts.size() << " contact(s)"
          << " (";
      for( typename Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c )
         log << " " << c->getID();
      log << " )...";
   }

   pe_PROFILING_SECTION {
      timeCollisionResponse_.start();
      memCollisionResponse_.start();

      timeCollisionResponseContactFiltering_.start();
      memCollisionResponseContactFiltering_.start();
   }

   // Filter out contacts
   size_t numContacts( contacts.size() );
   size_t numContactsMasked( 0 );

   contactsMask_.resize( numContacts );
   for( size_t i = 0; i < numContacts; ++i ) {
      const ContactID c( contacts[i] );
      BodyID b1( c->getBody1() );
      BodyID b2( c->getBody2() );

      contactsMask_[i] = false;

      pe_INTERNAL_ASSERT( !b1->isFixed() || !b2->isFixed(), "Invalid contact between two fixed objects." );

      /* Contact filtering rules
       *
       * L: Local body
       * G: Global body
       * R: Remote body
       *
       * Option 1:              Option 2:
       * +---+---+---+---+      +---+---+---+---+
       * |   | L | G | R |      |   | L | G | R |
       * +---+---+---+---+      +---+---+---+---+
       * | L | + | + | * |      | L |§/+| + | § |
       * +---+---+---+---+      +---+---+---+---+
       * | G | + | ~ | - |      | G | + | ~ | - |
       * +---+---+---+---+      +---+---+---+---+
       * | R | * | - | # |      | R | § | - | § |
       * +---+---+---+---+      +---+---+---+---+
       *
       *  + Accept contact unconditionally
       *  - Reject contact unconditionally
       *  * Accept contact if we own the contact point
       *  # Accept contact if we own the contact point and the owners of the involved bodies are not the same
       *  ~ Accept contact only on root process
       *  § Accept contact if we are the owner with the smallest rank witnessing the contact or if none of the owners witness the contact and we are the process with the smallest rank witnessing the contact
       *
       * Note: Local-global contacts actually require a reduction of the contact reactions applied to the global body (unless it is fixed).
       * => MPI_Allreduce for all non-fixed global bodies before time-integration.
       */

      if( !b1->isRemote() && !b2->isRemote() ) {
         // local-local, local-global, global-global contacts

         if( b1->isGlobal() && b2->isGlobal() ) {
            // Resolve global-global contacts only on root process
            if( myRank != MPISettings::root() ) {
               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Rejecting global-global contact " << *c << " on non-root process.\n";
               }
               continue;
            }
         }
         else {
            // Always resolve local-local and local-global contacts even if they are outside of our domain
         }
      }
      else {
         // local-remote, global-remote or remote-remote contact

         if( b1->isGlobal() || b2->isGlobal() ) {
            // Never resolve remote-global contacts
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Rejecting global-remote contact " << *c << ".\n";
            }
            continue;
         }
         else if( b1->isRemote() && b2->isRemote() && b1->getOwnerRank() == b2->getOwnerRank() ) {
            // Do not treat remote-remote contacts if it is a local-local contact at the owner.
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Rejecting remote-remote contact since it will be a local-local contact at the owner process: " << *c << ".\n";
            }
            continue;
         }
         else {
#if 0
            // Option 1
            if( !domain_.ownsPoint( c->getPosition() ) ) {
               pe_LOG_DEBUG_SECTION( log ) {
                  if( b1->isRemote() && b2->isRemote() )
                     log << "Rejecting remote-remote contact " << *c << " since we don't own it.\n";
                  else
                     log << "Rejecting remote-local contact " << *c << " since we don't own it.\n";
               }
               continue;
            }
#else
            // Option 2
            std::vector<int> bodyHoldersA, bodyHoldersB;

            bodyHoldersA.reserve( b1->sizeProcesses() + 2 );
            bodyHoldersA.push_back( b1->getOwnerRank() );
            if( b1->isRemote() )
               bodyHoldersA.push_back( myRank );
            for( ProcessIterator it = b1->beginProcesses(); it != b1->endProcesses(); ++it )
               bodyHoldersA.push_back( it->getRank() );
            std::sort( bodyHoldersA.begin(), bodyHoldersA.end() );

            bodyHoldersB.reserve( b2->sizeProcesses() + 2 );
            bodyHoldersB.push_back( b2->getOwnerRank() );
            if( b2->isRemote() )
               bodyHoldersB.push_back( myRank );
            for( ProcessIterator it = b2->beginProcesses(); it != b2->endProcesses(); ++it )
               bodyHoldersB.push_back( it->getRank() );
            std::sort( bodyHoldersB.begin(), bodyHoldersB.end() );

            int contactOwner( std::numeric_limits<int>::max() );

            // If one of the body owners is a contact witness then make it the contact owner, if both are contact witnesses then make the owner with the smaller process rank the contact owner.
            if( b1->getOwnerRank() < b2->getOwnerRank() ) {
               if( std::binary_search( bodyHoldersB.begin(), bodyHoldersB.end(), b1->getOwnerRank() ) )
                  contactOwner = b1->getOwnerRank();
               else if( std::binary_search( bodyHoldersA.begin(), bodyHoldersA.end(), b2->getOwnerRank() ) )
                  contactOwner = b2->getOwnerRank();
            }
            else {
               if( std::binary_search( bodyHoldersA.begin(), bodyHoldersA.end(), b2->getOwnerRank() ) )
                  contactOwner = b2->getOwnerRank();
               else if( std::binary_search( bodyHoldersB.begin(), bodyHoldersB.end(), b1->getOwnerRank() ) )
                  contactOwner = b1->getOwnerRank();
            }

            // If neither of the body owners is a contact witness then choose the contact witness with smallest process rank as the contact owner.
            if( contactOwner == std::numeric_limits<int>::max() ) {
               for( std::vector<int>::iterator itA( bodyHoldersA.begin() ), itB( bodyHoldersB.begin() ); itA != bodyHoldersA.end() && itB != bodyHoldersB.end(); ( *itA < *itB ? ++itA : ++itB ) ) {
                  if( *itA == *itB ) {
                     // Smallest element in intersection found.
                     contactOwner = *itA;
                     break;
                  }
               }
            }

            pe_INTERNAL_ASSERT( contactOwner != std::numeric_limits<int>::max(), "No contact owner could be found." );

            // Resolve contacts between local and remote object if and only if the contact point is owned by us.
            if( myRank != contactOwner ) {
               pe_LOG_DEBUG_SECTION( log ) {
                  if( b1->isRemote() && b2->isRemote() )
                     log << "Rejecting remote-remote contact " << *c << " since contact witness with process rank " << contactOwner << " will treat it.\n";
                  else
                     log << "Rejecting remote-local contact " << *c << " since contact witness with process rank " << contactOwner << " will treat it.\n";
               }
               continue;
            }
#endif
         }
      }

      contactsMask_[i] = true;
      ++numContactsMasked;
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseContactFiltering_.end();
      memCollisionResponseContactFiltering_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif

      timeCollisionResponseContactCaching_.start();
      memCollisionResponseContactCaching_.start();
   }

   // Cache contact properties
   body1_.resize( numContactsMasked );
   body2_.resize( numContactsMasked );
   r1_.resize( numContactsMasked );
   r2_.resize( numContactsMasked );
   n_.resize( numContactsMasked );
   t_.resize( numContactsMasked );
   o_.resize( numContactsMasked );
   dist_.resize( numContactsMasked );
   mu_.resize( numContactsMasked );
   diag_nto_.resize( numContactsMasked );
   diag_nto_inv_.resize( numContactsMasked );
   diag_to_inv_.resize( numContactsMasked );
   diag_n_inv_.resize( numContactsMasked );
   p_.resize( numContactsMasked );

   {
      maximumPenetration_ = 0;
      numContacts_ = numContacts;

      size_t j = 0;
      for( size_t i = 0; i < numContacts; ++i ) {
         if( !contactsMask_[i] )
            continue;

         const ContactID c( contacts[i] );
         BodyID b1( c->getBody1() );
         BodyID b2( c->getBody2() );

         // the contact position is "in the middle"; r1 and r2 point to the same point but relative to body 1 and 2
         body1_[j]    = b1;
         body2_[j]    = b2;
         r1_[j]       = c->getPosition() - b1->getPosition();
         r2_[j]       = c->getPosition() - b2->getPosition();
         n_[j]        = c->getNormal();   // points from body 2 towards body 1 and is normalized
         // construct vector perpendicular to the normal (cross product with cardinal basis vector where the 1 component is where the other vector has its component of smallest magnitude)
         if( std::fabs( n_[j][0] ) < std::fabs( n_[j][1] ) ) {
            if( std::fabs( n_[j][0] ) < std::fabs( n_[j][2] ) )
               t_[j] = Vec3( 0, n_[j][2], -n_[j][1] );   // = n x (1, 0, 0)^T
            else
               t_[j] = Vec3( n_[j][1], -n_[j][0], 0 );   // = n x (0, 0, 1)^T
         }
         else {
            if( std::fabs( n_[j][1] ) < std::fabs( n_[j][2] ) )
               t_[j] = Vec3( -n_[j][2], 0, n_[j][0] );   // = n x (0, 1, 0)^T
            else
               t_[j] = Vec3( n_[j][1], -n_[j][0], 0 );   // = n x (0, 0, 1)^T
         }
         t_[j].normalize();
         o_[j]        = n_[j] % t_[j];
         Mat3 contactframe( n_[j], t_[j], o_[j] );

         dist_[j]  = c->getDistance();
         // If the distance is negative then penetration is present. This is an error and should be corrected. Correcting the whole error is not recommended since due to the linearization the errors cannot completely fixed anyway and the error reduction will introduce artificial restitution. However, if the distance is positive then it is not about error correction but the distance that can still be overcome without penetration and thus no error correction parameter should be applied.
         if( dist_[j] < 0 ) {
            maximumPenetration_ = std::max( maximumPenetration_, -dist_[j] );
            dist_[j] *= erp_;
         }

         mu_[j]       = c->getFriction();

         Mat3 diag    = -( r1_[j] % b1->getInvInertia() % r1_[j] + r2_[j] % b2->getInvInertia() % r2_[j] );
         diag[0]     += b1->getInvMass() + b2->getInvMass();
         diag[4]     += b1->getInvMass() + b2->getInvMass();
         diag[8]     += b1->getInvMass() + b2->getInvMass();
         diag         = trans( contactframe ) * diag * contactframe;

         // Diagonal block is know to be positive-definite and thus inverse always exists.
         diag_nto_[j] = diag;
         diag_nto_inv_[j]  = inv( diag );
         diag_n_inv_[j] = inv( diag[0] );
         diag_to_inv_[j] = inv( Mat2( diag[4], diag[5], diag[7], diag[8] ) );

         p_[j] = Vec3();

         ++j;
      }
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseContactCaching_.end();
      memCollisionResponseContactCaching_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif

      timeCollisionResponseBodyCaching_.start();
      memCollisionResponseBodyCaching_.start();
   }

   // Cache body properties (and time integrate v and w to the end of the time step by applying external forces, torques and accelerations)
   size_t numBodies( bodystorage_.size() + bodystorageShadowCopies_.size() );
   v_.resize( numBodies );
   w_.resize( numBodies );
   dv_.resize( numBodies );
   dw_.resize( numBodies );

   {
      size_t j = 0;
      for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body, ++j ) {
         body->wake(); // BUGFIX: Force awaking of all bodies!
         body->index_ = j;
         pe_INTERNAL_ASSERT( !body->isFixed() || ( isDefault( body->getInvMass() ) && isDefault( body->getInvInertia() ) ), "fatal" );

         initializeVelocityCorrections( *body, dv_[j], dw_[j], dt );

         if( body->awake_ && !body->isFixed() ) {
            v_[j] = body->getLinearVel() + Settings::gravity() * dt;
            w_[j] = body->getAngularVel() + dt * ( body->getInvInertia() * ( ( body->getInertia() * body->getAngularVel() ) % body->getAngularVel() ) );
         }
         else {
            v_[j] = body->getLinearVel();
            w_[j] = body->getAngularVel();
         }
      }

      for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body, ++j ) {
         body->wake(); // BUGFIX: Force awaking of all bodies!
         body->index_ = j;
         pe_INTERNAL_ASSERT( !body->isFixed() || ( isDefault( body->getInvMass() ) && isDefault( body->getInvInertia() ) ), "fatal" );

         initializeVelocityCorrections( *body, dv_[j], dw_[j], dt );

         // Velocities of shadow copies will be initialized by velocity synchronization.
#ifndef NDEBUG
         v_[j] = std::numeric_limits<real>::quiet_NaN();
         w_[j] = std::numeric_limits<real>::quiet_NaN();
#endif
      }

      const real rp = relaxationParam_;
      relaxationParam_ = real(1); // must be set to 1.0 such that dv and dw caused by external forces and torques are not falsely altered
      synchronizeVelocities();
      relaxationParam_ = rp;
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseBodyCaching_.end();
      memCollisionResponseBodyCaching_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif
   }

   // Iterate relaxation a constant number of times
   for( size_t it = 0; it < maxIterations_; ++it ) {
      //pe_LOG_WARNING_SECTION( log ) {
      //   log << "Iteration #" << it << "\n";
      //}
      real delta_max( 0 );

      iteration_ = it;

      switch( relaxationModel_ ) {
         case InelasticFrictionlessContact:
            delta_max = relaxInelasticFrictionlessContacts( dtinv );
            break;

         case ApproximateInelasticCoulombContactByDecoupling:
            delta_max = relaxApproximateInelasticCoulombContactsByDecoupling( dtinv );
            break;

         case ApproximateInelasticCoulombContactByOrthogonalProjections:
            delta_max = relaxInelasticCoulombContactsByOrthogonalProjections( dtinv, true );
            break;

         case InelasticCoulombContactByDecoupling:
            delta_max = relaxInelasticCoulombContactsByDecoupling( dtinv );
            break;

         case InelasticCoulombContactByOrthogonalProjections:
            delta_max = relaxInelasticCoulombContactsByOrthogonalProjections( dtinv, false );
            break;

         case InelasticGeneralizedMaximumDissipationContact:
            delta_max = relaxInelasticGeneralizedMaximumDissipationContacts( dtinv );
            break;

         default:
            throw std::runtime_error( "Unsupported relaxation model." );
      }

      synchronizeVelocities();

      // Compute maximum impulse variation.
      // TODO:
      // - velocity variation would be better.
      // - do not reduce in every iteration
#if 0
#if HAVE_MPI
      if( MPISettings::size() > 1 )
         MPI_Allreduce( MPI_IN_PLACE, &delta_max, 1, MPITrait<real>::getType(), MPI_MAX, MPISettings::comm() );
#endif
      if( delta_max < abortThreshold_ ) {
         ++it;
         break;
      }
#else
      UNUSED( delta_max );
#endif
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseIntegration_.start();
      memCollisionResponseIntegration_.start();
   }

   // Apply cached body properties (velocities) and time integrate positions
   {
      size_t j = 0;
      for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body, ++j ) {
         if( body->isGlobal() && body->isFixed() ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Integrating position of fixed global body " << *body << " with velocity " << v_[j] << "\n";
            }

            // WARNING: Even though dv_[j] and dw_[j] _should_ be exactly 0 at all times for global
            // bodies with infinite mass/inertia, this is not the case if the simulation breaks.
            // Infinite contact impulses can cause velocity corrections, which are not a number. If
            // added to the velocities of these global bodies their positions on different
            // processes can get out of sync (some of them can get NaNs). To prevent these NaNs and
            // thus out of sync global bodies, velocity corrections for global bodies of infinite
            // mass/inertia are silently ignored.

            integratePositions( *body, v_[j], w_[j], dt );
         }
         else if( body->isGlobal() ) {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Integrating position of non-fixed global body " << *body << " with velocity " << v_[j] << "\n";
            }

            integratePositions( *body, v_[j] + dv_[j], w_[j] + dw_[j], dt );
         }
         else {
            pe_LOG_DEBUG_SECTION( log ) {
               log << "Integrating position of local body " << *body << " with velocity " << v_[j] << "\n";
            }

            integratePositions( *body, v_[j] + dv_[j], w_[j] + dw_[j], dt );
         }
         pe_LOG_DETAIL_SECTION( log ) {
            log << "Result:\n" << *body << "\n";
         }
      }

      // Also apply time integration to shadow copies to skip the communication of the integrated positions (no more shadow copy updates)
      for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body, ++j ) {
         pe_LOG_DEBUG_SECTION( log ) {
            log << "Integrating position of shadow copy " << *body << " with velocity " << v_[j] << "\n";
         }
         integratePositions( *body, v_[j] + dv_[j], w_[j] + dw_[j], dt );
         pe_LOG_DETAIL_SECTION( log ) {
            log << "Result:\n" << *body << "\n";
         }
      }

      // NOTE: We might still need shadow copy updates if the user sets velocities or positions. Thus we might have to split up synchronize() again. It doesn't break anything if we still communicate the shadow copy updates though.
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseIntegration_.end();
      memCollisionResponseIntegration_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif

      timeCollisionResponse_.end();
      memCollisionResponse_.stop();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Relaxes all contacts once. The contact model is for inelastic unilateral contacts without friction.
 *
 * \return The largest variation of contact impulses in the L-infinity norm.
 *
 * This function is to be called from resolveContacts(). Separating contacts are preferred over
 * persisting solutions if valid.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
real CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::relaxInelasticFrictionlessContacts( real dtinv )
{
   real delta_max( 0 );
   size_t numContactsMasked( p_.size() );

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.start();
      memCollisionResponseSolving_.start();
   }

   // Relax contacts
   for( size_t i = 0; i < numContactsMasked; ++i ) {
      // Remove velocity corrections of this contact's reaction.
      dv_[body1_[i]->index_] -= body1_[i]->getInvMass() * p_[i];
      dw_[body1_[i]->index_] -= body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
      dv_[body2_[i]->index_] += body2_[i]->getInvMass() * p_[i];
      dw_[body2_[i]->index_] += body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );

      // Calculate the relative contact velocity in the global world frame (if no contact reaction is present at contact i)
      Vec3 gdot    ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );

      // Change from the global world frame to the contact frame
      Mat3 contactframe( n_[i], t_[i], o_[i] );
      Vec3 gdot_nto( trans( contactframe ) * gdot );

      // The constraint in normal direction is actually a positional constraint but instead of g_n we use g_n/dt equivalently and call it gdot_n
      gdot_nto[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;

      if( gdot_nto[0] >= 0 ) {
         // Contact is separating if no contact reaction is present at contact i.

         delta_max = std::max( delta_max, std::max( std::abs( p_[i][0] ), std::max( std::abs( p_[i][1] ), std::abs( p_[i][2] ) ) ) );
         p_[i] = Vec3();

         // No need to apply zero impulse.
      }
      else {
         // Contact is persisting.

         // Calculate the impulse necessary for a static contact expressed as components in the contact frame.
         Vec3 p_wf( n_[i] * ( -diag_n_inv_[i] * gdot_nto[0] ) );
         Vec3 dp( p_[i] - p_wf );
         delta_max = std::max( delta_max, std::max( std::abs( dp[0] ), std::max( std::abs( dp[1] ), std::abs( dp[2] ) ) ) );

         p_[i] = p_wf;

         // Apply impulse right away.
         dv_[body1_[i]->index_] += body1_[i]->getInvMass() * p_[i];
         dw_[body1_[i]->index_] += body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
         dv_[body2_[i]->index_] -= body2_[i]->getInvMass() * p_[i];
         dw_[body2_[i]->index_] -= body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );
      }
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.end();
      memCollisionResponseSolving_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif
   }

   return delta_max;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Relaxes all contacts once. The contact model is for inelastic unilateral contacts with approximate Coulomb friction.
 *
 * \return The largest variation of contact impulses in the L-infinity norm.
 *
 * This function is to be called from resolveContacts(). Separating contacts are preferred over
 * other solutions if valid. Static solutions are preferred over dynamic solutions. Dynamic
 * solutions are computed by decoupling the normal from the frictional components. That is
 * for a dynamic contact the normal component is relaxed first followed by the frictional
 * components. The determination of the frictional components does not perform any subiterations
 * and guarantees that the friction partially opposes slip.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
real CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::relaxApproximateInelasticCoulombContactsByDecoupling( real dtinv )
{
   real delta_max( 0 );
   size_t numContactsMasked( p_.size() );

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.start();
      memCollisionResponseSolving_.start();
   }

   // Relax contacts
   for( size_t i = 0; i < numContactsMasked; ++i ) {
      // Remove velocity corrections of this contact's reaction.
      dv_[body1_[i]->index_] -= body1_[i]->getInvMass() * p_[i];
      dw_[body1_[i]->index_] -= body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
      dv_[body2_[i]->index_] += body2_[i]->getInvMass() * p_[i];
      dw_[body2_[i]->index_] += body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );

      // Calculate the relative contact velocity in the global world frame (if no contact reaction is present at contact i)
      Vec3 gdot    ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );

      // Change from the global world frame to the contact frame
      Mat3 contactframe( n_[i], t_[i], o_[i] );
      Vec3 gdot_nto( trans( contactframe ) * gdot );

      //real gdot_n  ( trans( n_[i] ) * gdot );  // The component of gdot along the contact normal n
      //Vec3 gdot_t  ( gdot - gdot_n * n_[i] );  // The components of gdot tangential to the contact normal n
      //real g_n     ( gdot_n * dt /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] );  // The gap in normal direction

      // The constraint in normal direction is actually a positional constraint but instead of g_n we use g_n/dt equivalently and call it gdot_n
      gdot_nto[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;

      if( gdot_nto[0] >= 0 ) {
         // Contact is separating if no contact reaction is present at contact i.

         delta_max = std::max( delta_max, std::max( std::abs( p_[i][0] ), std::max( std::abs( p_[i][1] ), std::abs( p_[i][2] ) ) ) );
         p_[i] = Vec3();

         // No need to apply zero impulse.
      }
      else {
         // Contact is persisting (either static or dynamic).

         // Calculate the impulse necessary for a static contact expressed as components in the contact frame.
         Vec3 p_cf( -( diag_nto_inv_[i] * gdot_nto ) );

         // Can p_cf[0] be negative even though -gdot_nto[0] > 0? Yes! Try:
         // A = [0.5 -0.1 +0.1; -0.1 0.5 -0.1; +0.1 -0.1 1];
         // b = [0.01 -1 -1]';
         // A\b    \approx [-0.19 -2.28 -1.21]'
         // eig(A) \approx [ 0.40  0.56  1.04]'

         real flimit( mu_[i] * p_cf[0] );
         real fsq( p_cf[1] * p_cf[1] + p_cf[2] * p_cf[2] );
         if( fsq > flimit * flimit || p_cf[0] < 0 ) {
            // Contact cannot be static so it must be dynamic.
            // => Complementarity condition on normal reaction now turns into an equation since we know that the normal reaction is definitely not zero.

            // For simplicity we change to a simpler relaxation scheme here:
            // 1. Relax normal reaction with the tangential components equal to the previous values
            // 2. Relax tangential components with the newly relaxed normal reaction
            // Note: The better approach would be to solve the true 3x3 block problem!
            // Warning: Simply projecting the frictional components is wrong since then the normal action is no longer 0 and simulations break.

            // Add the action of the frictional reactions from the last iteration to the relative contact velocity in normal direction so we can relax it separately.
            // TODO This can be simplified:
            //p_cf = trans( contactframe ) * p_[i];
            //p_cf[0] = 0;
            //p_[i] = contactframe * p_cf;
            Vec3 p_tmp = ( trans( t_[i] ) * p_[i] ) * t_[i] + ( trans( o_[i] ) * p_[i] ) * o_[i];

            //      |<-- This should vanish below since p_cf[0] = 0          -->|
            //gdot += ( body1_[i]->getInvMass() + body2_[i]->getInvMass() ) * p_tmp + ( body1_[i]->getInvInertia() * ( r1_[i] % p_tmp] ) ) % r1_[i] + ( body2_[i]->getInvInertia() * ( r2_[i] % p_tmp ) ) % r2_[i] /* + diag_[i] * p */;
            //real gdot_n = trans( n_[i] ) * gdot;
            //gdot_n += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;

            real gdot_n = gdot_nto[0] + trans( n_[i] ) * ( ( body1_[i]->getInvInertia() * ( r1_[i] % p_tmp ) ) % r1_[i] + ( body2_[i]->getInvInertia() * ( r2_[i] % p_tmp ) ) % r2_[i] /* + diag_[i] * p */ );
            p_cf[0] = -( diag_n_inv_[i] * gdot_n );

            // We cannot be sure that gdot_n <= 0 here and thus p_cf[0] >= 0 since we just modified it with the old values of the tangential reactions! => Project
            p_cf[0] = std::max( real( 0 ), p_cf[0] );

            // Now add the action of the normal reaction to the relative contact velocity in the tangential directions so we can relax the frictional components separately.
            p_tmp = n_[i] * p_cf[0];
            Vec3 gdot2 = gdot + ( body1_[i]->getInvInertia() * ( r1_[i] % p_tmp ) ) % r1_[i] + ( body2_[i]->getInvInertia() * ( r2_[i] % p_tmp ) ) % r2_[i];
            Vec2 gdot_to;
            gdot_to[0] = trans( t_[i] ) * gdot2;
            gdot_to[1] = trans( o_[i] ) * gdot2;

            Vec2 ret = -( diag_to_inv_[i] * gdot_to );
            p_cf[1] = ret[0];
            p_cf[2] = ret[1];

            flimit = mu_[i] * p_cf[0];
            fsq = p_cf[1] * p_cf[1] + p_cf[2] * p_cf[2];
            if( fsq > flimit * flimit ) {
               const real f( flimit / std::sqrt( fsq ) );
               p_cf[1] *= f;
               p_cf[2] *= f;
            }
         }
         else {
            // Contact is static.
         }
         Vec3 p_wf( contactframe * p_cf );
         Vec3 dp( p_[i] - p_wf );
         delta_max = std::max( delta_max, std::max( std::abs( dp[0] ), std::max( std::abs( dp[1] ), std::abs( dp[2] ) ) ) );

         p_[i] = p_wf;

         // Apply impulse right away
         dv_[body1_[i]->index_] += body1_[i]->getInvMass() * p_[i];
         dw_[body1_[i]->index_] += body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
         dv_[body2_[i]->index_] -= body2_[i]->getInvMass() * p_[i];
         dw_[body2_[i]->index_] -= body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );
      }

#if 0
      Vec3 gdot2   ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );
      Vec3 gdot_nto2( trans( contactframe ) * gdot2 );
      pe_LOG_INFO_SECTION( log ) {
         log << "gdot_n2 = " << gdot_nto2[0] << "\n";
         log << "gdot_t2 = " << gdot_nto2[1] << "\n";
         log << "gdot_o2 = " << gdot_nto2[2] << "\n";
      }
      gdot_nto2[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;
      pe_LOG_INFO_SECTION( log ) {
         log << "gdot_n2' = " << gdot_nto2[0] << "\n";
      }
#endif

      /*
       * compare DEM time-step with NSCD iteration:
       * - projections are the same
       * - velocities are the same if we use an explicit Euler discretization for the velocity time integration
       *
      f_cf[0] = -stiffness * dist_ - damping_n * gdot_n = -[(stiffness * dt) * dist_ * dtinv + damping_n * gdot_n] = -foo * (gdot_n + dist_ * dtinv) where foo = stiffness * dt = damping_n;
      f_cf[1] = -damping_t * gdot_t                     = -damping_t * gdot_t;
      f_cf[2] = -damping_t * gdot_o                     = -damping_t * gdot_o;

      or: f_cf = -diag(foo, damping_t, damping_t) * gdot_nto   (since gdot_nto[0] is modified)
      vs. f_cf = -diaginv * gdot_nto in NSCD iteration

      => The NSCD iteration is more or less a DEM time step where we choose the stiffness and damping parameters such that penetration is non-existent after a time step and contacts are truly static (tangential rel. vel. is zero) unless the friction force hits its limit

      f_cf[0] = std::max( 0, f_cf[0] );

      flimit = mu_ * f_cf[0];
      fsq = f_cf[1] * f_cf[1] + f_cf[2] * f_cf[2]
      if( fsq > flimit * flimit ) {
         f = flimit / sqrt( fsq );
         f_cf[1] *= f;
         f_cf[2] *= f;
      }

      f_wf = contactframe * f_cf;

      b1->addForceAtPos(  f_wf, gpos );
      b2->addForceAtPos( -f_wf, gpos );
      */

   }

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.end();
      memCollisionResponseSolving_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif
   }

   return delta_max;
}
//*************************************************************************************************


//*************************************************************************************************

/*!\brief Relaxes all contacts once. The contact model is for inelastic unilateral contacts with Coulomb friction.
 *
 * \return The largest variation of contact impulses in the L-infinity norm.
 *
 * This function is to be called from resolveContacts(). Separating contacts are preferred over
 * other solutions if valid. Static solutions are preferred over dynamic solutions. Dynamic
 * solutions are computed by decoupling the normal from the frictional components. That is
 * for a dynamic contact the normal component is relaxed first followed by the frictional
 * components. How much the frictional components directly oppose slip as required by the Coulomb
 * friction model depends on the number of subiterations performed. If no subiterations are
 * performed the friction is guaranteed to be at least partially dissipative.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
real CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::relaxInelasticCoulombContactsByDecoupling( real dtinv )
{
   real delta_max( 0 );
   size_t numContactsMasked( p_.size() );

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.start();
      memCollisionResponseSolving_.start();
   }

   // Relax contacts
   for( size_t i = 0; i < numContactsMasked; ++i ) {
      // Remove velocity corrections of this contact's reaction.
      dv_[body1_[i]->index_] -= body1_[i]->getInvMass() * p_[i];
      dw_[body1_[i]->index_] -= body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
      dv_[body2_[i]->index_] += body2_[i]->getInvMass() * p_[i];
      dw_[body2_[i]->index_] += body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );

      // Calculate the relative contact velocity in the global world frame (if no contact reaction is present at contact i)
      Vec3 gdot    ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );

      // Change from the global world frame to the contact frame
      Mat3 contactframe( n_[i], t_[i], o_[i] );
      Vec3 gdot_nto( trans( contactframe ) * gdot );

      //real gdot_n  ( trans( n_[i] ) * gdot );  // The component of gdot along the contact normal n
      //Vec3 gdot_t  ( gdot - gdot_n * n_[i] );  // The components of gdot tangential to the contact normal n
      //real g_n     ( gdot_n * dt /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] );  // The gap in normal direction

      // The constraint in normal direction is actually a positional constraint but instead of g_n we use g_n/dt equivalently and call it gdot_n
      gdot_nto[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;
         
      //pe_LOG_WARNING_SECTION( log ) {
      //   log << "Contact #" << i << " is\nA = \n" << diag_nto_[i] << "\nb = \n" << gdot_nto << "\nmu = " << mu_[i] << "\n";
      //}

      if( gdot_nto[0] >= 0 ) {
         // Contact is separating if no contact reaction is present at contact i.

         delta_max = std::max( delta_max, std::max( std::abs( p_[i][0] ), std::max( std::abs( p_[i][1] ), std::abs( p_[i][2] ) ) ) );
         p_[i] = Vec3();
         //pe_LOG_WARNING_SECTION( log ) {
         //   log << "Contact #" << i << " is separating.\n";
         //}

         // No need to apply zero impulse.
      }
      else {
         // Contact is persisting (either static or dynamic).

         // Calculate the impulse necessary for a static contact expressed as components in the contact frame.
         Vec3 p_cf( -( diag_nto_inv_[i] * gdot_nto ) );

         // Can p_cf[0] be negative even though -gdot_nto[0] > 0? Yes! Try:
         // A = [0.5 -0.1 +0.1; -0.1 0.5 -0.1; +0.1 -0.1 1];
         // b = [0.01 -1 -1]';
         // A\b    \approx [-0.19 -2.28 -1.21]'
         // eig(A) \approx [ 0.40  0.56  1.04]'

         real flimit( mu_[i] * p_cf[0] );
         real fsq( p_cf[1] * p_cf[1] + p_cf[2] * p_cf[2] );
         if( fsq > flimit * flimit || p_cf[0] < 0 ) {
            // Contact cannot be static so it must be dynamic.
            // => Complementarity condition on normal reaction now turns into an equation since we know that the normal reaction is definitely not zero.

            for (int j = 0; j < 20; ++j) {
               // For simplicity we change to a simpler relaxation scheme here:
               // 1. Relax normal reaction with the tangential components equal to the previous values
               // 2. Relax tangential components with the newly relaxed normal reaction
               // Note: The better approach would be to solve the true 3x3 block problem!
               // Warning: Simply projecting the frictional components is wrong since then the normal action is no longer 0 and simulations break.

               Vec3 gdotCorrected;
               real gdotCorrected_n;
               Vec2 gdotCorrected_to;

               // Calculate the relative contact velocity in the global world frame (if no normal contact reaction is present at contact i)
               p_cf[0] = 0;
               //                       |<- p_cf is orthogonal to the normal and drops out in next line ->|
               gdotCorrected   = gdot + /* ( body1_[i]->getInvMass() + body2_[i]->getInvMass() ) * p_cf  */ + ( body1_[i]->getInvInertia() * ( r1_[i] % ( t_[i] * p_cf[1] + o_[i] * p_cf[2] ) ) ) % r1_[i] + ( body2_[i]->getInvInertia() * ( r2_[i] % ( t_[i] * p_cf[1] + o_[i] * p_cf[2] ) ) ) % r2_[i];
               gdotCorrected_n = trans( n_[i] ) * gdotCorrected + dist_[i] * dtinv;

               // Relax normal component.
               p_cf[0] = std::max( real( 0 ), -( diag_n_inv_[i] * gdotCorrected_n ) );

               // Calculate the relative contact velocity in the global world frame (if no frictional contact reaction is present at contact i)
               p_cf[1] = p_cf[2] = real( 0 );
               //                       |<- p_cf is orthogonal to the tangential plane and drops out   ->|
               gdotCorrected   = gdot + /* ( body1_[i]->getInvMass() + body2_[i]->getInvMass() ) * p_cf */ + ( body1_[i]->getInvInertia() * ( r1_[i] % ( n_[i] * p_cf[0] ) ) ) % r1_[i] + ( body2_[i]->getInvInertia() * ( r2_[i] % ( n_[i] * p_cf[0] ) ) ) % r2_[i];
               gdotCorrected_to[0] = trans( t_[i] ) * gdotCorrected;
               gdotCorrected_to[1] = trans( o_[i] ) * gdotCorrected;

               // Relax frictional components.
               Vec2 ret = -( diag_to_inv_[i] * gdotCorrected_to );
               p_cf[1] = ret[0];
               p_cf[2] = ret[1];

               flimit = mu_[i] * p_cf[0];
               fsq = p_cf[1] * p_cf[1] + p_cf[2] * p_cf[2];
               if( fsq > flimit * flimit ) {
                  // 3.2.1 Decoupling
                  // \tilde{x}^0 = p_cf[1..2]

                  // Determine \tilde{A}
                  Mat2 diag_to( diag_nto_[i](1, 1), diag_nto_[i](1, 2), diag_nto_[i](2, 1), diag_nto_[i](2, 2) );

                  const real f( flimit / std::sqrt( fsq ) );
                  //p_cf[1] *= f;
                  //p_cf[2] *= f;

                  // Determine search interval for Golden Section Search
                  const real phi( real(0.5) * ( real(1) + sqrt( real( 5 ) ) ) );
                  real shift( std::atan2( -p_cf[2], p_cf[1] ) );
                  real acos_f( std::acos( f ) );

                  //pe_LOG_WARNING_SECTION( log ) {
                  //   log << acos_f << " " << shift << "\n";
                  //}

                  real alpha_left( -acos_f - shift );
                  //Vec2 x_left( flimit * std::cos( alpha_left ), flimit * std::sin( alpha_left ) );
                  //real f_left( 0.5 * trans( x_left ) * ( diag_to * x_left ) - trans( x_left ) * ( -gdot_to ) );

                  real alpha_right( acos_f - shift );
                  //Vec2 x_right( flimit * std::cos( alpha_right ), flimit * std::sin( alpha_right ) );
                  //real f_right( 0.5 * trans( x_right ) * ( diag_to * x_right ) - trans( x_right ) * ( -gdot_to ) );

                  real alpha_mid( ( alpha_right + alpha_left * phi ) / ( 1 + phi ) );
                  Vec2 x_mid( flimit * std::cos( alpha_mid ), flimit * std::sin( alpha_mid ) );
                  real f_mid( real(0.5) * trans( x_mid ) * ( diag_to * x_mid ) - trans( x_mid ) * ( -gdotCorrected_to ) );

                  bool leftlarger = false;
                  for( size_t k = 0; k < maxSubIterations_; ++k ) {
                     real alpha_next( alpha_left + ( alpha_right - alpha_mid ) );
                     Vec2 x_next( flimit * std::cos( alpha_next ), flimit * std::sin( alpha_next ) );
                     real f_next( real(0.5) * trans( x_next ) * ( diag_to * x_next ) - trans( x_next ) * ( -gdotCorrected_to ) );
                     //pe_LOG_WARNING_SECTION( log ) {
                     //   log << "[(" << alpha_left << ", ?); (" << alpha_mid << ", " << f_mid << "); (" << alpha_right << ", ?)] <- (" << alpha_next << ", " << f_next << ")\n";
                     //}
                     //pe_LOG_WARNING_SECTION( log ) {
                     //   log << "left: " << alpha_mid - alpha_left << "  right: " << alpha_right - alpha_mid << "  ll: " << leftlarger << "\n";
                     //}
                     //pe_INTERNAL_ASSERT(leftlarger ? (alpha_mid - alpha_left > alpha_right - alpha_mid) : (alpha_mid - alpha_left < alpha_right - alpha_mid), "ll inconsistent!" );

                     if (leftlarger) {
                        // left interval larger
                        if( f_next < f_mid ) {
                           alpha_right = alpha_mid;
                           alpha_mid   = alpha_next;
                           x_mid       = x_next;
                           f_mid       = f_next;
                           leftlarger = true;
                        }
                        else {
                           alpha_left  = alpha_next;
                           leftlarger = false;
                        }
                     }
                     else {
                        // right interval larger
                        if( f_next < f_mid ) {
                           alpha_left = alpha_mid;
                           alpha_mid  = alpha_next;
                           x_mid      = x_next;
                           f_mid      = f_next;
                           leftlarger = false;
                        }
                        else {
                           alpha_right = alpha_next;
                           leftlarger = true;
                        }
                     }
                  }
                  //pe_LOG_WARNING_SECTION( log ) {
                  //   log << "dalpha = " << alpha_right - alpha_left << "\n";
                  //}

                  p_cf[1] = x_mid[0];
                  p_cf[2] = x_mid[1];
               }
            }
            //pe_LOG_WARNING_SECTION( log ) {
            //   log << "Contact #" << i << " is dynamic.\n";
            //}
         }
         else {
            // Contact is static.
            //pe_LOG_WARNING_SECTION( log ) {
            //   log << "Contact #" << i << " is static.\n";
            //}
         }

         //pe_LOG_WARNING_SECTION( log ) {
         //   log << "Contact reaction in contact frame: " << p_cf << "\n" << diag_nto_[i]*p_cf + gdot_nto << "\n";
         //}
         Vec3 p_wf( contactframe * p_cf );
         Vec3 dp( p_[i] - p_wf );
         delta_max = std::max( delta_max, std::max( std::abs( dp[0] ), std::max( std::abs( dp[1] ), std::abs( dp[2] ) ) ) );

         p_[i] = p_wf;

         // Apply impulse right away
         dv_[body1_[i]->index_] += body1_[i]->getInvMass() * p_[i];
         dw_[body1_[i]->index_] += body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
         dv_[body2_[i]->index_] -= body2_[i]->getInvMass() * p_[i];
         dw_[body2_[i]->index_] -= body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );
      }

#if 0
      Vec3 gdot2   ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );
      Vec3 gdot_nto2( trans( contactframe ) * gdot2 );
      pe_LOG_INFO_SECTION( log ) {
         log << "gdot_n2 = " << gdot_nto2[0] << "\n";
         log << "gdot_t2 = " << gdot_nto2[1] << "\n";
         log << "gdot_o2 = " << gdot_nto2[2] << "\n";
      }
      gdot_nto2[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;
      pe_LOG_INFO_SECTION( log ) {
         log << "gdot_n2' = " << gdot_nto2[0] << "\n";
      }
#endif

      /*
       * compare DEM time-step with NSCD iteration:
       * - projections are the same
       * - velocities are the same if we use an explicit Euler discretization for the velocity time integration
       *
      f_cf[0] = -stiffness * dist_ - damping_n * gdot_n = -[(stiffness * dt) * dist_ * dtinv + damping_n * gdot_n] = -foo * (gdot_n + dist_ * dtinv) where foo = stiffness * dt = damping_n;
      f_cf[1] = -damping_t * gdot_t                     = -damping_t * gdot_t;
      f_cf[2] = -damping_t * gdot_o                     = -damping_t * gdot_o;

      or: f_cf = -diag(foo, damping_t, damping_t) * gdot_nto   (since gdot_nto[0] is modified)
      vs. f_cf = -diaginv * gdot_nto in NSCD iteration

      => The NSCD iteration is more or less a DEM time step where we choose the stiffness and damping parameters such that penetration is non-existent after a time step and contacts are truly static (tangential rel. vel. is zero) unless the friction force hits its limit

      f_cf[0] = std::max( 0, f_cf[0] );

      flimit = mu_ * f_cf[0];
      fsq = f_cf[1] * f_cf[1] + f_cf[2] * f_cf[2]
      if( fsq > flimit * flimit ) {
         f = flimit / sqrt( fsq );
         f_cf[1] *= f;
         f_cf[2] *= f;
      }

      f_wf = contactframe * f_cf;

      b1->addForceAtPos(  f_wf, gpos );
      b2->addForceAtPos( -f_wf, gpos );
      */
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.end();
      memCollisionResponseSolving_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif
   }

   return delta_max;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Relaxes all contacts once. The contact model is for inelastic unilateral contacts with Coulomb friction.
 *
 * \param dtinv The inverse of the current time step.
 * \param approximate Use the approximate model showing bouncing.
 * \return The largest variation of contact impulses in the L-infinity norm.
 *
 * This function is to be called from resolveContacts(). The iterative method to solve the contact
 * problem is e.g. described in the article "A matrix-free cone complementarity approach for
 * solving large-scale, nonsmooth, rigid body dynamics" by A. Tasora and M. Anitescu in Computer
 * Methods in Applied Mechanics and Engineering (Volume 200, Issues 5–8, 15 January 2011,
 * Pages 439-453). The contact model is purely quadratic and convergence should be good but depends
 * on a parameter. The one-contact problem has a unique solution. The frictional reactions
 * for a dynamic contact converge to those that directly oppose slip. However, the contact is
 * not perfectly inelastic for dynamic contacts but bounces. These vertical motions tend to
 * go to zero for smaller time steps and can be interpreted as exaggerated vertical motions
 * coming from micro asperities (see "Optimization-based simulation of nonsmooth rigid multibody
 * dynamics" by M. Anitescu in Mathematical Programming (Volume 105, Issue 1, January 2006, Pages
 * 113-143). These motions can be prevented by a small change in the iteration proposed in "The
 * bipotential method: a constructive approach to design the complete contact law with friction and
 * improved numerical algorithms" by G. De Saxce and Z-Q. Feng in Mathematical and Computer
 * Modelling (Volume 28, Issue 4, 1998, Pages 225-245). Which iteration is used is controlled with
 * the approximate parameter.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
real CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::relaxInelasticCoulombContactsByOrthogonalProjections( real dtinv, bool approximate )
{
   real delta_max( 0 );
   size_t numContactsMasked( p_.size() );

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.start();
      memCollisionResponseSolving_.start();
   }

   // Relax contacts
   for( size_t i = 0; i < numContactsMasked; ++i ) {
      // Remove velocity corrections of this contact's reaction.
      dv_[body1_[i]->index_] -= body1_[i]->getInvMass() * p_[i];
      dw_[body1_[i]->index_] -= body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
      dv_[body2_[i]->index_] += body2_[i]->getInvMass() * p_[i];
      dw_[body2_[i]->index_] += body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );

      // Calculate the relative contact velocity in the global world frame (if no contact reaction is present at contact i)
      Vec3 gdot    ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );

      // Change from the global world frame to the contact frame
      Mat3 contactframe( n_[i], t_[i], o_[i] );
      Vec3 gdot_nto( trans( contactframe ) * gdot );

      //real gdot_n  ( trans( n_[i] ) * gdot );  // The component of gdot along the contact normal n
      //Vec3 gdot_t  ( gdot - gdot_n * n_[i] );  // The components of gdot tangential to the contact normal n
      //real g_n     ( gdot_n * dt /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] );  // The gap in normal direction

      // The constraint in normal direction is actually a positional constraint but instead of g_n we use g_n/dt equivalently and call it gdot_n
      gdot_nto[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;

      const real w( 1 ); // w > 0
      Vec3 p_cf( trans( contactframe ) * p_[i] );
      if( approximate ) {
         // Calculate next iterate (Anitescu/Tasora).
         p_cf = p_cf - w * ( diag_nto_[i] * p_cf + gdot_nto );
      }
      else {
         // Calculate next iterate (De Saxce/Feng).
         Vec3 tmp( diag_nto_[i] * p_cf + gdot_nto );
         tmp[0] += std::sqrt( sq( tmp[1] ) + sq( tmp[2] ) ) * mu_[i];
         p_cf = p_cf - w * tmp;
      }

      // Project.
      real flimit( mu_[i] * p_cf[0] );
      real fsq( p_cf[1] * p_cf[1] + p_cf[2] * p_cf[2] );
      if( p_cf[0] > 0 && fsq < flimit * flimit ) {
         // Unconstrained minimum is in cone leading to a static contact and no projection
         // is necessary.
      }
      else if( p_cf[0] < 0 && fsq < p_cf[0] / sq( mu_[i] ) ) {
         // Unconstrained minimum is in dual cone leading to a separating contact where no contact
         // reaction is present (the unconstrained minimum is projected to the tip of the cone).

         p_cf = Vec3();
      }
      else {
         // The contact is dynamic.
         real f( std::sqrt( fsq ) );
         p_cf[0] = ( f * mu_[i] + p_cf[0] ) / ( sq( mu_[i] ) + 1 );
         real factor( mu_[i] * p_cf[0] / f );
         p_cf[1] *= factor;
         p_cf[2] *= factor;
      }

      Vec3 p_wf( contactframe * p_cf );
      Vec3 dp( p_[i] - p_wf );
      delta_max = std::max( delta_max, std::max( std::abs( dp[0] ), std::max( std::abs( dp[1] ), std::abs( dp[2] ) ) ) );

      p_[i] = p_wf;

      // Apply impulse right away
      dv_[body1_[i]->index_] += body1_[i]->getInvMass() * p_[i];
      dw_[body1_[i]->index_] += body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
      dv_[body2_[i]->index_] -= body2_[i]->getInvMass() * p_[i];
      dw_[body2_[i]->index_] -= body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.end();
      memCollisionResponseSolving_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif
   }

   return delta_max;
}
//*************************************************************************************************


//*************************************************************************************************

/*!\brief Relaxes all contacts once. The contact model is for inelastic unilateral contacts with the generalized maximum dissipation principle for friction.
 *
 * \return The largest variation of contact impulses in the L-infinity norm.
 *
 * This function is to be called from resolveContacts(). Dynamic solutions are computed by
 * minimizing the kinetic energy along the intersection of the plane of maximum compression and
 * the friction cone.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
real CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::relaxInelasticGeneralizedMaximumDissipationContacts( real dtinv )
{
   real delta_max( 0 );
   size_t numContactsMasked( p_.size() );

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.start();
      memCollisionResponseSolving_.start();
   }

   // Relax contacts
   for( size_t i = 0; i < numContactsMasked; ++i ) {
      // Remove velocity corrections of this contact's reaction.
      dv_[body1_[i]->index_] -= body1_[i]->getInvMass() * p_[i];
      dw_[body1_[i]->index_] -= body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
      dv_[body2_[i]->index_] += body2_[i]->getInvMass() * p_[i];
      dw_[body2_[i]->index_] += body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );

      // Calculate the relative contact velocity in the global world frame (if no contact reaction is present at contact i)
      Vec3 gdot    ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );

      // Change from the global world frame to the contact frame
      Mat3 contactframe( n_[i], t_[i], o_[i] );
      Vec3 gdot_nto( trans( contactframe ) * gdot );

      // The constraint in normal direction is actually a positional constraint but instead of g_n we use g_n/dt equivalently and call it gdot_n
      gdot_nto[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;

      //pe_LOG_WARNING_SECTION( log ) {
      //   log << "Contact #" << i << " is\nA = \n" << diag_nto_[i] << "\nb = \n" << gdot_nto << "\nmu = " << mu_[i] << "\n";
      //}

      if( gdot_nto[0] >= 0 ) {
         // Contact is separating if no contact reaction is necessary without violating the penetration constraint.

         delta_max = std::max( delta_max, std::max( std::abs( p_[i][0] ), std::max( std::abs( p_[i][1] ), std::abs( p_[i][2] ) ) ) );
         p_[i] = Vec3();

         //pe_LOG_WARNING_SECTION( log ) {
         //   log << "Contact #" << i << " is separating.\n";
         //}

         // No need to apply zero impulse.
      }
      else {
         // Contact is persisting (either static or dynamic).

         // Calculate the impulse necessary for a static contact expressed as components in the contact frame.
         Vec3 p_cf( -( diag_nto_inv_[i] * gdot_nto ) );

         // Can p_cf[0] be negative even though -gdot_nto[0] > 0? Yes! Try:
         // A = [0.5 -0.1 +0.1; -0.1 0.5 -0.1; +0.1 -0.1 1];
         // b = [0.01 -1 -1]';
         // A\b    \approx [-0.19 -2.28 -1.21]'
         // eig(A) \approx [ 0.40  0.56  1.04]'

         real flimit( mu_[i] * p_cf[0] );
         real fsq( p_cf[1] * p_cf[1] + p_cf[2] * p_cf[2] );
         if( fsq > flimit * flimit || p_cf[0] < 0 ) {
            // Contact cannot be static so it must be dynamic.
            // => Complementarity condition on normal reaction now turns into an equation since we know that the normal reaction is definitely not zero.

            // \breve{x}^0 = p_cf[1..2]

            // Eliminate normal component from 3x3 system: diag_nto_[i]*p_cf + gdot_nto => \breve{A} \breve{x} - \breve{b}
            const real invA_nn( inv( diag_nto_[i](0, 0) ) );
            const real offdiag( diag_nto_[i](1, 2) - invA_nn * diag_nto_[i](0, 1) * diag_nto_[i](0, 2) );
            Mat2 A_breve( diag_nto_[i](1, 1) - invA_nn * sq( diag_nto_[i](0, 1) ), offdiag, offdiag, diag_nto_[i](2, 2) - invA_nn * sq( diag_nto_[i](0, 2) ) );
            Vec2 b_breve( -gdot_nto[1] + invA_nn * diag_nto_[i](0, 1) * gdot_nto[0], -gdot_nto[2] + invA_nn * diag_nto_[i](0, 2) * gdot_nto[0] );

            const real shiftI( std::atan2( -diag_nto_[i](0, 2), diag_nto_[i](0, 1) ) );
            const real shiftJ( std::atan2( -p_cf[2], p_cf[1] ) );
            const real a3( std::sqrt( sq( diag_nto_[i](0, 1) ) + sq( diag_nto_[i](0, 2) ) ) );
            const real fractionI( -diag_nto_[i](0, 0) / ( mu_[i] * a3 ) );
            const real fractionJ( std::min( invA_nn * mu_[i] * ( ( -gdot_nto[0] ) / std::sqrt( fsq ) - a3 * std::cos( shiftI - shiftJ ) ), real( 1 ) ) );

            // Search interval determination.
            real alpha_left, alpha_right;
            if( fractionJ < -1 ) {
               // J is complete
               const real angleI( std::acos( fractionI ) );
               alpha_left = -angleI - shiftI;
               alpha_right = +angleI - shiftI;
               if( alpha_left < 0 ) {
                  alpha_left += 2 * M_PI;
                  alpha_right += 2 * M_PI;
               }
            }
            else if( diag_nto_[i](0, 0) > mu_[i] * a3 ) {
               // I is complete
               const real angleJ( std::acos( fractionJ ) );
               alpha_left = -angleJ - shiftJ;
               alpha_right = +angleJ - shiftJ;
               if( alpha_left < 0 ) {
                  alpha_left += 2 * M_PI;
                  alpha_right += 2 * M_PI;
               }
            }
            else {
               // neither I nor J is complete
               const real angleJ( std::acos( fractionJ ) );
               real alpha1_left( -angleJ - shiftJ );
               real alpha1_right( +angleJ - shiftJ );
               if( alpha1_left < 0 ) {
                  alpha1_left += 2 * M_PI;
                  alpha1_right += 2 * M_PI;
               }
               const real angleI( std::acos( fractionI ) );
               real alpha2_left( -angleI - shiftI );
               real alpha2_right( +angleI - shiftI );
               if( alpha2_left < 0 ) {
                  alpha2_left += 2 * M_PI;
                  alpha2_right += 2 * M_PI;
               }

               // Swap intervals if second interval does not start right of the first interval.
               if( alpha1_left > alpha2_left ) {
                  std::swap( alpha1_left, alpha2_left );
                  std::swap( alpha1_right, alpha2_right );
               }

               if( alpha2_left > alpha1_right ) {
                  alpha2_right -= 2*M_PI;
                  if( alpha2_right > alpha1_right ) {
                     // [alpha1_left; alpha1_right] \subset [alpha2_left; alpha2_right]
                  }
                  else {
                     // [alpha2_left; alpha2_right] intersects the left end of [alpha1_left; alpha1_right]
                     alpha1_right = alpha2_right;
                  }
               }
               else {
                  alpha1_left = alpha2_left;
                  if( alpha2_right > alpha1_right ) {
                     // [alpha2_left; alpha2_right] intersects the right end of [alpha1_left; alpha1_right]
                  }
                  else {
                     // [alpha2_left; alpha2_right] \subset [alpha1_left; alpha1_right]
                     alpha1_right = alpha2_right;
                  }
               }

               alpha_left = alpha1_left;
               alpha_right = alpha1_right;
            }

            const real phi( real(0.5) * ( real(1) + sqrt( real( 5 ) ) ) );
            real alpha_mid( ( alpha_right + alpha_left * phi ) / ( 1 + phi ) );
            Vec2 x_mid;
            real f_mid;

            {
               real r_ub = mu_[i] * ( -gdot_nto[0] ) / ( diag_nto_[i](0, 0) + mu_[i] * a3 * std::cos( alpha_mid + shiftI ) );
               if( r_ub < 0 )
                  r_ub = inf;
               x_mid = Vec2( r_ub * std::cos( alpha_mid ), r_ub * std::sin( alpha_mid ) );
               f_mid = real(0.5) * trans( x_mid ) * ( A_breve * x_mid ) - trans( x_mid ) * b_breve;
            }

            bool leftlarger = false;
            for( size_t k = 0; k < maxSubIterations_; ++k ) {
               real alpha_next( alpha_left + ( alpha_right - alpha_mid ) );
               real r_ub = mu_[i] * ( -gdot_nto[0] ) / ( diag_nto_[i](0, 0) + mu_[i] * a3 * std::cos( alpha_next + shiftI ) );
               if( r_ub < 0 )
                  r_ub = inf;
               Vec2 x_next( r_ub * std::cos( alpha_next ), r_ub * std::sin( alpha_next ) );
               real f_next( real(0.5) * trans( x_next ) * ( A_breve * x_next ) - trans( x_next ) * b_breve );

               //pe_LOG_WARNING_SECTION( log ) {
               //   log << "[(" << alpha_left << ", ?); (" << alpha_mid << ", " << f_mid << "); (" << alpha_right << ", ?)] <- (" << alpha_next << ", " << f_next << ")\n";
               //}
               //pe_LOG_WARNING_SECTION( log ) {
               //   log << "left: " << alpha_mid - alpha_left << "  right: " << alpha_right - alpha_mid << "  ll: " << leftlarger << "\n";
               //}
               //pe_INTERNAL_ASSERT(leftlarger ? (alpha_mid - alpha_left > alpha_right - alpha_mid) : (alpha_mid - alpha_left < alpha_right - alpha_mid), "ll inconsistent!" );

               if (leftlarger) {
                  // left interval larger
                  if( f_next < f_mid ) {
                     alpha_right = alpha_mid;
                     alpha_mid   = alpha_next;
                     x_mid       = x_next;
                     f_mid       = f_next;
                     leftlarger = true;
                  }
                  else {
                     alpha_left  = alpha_next;
                     leftlarger = false;
                  }
               }
               else {
                  // right interval larger
                  if( f_next < f_mid ) {
                     alpha_left = alpha_mid;
                     alpha_mid  = alpha_next;
                     x_mid      = x_next;
                     f_mid      = f_next;
                     leftlarger = false;
                  }
                  else {
                     alpha_right = alpha_next;
                     leftlarger = true;
                  }
               }
            }
            //pe_LOG_WARNING_SECTION( log ) {
            //   log << "dalpha = " << alpha_right - alpha_left << "\n";
            //}
            {
               real alpha_init( std::atan2( p_cf[2], p_cf[1] ) );
               real r_ub = mu_[i] * ( -gdot_nto[0] ) / ( diag_nto_[i](0, 0) + mu_[i] * a3 * std::cos( alpha_init + shiftI ) );
               if( r_ub < 0 )
                  r_ub = inf;
               Vec2 x_init( r_ub * std::cos( alpha_init ), r_ub * std::sin( alpha_init ) );
               real f_init( real(0.5) * trans( x_init ) * ( A_breve * x_init ) - trans( x_init ) * b_breve );

               if( f_init < f_mid ) {
                  x_mid = x_init;
                  pe_LOG_DEBUG_SECTION( log ) {
                     log << "Replacing solution by primitive dissipative solution (" << f_init << " < " << f_mid << " at " << alpha_init << " vs. " << alpha_mid << ").\n";
                  }
               }
            }

            p_cf[0] = invA_nn * ( -gdot_nto[0] - diag_nto_[i](0, 1) * x_mid[0] - diag_nto_[i](0, 2) * x_mid[1] );
            p_cf[1] = x_mid[0];
            p_cf[2] = x_mid[1];
            //pe_LOG_WARNING_SECTION( log ) {
            //   log << "Contact #" << i << " is dynamic.\n";
            //}
         }
         else {
            // Contact is static.
            //pe_LOG_WARNING_SECTION( log ) {
            //   log << "Contact #" << i << " is static.\n";
            //}
         }
         Vec3 p_wf( contactframe * p_cf );
         Vec3 dp( p_[i] - p_wf );
         delta_max = std::max( delta_max, std::max( std::abs( dp[0] ), std::max( std::abs( dp[1] ), std::abs( dp[2] ) ) ) );
         //pe_LOG_WARNING_SECTION( log ) {
         //   log << "Contact reaction in contact frame: " << p_cf << "\nContact action in contact frame: " << diag_nto_[i]*p_cf + gdot_nto << "\n";
         //}

         p_[i] = p_wf;

         // Apply impulse right away
         dv_[body1_[i]->index_] += body1_[i]->getInvMass() * p_[i];
         dw_[body1_[i]->index_] += body1_[i]->getInvInertia() * ( r1_[i] % p_[i] );
         dv_[body2_[i]->index_] -= body2_[i]->getInvMass() * p_[i];
         dw_[body2_[i]->index_] -= body2_[i]->getInvInertia() * ( r2_[i] % p_[i] );
      }

#if 0
      Vec3 gdot2   ( ( v_[body1_[i]->index_] + dv_[body1_[i]->index_] ) - ( v_[body2_[i]->index_] + dv_[body2_[i]->index_] ) + ( w_[body1_[i]->index_] + dw_[body1_[i]->index_] ) % r1_[i] - ( w_[body2_[i]->index_] + dw_[body2_[i]->index_] ) % r2_[i] /* + diag_[i] * p */ );
      Vec3 gdot_nto2( trans( contactframe ) * gdot2 );
      pe_LOG_INFO_SECTION( log ) {
         log << "gdot_n2 = " << gdot_nto2[0] << "\n";
         log << "gdot_t2 = " << gdot_nto2[1] << "\n";
         log << "gdot_o2 = " << gdot_nto2[2] << "\n";
      }
      gdot_nto2[0] += ( /* + trans( n_[i] ) * ( body1_[i]->getPosition() + r1_[i] ) - ( body2_[i]->getPosition() + r2_[i] ) */ + dist_[i] ) * dtinv;
      pe_LOG_INFO_SECTION( log ) {
         log << "gdot_n2' = " << gdot_nto2[0] << "\n";
      }
#endif

      /*
       * compare DEM time-step with NSCD iteration:
       * - projections are the same
       * - velocities are the same if we use an explicit Euler discretization for the velocity time integration
       *
      f_cf[0] = -stiffness * dist_ - damping_n * gdot_n = -[(stiffness * dt) * dist_ * dtinv + damping_n * gdot_n] = -foo * (gdot_n + dist_ * dtinv) where foo = stiffness * dt = damping_n;
      f_cf[1] = -damping_t * gdot_t                     = -damping_t * gdot_t;
      f_cf[2] = -damping_t * gdot_o                     = -damping_t * gdot_o;

      or: f_cf = -diag(foo, damping_t, damping_t) * gdot_nto   (since gdot_nto[0] is modified)
      vs. f_cf = -diaginv * gdot_nto in NSCD iteration

      => The NSCD iteration is more or less a DEM time step where we choose the stiffness and damping parameters such that penetration is non-existent after a time step and contacts are truly static (tangential rel. vel. is zero) unless the friction force hits its limit

      f_cf[0] = std::max( 0, f_cf[0] );

      flimit = mu_ * f_cf[0];
      fsq = f_cf[1] * f_cf[1] + f_cf[2] * f_cf[2]
      if( fsq > flimit * flimit ) {
         f = flimit / sqrt( fsq );
         f_cf[1] *= f;
         f_cf[2] *= f;
      }

      f_wf = contactframe * f_cf;

      b1->addForceAtPos(  f_wf, gpos );
      b2->addForceAtPos( -f_wf, gpos );
      */
   }

   pe_PROFILING_SECTION {
      timeCollisionResponseSolving_.end();
      memCollisionResponseSolving_.stop();

#if HAVE_MPI
      pe_MPI_SECTION {
         //MPI_Barrier( MPISettings::comm() );
      }
#endif
   }

   return delta_max;
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
void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::synchronize()
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
            b->deregisterProcess( p );

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

      //MPI_Barrier( MPISettings::comm() );

      timeBodySyncCommunicate_.start();
      memBodySyncCommunicate_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Communication of body synchronization messages starts...\n";
   }

   // TODO send message as soon as it is ready, receive non-blocking and wait here for reception to complete
   // FIXME: If synchronize is called twice that is once by the user after a timestep for example this can break things due to a race condition (see bug #5); using different tags as a workaround
   if( lastSyncTag_ == mpitagHCTSSynchronizePositionsAndVelocities2 ) {
      communicate( mpitagHCTSSynchronizePositionsAndVelocities );
      lastSyncTag_ = mpitagHCTSSynchronizePositionsAndVelocities;
   }
   else {
      communicate( mpitagHCTSSynchronizePositionsAndVelocities2 );
      lastSyncTag_ = mpitagHCTSSynchronizePositionsAndVelocities2;
   }

   pe_PROFILING_SECTION {
      timeBodySyncCommunicate_.end();
      memBodySyncCommunicate_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedBodySync_.transfered( process->getRecvBuffer().size() );

      //MPI_Barrier( MPISettings::comm() );

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
                     pe_LOG_ERROR_SECTION( log ) {
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
                  else {
                     std::stringstream ss;
                     ss << "Registering distant process on rank " << objparam.reglist_[i] << " for body copy on rank " << myRank << " failed!\n"
                        << obj;
                     throw std::runtime_error( ss.str().c_str() );
                  }
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
                  else {
                     std::stringstream ss;
                     ss << "Registering distant process on rank " << objparam.reglist_[i] << " for body update on rank " << myRank << " failed!\n"
                        << b;
                     throw std::runtime_error( ss.str().c_str() );
                  }
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

      //MPI_Barrier( MPISettings::comm() );

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


//*************************************************************************************************
/*!\brief TODO
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
void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::synchronizeVelocities()
{
#if HAVE_MPI
   // Skip synchronization if we compute on a single process
   if( MPISettings::size() <= 1 )
      return;

   pe_PROFILING_SECTION {
      timeVelocitiesSync_.start();
      memVelocitiesSync_.start();

      timeVelocitiesSyncCorrectionsAssembling_.start();
      memVelocitiesSyncCorrectionsAssembling_.start();
   }

   // Sending local force contributions of shadow copies to owner.
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Assembling of velocity correction message starts...\n";
   }

   {
      size_t i = bodystorage_.size();
      for( BodyIterator body = bodystorageShadowCopies_.begin(); body != bodystorageShadowCopies_.end(); ++body, ++i ) {
         pe_INTERNAL_ASSERT( body->index_ == i, "Invalid body index." );

         if( dv_[i] == Vec3() && dw_[i] == Vec3() ) {
            // If we did not apply any corrections do not send anything.
            continue;
         }

         ProcessID process( body->getOwnerHandle() );
         Process::SendBuff& buffer( process->getSendBuffer() );

         pe_LOG_DEBUG_SECTION( log ) {
            log << "Sending velocity correction " << dv_[i] << ", " << dw_[i] << " of body " << body->getSystemID() << " to owner process " << process->getRank() << ".\n";
         }
         marshal( buffer, notificationType<RigidBodyVelocityCorrectionNotification>() );
         marshal( buffer, RigidBodyVelocityCorrectionNotification( *(*body), dv_[i], dw_[i] ) );
      }
   }

   pe_PROFILING_SECTION {
      timeVelocitiesSyncCorrectionsAssembling_.end();
      memVelocitiesSyncCorrectionsAssembling_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         sentVelocitiesSyncCorrections_.transfered( process->getSendBuffer().size() );

      //MPI_Barrier( MPISettings::comm() );

      timeVelocitiesSyncCorrectionsCommunicate_.start();
      memVelocitiesSyncCorrectionsCommunicate_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Communication of velocity correction message starts...\n";
   }

   communicate( mpitagHCTSSynchronizeVelocityCorrections );

   pe_PROFILING_SECTION {
      timeVelocitiesSyncCorrectionsCommunicate_.end();
      memVelocitiesSyncCorrectionsCommunicate_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedVelocitiesSyncCorrections_.transfered( process->getRecvBuffer().size() );

      //MPI_Barrier( MPISettings::comm() );

      timeVelocitiesSyncCorrectionsParsing_.start();
      memVelocitiesSyncCorrectionsParsing_.start();
   }

   // Receiving force and torque contributions
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Parsing of velocity correction message starts...\n";
   }

   for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process ) {
      Process::RecvBuff& buffer( process->getRecvBuffer() );
      NotificationType notificationType;

      // Receiving rigid body force/torque contribution notifications [DN] from neighbors (N) and distant processes (D)
      while( !buffer.isEmpty() ) {
         unmarshal( buffer, notificationType );

         switch( notificationType ) {
            case rigidBodyVelocityCorrectionNotification: {
               RigidBodyVelocityCorrectionNotification::Parameters objparam;
               unmarshal( buffer, objparam );

               BodyID b( *bodystorage_.find( objparam.sid_ ) );

               pe_INTERNAL_ASSERT( !b->isRemote(), "Update notification must only concern local bodies." );

               v_[b->index_] += relaxationParam_*objparam.dv_;
               w_[b->index_] += relaxationParam_*objparam.dw_;

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body velocity correction from neighboring process with rank " << process->getRank() << ":\ndv = " << objparam.dv_ << "\ndw = " << objparam.dw_ << "\nv_total = " << b->getLinearVel() << "\nw_total = " << b->getAngularVel() << "\nbody = " << b << "\n";
               }
               break;
            }
            default:
               throw std::runtime_error( "Received invalid notification type." );
         }
      }
   }

   pe_PROFILING_SECTION {
      timeVelocitiesSyncCorrectionsParsing_.end();
      memVelocitiesSyncCorrectionsParsing_.stop();

      //MPI_Barrier( MPISettings::comm() );

      timeVelocitiesSyncUpdatesAssembling_.start();
      memVelocitiesSyncUpdatesAssembling_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Assembling of velocity update message starts...\n";
   }

   {
      size_t i = 0;
      for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body, ++i ) {
         if( body->isGlobal() )
            continue;

         v_[i] += relaxationParam_*dv_[i];
         w_[i] += relaxationParam_*dw_[i];
         dv_[i] = Vec3();
         dw_[i] = Vec3();

         for( ProcessIterator it = body->beginProcesses(); it != body->endProcesses(); ++it ) {
            Process::SendBuff& buffer( it->getSendBuffer() );
            marshal( buffer, notificationType<RigidBodyVelocityCorrectionNotification>() );
            marshal( buffer, RigidBodyVelocityCorrectionNotification( *(*body), v_[i], w_[i] ) );

            pe_LOG_DEBUG_SECTION( log ) {
               log << "Sending velocity update " << v_[i] << ", " << w_[i] << " of body " << body->getSystemID() << " to process " << it->getRank() << " having a shadow copy.\n";
            }
         }
      }
   }

   pe_PROFILING_SECTION {
      timeVelocitiesSyncUpdatesAssembling_.end();
      memVelocitiesSyncUpdatesAssembling_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         sentVelocitiesSyncUpdates_.transfered( process->getSendBuffer().size() );

      //MPI_Barrier( MPISettings::comm() );

      timeVelocitiesSyncUpdatesCommunicate_.start();
      memVelocitiesSyncUpdatesCommunicate_.start();
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "Communication of velocity update message starts...\n";
   }

   communicate( mpitagHCTSSynchronizeVelocities );

   pe_PROFILING_SECTION {
      timeVelocitiesSyncUpdatesCommunicate_.end();
      memVelocitiesSyncUpdatesCommunicate_.stop();

      for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process )
         receivedVelocitiesSyncUpdates_.transfered( process->getRecvBuffer().size() );

      //MPI_Barrier( MPISettings::comm() );

      timeVelocitiesSyncUpdatesParsing_.start();
      memVelocitiesSyncUpdatesParsing_.start();
   }

   // Receiving velocity updates
   pe_LOG_DEBUG_SECTION( log ) {
      log << "Parsing of velocity update message starts...\n";
   }

   for( ProcessIterator process = processstorage_.begin(); process != processstorage_.end(); ++process ) {
      Process::RecvBuff& buffer( process->getRecvBuffer() );
      NotificationType notificationType;

      // Receiving rigid body force/torque contribution notifications [DN] from neighbors (N) and distant processes (D)
      while( !buffer.isEmpty() ) {
         unmarshal( buffer, notificationType );

         switch( notificationType ) {
            case rigidBodyVelocityCorrectionNotification: {
               RigidBodyVelocityCorrectionNotification::Parameters objparam;
               unmarshal( buffer, objparam );

               BodyID b( *bodystorageShadowCopies_.find( objparam.sid_ ) );

               pe_INTERNAL_ASSERT( b->isRemote(), "Update notification must only concern shadow copies." );

               v_[b->index_] = objparam.dv_;
               w_[b->index_] = objparam.dw_;
               dv_[b->index_] = Vec3();
               dw_[b->index_] = Vec3();

               pe_LOG_DEBUG_SECTION( log ) {
                  log << "Received rigid body velocity update from neighboring process with rank " << process->getRank() << ":\nv = " << objparam.dv_ << "\nw = " << objparam.dw_ << "\nbody = " << b << "\n";
               }
               break;
            }
            default:
               throw std::runtime_error( "Received invalid notification type." );
         }
      }
   }

   pe_PROFILING_SECTION {
      timeVelocitiesSyncUpdatesParsing_.end();
      memVelocitiesSyncUpdatesParsing_.stop();

      //MPI_Barrier( MPISettings::comm() );

      timeVelocitiesSyncGlobals_.start();
      memVelocitiesSyncGlobals_.start();
   }

   if( globalNonfixedBodies_.size() > 0 ) {
      size_t i;
      reductionBuffer_.resize( globalNonfixedBodies_.size() * 6 );

      i = 0;
      for( typename std::set<BodyID,LessSystemID>::iterator it = globalNonfixedBodies_.begin(); it != globalNonfixedBodies_.end(); ++it ) {
         reductionBuffer_[i++] = dv_[(*it)->index_][0];
         reductionBuffer_[i++] = dv_[(*it)->index_][1];
         reductionBuffer_[i++] = dv_[(*it)->index_][2];
         reductionBuffer_[i++] = dw_[(*it)->index_][0];
         reductionBuffer_[i++] = dw_[(*it)->index_][1];
         reductionBuffer_[i++] = dw_[(*it)->index_][2];
      }

      MPI_Allreduce( MPI_IN_PLACE, &reductionBuffer_[0], static_cast<int>( reductionBuffer_.size() ), MPITrait<real>::getType(), MPI_SUM, MPISettings::comm() );

      i = 0;
      for( typename std::set<BodyID,LessSystemID>::iterator it = globalNonfixedBodies_.begin(); it != globalNonfixedBodies_.end(); ++it, i += 6 ) {
         v_[(*it)->index_] += Vec3( reductionBuffer_[i    ], reductionBuffer_[i + 1], reductionBuffer_[i + 2] );
         w_[(*it)->index_] += Vec3( reductionBuffer_[i + 3], reductionBuffer_[i + 4], reductionBuffer_[i + 5] );
      }
   }

   pe_PROFILING_SECTION {
      timeVelocitiesSyncGlobals_.end();
      memVelocitiesSyncGlobals_.stop();

      //MPI_Barrier( MPISettings::comm() );

      timeVelocitiesSync_.end();
      memVelocitiesSync_.stop();
   }
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::clear()
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
inline void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::clearContacts()
{
   typename Contacts::Iterator       begin( contacts_.begin() );
   typename Contacts::Iterator const end  ( contacts_.end()   );

   for( ; begin!=end; ++begin )
      delete *begin;

   contacts_.clear();
}
//*************************************************************************************************




//=================================================================================================
//
//  TIME-INTEGRATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Calculates the initial velocity corrections of a given body.
 *
 * \param body The body whose velocities to time integrate
 * \param dv On return the initial linear velocity correction.
 * \param w On return the initial angular velocity correction.
 * \param dt The time step size.
 * \return void
 *
 * Calculates the velocity corrections effected by external forces and torques in an explicit Euler
 * time integration of the velocities of the given body. For fixed objects the velocity corrections
 * are set to zero. External forces and torques are reset if indicated by the settings.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::initializeVelocityCorrections( BodyID body, Vec3& dv, Vec3& dw, real dt ) const
{
   if( body->awake_ ) {
      if( !body->isFixed() ) {
         dv = ( body->getInvMass() * dt ) * body->getForce();
         dw = dt * ( body->getInvInertia() * body->getTorque() );
      }
   }
   else {
      pe_INTERNAL_ASSERT( body->getLinearVel() == Vec3( 0, 0, 0 ) && body->getAngularVel() == Vec3( 0, 0, 0 ), "Sleeping body has non-zero velocities." );
      pe_INTERNAL_ASSERT( body->getForce() == Vec3( 0, 0, 0 ) && body->getTorque() == Vec3( 0, 0, 0 ), "Sleeping body has non-zero forces or torques." );
   }

   if( Settings::forceReset() )
      body->resetForce();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Time integration of the position and orientation of a given body.
 *
 * \param body The body whose position and orientation to time integrate
 * \param v The linear velocity to use for time integration of the position.
 * \param w The angular velocity to use for time integration of the orientation.
 * \param dt The time step size.
 * \return void
 *
 * Performs an Euler time integration of the positions of the given body. Velocities are damped if
 * indicated by the settings and stored back in the body properties. The bounding box is
 * recalculated and it is redetermined whether the body is awake or not. Also the data
 * structure tracking the contacts attached to the body are cleared and
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::HardContactSemiImplicitTimesteppingSolvers> >::integratePositions( BodyID body, Vec3 v, Vec3 w, real dt ) const
{
   // Resetting the contact node and removing all attached contacts
   body->resetNode();
   body->clearContacts();

   if( body->awake_ ) {
      // Calculating the translational displacement
      body->gpos_ += v * dt;

      // Calculating the rotation angle
      const Vec3 phi( w * dt );

      // Calculating the new orientation
      const Quat dq( phi, phi.length() );
      body->q_ = dq * body->q_;
      body->R_ = body->q_.toRotationMatrix();

      // Damping the movement
      if( Settings::damping() < real(1) ) {
         const real drag( std::pow( Settings::damping(), dt ) );
         v *= drag;
         w *= drag;
      }

      // Storing the velocities back in the body properties
      body->v_ = v;
      body->w_ = w;

      if( body->getType() == unionType ) {
         // Updating the contained bodies
         UnionID u( static_cast<UnionID>( body ) );

         for( BodyIterator subbody=u->begin(); subbody!=u->end(); ++subbody )
            u->updateBody( *subbody, dq );
      }
      else if( body->getType() == planeType ) {
         // Updating secondary plane description (normal and displacement)
         PlaneID plane( static_cast<PlaneID>( body ) );
         plane->normal_ = body->q_.rotate( Vec3( 0, 0, 1 ) );
         plane->d_ = trans( plane->normal_ ) * body->gpos_;
      }

      // Setting the axis-aligned bounding box
      body->calcBoundingBox();

      // Calculating the current motion of the box
      body->calcMotion();
   }
}
//*************************************************************************************************



} // namespace pe

#endif

