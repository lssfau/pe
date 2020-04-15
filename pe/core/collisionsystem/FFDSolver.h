//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/FFDSolver.h
 *  \brief Specialization of the CollisionSystem class template for the FFD solver
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

#ifndef _PE_CORE_COLLISIONSYSTEM_FFDSOLVER_H_
#define _PE_CORE_COLLISIONSYSTEM_FFDSOLVER_H_


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
#include <pe/core/joint/JointStorage.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISection.h>
#include <pe/core/MPISettings.h>
#include <pe/core/MPITag.h>
#include <pe/core/MPITrait.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
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
/*!\brief Specialization of the collision system for the fast frictional dynamics solver.
 * \ingroup core
 *
 * This specialization of the CollisionSystem class template adapts the collision system of the
 * rigid body simulation world to the requirements of the fast frictional dynamics solver. In
 * contrast to other collision response algorithms, the fast frictional dynamics solver doesn't
 * require the setup of contact batches. Therefore the setting for the batch generation process
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
class CollisionSystem< C<CD,FD,BG,response::FFDSolver> >
   : public MPICommunication, public Singleton< CollisionSystem< C<CD,FD,BG,response::FFDSolver> >, logging::Logger >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,BG,response::FFDSolver>      Config;             //!< Type of the configuration.

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
   typedef typename Config::ProcessType         ProcessType;        //!< Type of the remote processes.
   typedef typename Config::ProcessID           ProcessID;          //!< Handle for a remote process.
   typedef typename Config::ConstProcessID      ConstProcessID;     //!< Handle for a constant remote process.

   typedef MPIEncoder<Config>                   Encoder;            //!< Type of the MPI encoder.
   typedef MPIDecoder<Config>                   Decoder;            //!< Type of the MPI decoder.

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
       pe::pe_CONSTRAINT_SOLVER macro. */
   typedef response::FFDSolver<Config>  ContactSolver;
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
   inline CoarseDetector& getCoarseDetector();
   inline ContactSolver&  getContactSolver();
   inline const BS&       getBodyStorage()       const;
   inline const PS&       getProcessStorage()    const;
   inline const AS&       getAttachableStorage() const;
   inline const Domain&   getDomain()            const;
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
   void thirdCommunication ();
   void fourthCommunication();
   //@}
   //**********************************************************************************************

   //**Profiling functions*************************************************************************
   /*!\name Profiling functions */
   //@{
   void collectProfilingData();
   //@}
   //**********************************************************************************************

   //**Send functions******************************************************************************
   /*!\name Send functions */
   //@{
   inline void sendBody       ( ProcessID p, ConstBodyID       b );
   inline void sendSphere     ( ProcessID p, ConstSphereID     s );
   inline void sendBox        ( ProcessID p, ConstBoxID        b );
   inline void sendCapsule    ( ProcessID p, ConstCapsuleID    c );
   inline void sendCylinder   ( ProcessID p, ConstCylinderID   c );
   inline void sendUnion      ( ProcessID p, ConstUnionID      u );
   inline void updateBody     ( ProcessID p, ConstBodyID       b );

   inline void sendAttachable ( ProcessID p, ConstAttachableID a );
   inline void sendGravity    ( ProcessID p, ConstGravityID    g );
   inline void sendSpring     ( ProcessID p, ConstSpringID     s );

   inline void sendForce      ( ProcessID p, ConstBodyID b );
   inline void sendConstraints( ProcessID p, ConstBodyID b );
   //@}
   //**********************************************************************************************

   //**Receive functions***************************************************************************
   /*!\name Receive functions */
   //@{
          BodyID    recvBody( ProcessID p );
          void      recvAttachable( ProcessID p );

          void      recvForces( ProcessID p );
          void      recvConstraints( ProcessID p );

   inline MPIEntity peek( ProcessID p ) const;
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
   size_t numContacts_;       //!< The number of time steps in the last time step.
   Contacts contacts_;        //!< The currently active contacts of the simulation world.
   CoarseDetector detector_;  //!< The active collision detection algorithm.
   ContactSolver solver_;     //!< The active collision response algorithm.
   BS bodystorage_;           //!< The rigid body storage.
   Decoder decoder_;          //!< The body decoder.
   PS processstorage_;        //!< The process storage.
   Domain domain_;            //!< The local process domain.
   AS attachablestorage_;     //!< The attachable storage.
   JS jointstorage_;          //!< The joint storage.

   timing::WcTimer timerComm1_;        //!< Timer for the first communication step.
   timing::WcTimer timerComm1Encode_;  //!< Timer for the encode phase of the first communication step.
   timing::WcTimer timerComm1MPI_;     //!< Timer for the MPI phase of the first communication step.
   timing::WcTimer timerComm1Decode_;  //!< Timer for the decode phase of the first communication step.
   timing::WcTimer timerHalfStep1_;    //!< Timer for the first half step.
   timing::WcTimer timerComm2_;        //!< Timer for the second communication step.
   timing::WcTimer timerComm2Encode_;  //!< Timer for the encode phase of the second communication step.
   timing::WcTimer timerComm2MPI_;     //!< Timer for the MPI phase of the second communication step.
   timing::WcTimer timerComm2Decode_;  //!< Timer for the decode phase of the second communication step.
   timing::WcTimer timerDetection_;    //!< Timer for the collision detection step.
   timing::WcTimer timerComm3_;        //!< Timer for the third communication step.
   timing::WcTimer timerComm3Encode_;  //!< Timer for the encode phase of the third communication step.
   timing::WcTimer timerComm3MPI_;     //!< Timer for the MPI phase of the third communication step.
   timing::WcTimer timerComm3Decode_;  //!< Timer for the decode phase of the third communication step.
   timing::WcTimer timerResponse_;     //!< Timer for the collision response step.
   timing::WcTimer timerHalfStep2_;    //!< Timer for the second half step.
   timing::WcTimer timerComm4_;        //!< Timer for the fourth communication step.
   timing::WcTimer timerComm4Encode_;  //!< Timer for the encode phase of the fourth communication step.
   timing::WcTimer timerComm4MPI_;     //!< Timer for the MPI phase of the fourth communication step.
   timing::WcTimer timerComm4Decode_;  //!< Timer for the decode phase of the fourth communication step.
   timing::WcTimer timerCleanUp_;      //!< Timer for the clean up step.

   double sumTimes_[21];  //!< Auxiliary data structure for the total runtime of each step.
   double minTimes_[21];  //!< Auxiliary data structure for the minimum runtime of each step.
   double maxTimes_[21];  //!< Auxiliary data structure for the maximum runtime of each step.
   double avgTimes_[21];  //!< Auxiliary data structure for the average runtime of each step.
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
CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::CollisionSystem()
   : MPICommunication  ( processstorage_, domain_ )
   , Singleton<CollisionSystem,logging::Logger>()  // Initialization of the Singleton base object
   , numContacts_      (    0 )                    // The number of time steps in the last time step
   , contacts_         ( 5000 )                    // The currently active contacts of the simulation world
   , detector_         ( bodystorage_ )            // The active collision detection algorithm
   , solver_           ( domain_ )                 // The active collision response algorithm
   , bodystorage_      ()
   , decoder_          ( bodystorage_, attachablestorage_ )
   , processstorage_   ()
   , domain_           ( processstorage_ )
   , attachablestorage_()
   , jointstorage_     ()
   , timerComm1_       ()                          // Timer for the first communication step
   , timerComm1Encode_ ()                          // Timer for the encode phase of the first communication step
   , timerComm1MPI_    ()                          // Timer for the MPI phase of the first communication step
   , timerComm1Decode_ ()                          // Timer for the decode phase of the first communication step
   , timerHalfStep1_   ()                          // Timer for the first half step
   , timerComm2_       ()                          // Timer for the second communication step
   , timerComm2Encode_ ()                          // Timer for the encode phase of the second communication step
   , timerComm2MPI_    ()                          // Timer for the MPI phase of the second communication step
   , timerComm2Decode_ ()                          // Timer for the decode phase of the second communication step
   , timerDetection_   ()                          // Timer for the collision detection step
   , timerComm3_       ()                          // Timer for the third communication step
   , timerComm3Encode_ ()                          // Timer for the encode phase of the third communication step
   , timerComm3MPI_    ()                          // Timer for the MPI phase of the third communication step
   , timerComm3Decode_ ()                          // Timer for the decode phase of the third communication step
   , timerResponse_    ()                          // Timer for the collision response step
   , timerHalfStep2_   ()                          // Timer for the second half step
   , timerComm4_       ()                          // Timer for the fourth communication step
   , timerComm4Encode_ ()                          // Timer for the encode phase of the fourth communication step
   , timerComm4MPI_    ()                          // Timer for the MPI phase of the fourth communication step
   , timerComm4Decode_ ()                          // Timer for the decode phase of the fourth communication step
   , timerCleanUp_     ()                          // Timer for the clean up step.
{
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
CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::~CollisionSystem()
{
   // Clearing the collision system
   clear();

   // Logging the FFD profiling results
   pe_PROFILING_SECTION
   {
      // Logging the profiling results for MPI parallel simulations
      pe_MPI_SECTION
      {
         pe_ROOT_SECTION
         {
            pe_LOG_INFO_SECTION( log ) {
               log << "FFD timing results ( sum / min / max / avg ):\n"
                   << "   Communication 1    : " << sumTimes_[ 0] << " / " << minTimes_[ 0] << " / " << maxTimes_[ 0] << " / " << avgTimes_[ 0] << "\n"
                   << "      Encode          : " << sumTimes_[ 1] << " / " << minTimes_[ 1] << " / " << maxTimes_[ 1] << " / " << avgTimes_[ 1] << "\n"
                   << "      MPI             : " << sumTimes_[ 2] << " / " << minTimes_[ 2] << " / " << maxTimes_[ 2] << " / " << avgTimes_[ 2] << "\n"
                   << "      Decode          : " << sumTimes_[ 3] << " / " << minTimes_[ 3] << " / " << maxTimes_[ 3] << " / " << avgTimes_[ 3] << "\n"
                   << "   Half step 1        : " << sumTimes_[ 4] << " / " << minTimes_[ 4] << " / " << maxTimes_[ 4] << " / " << avgTimes_[ 4] << "\n"
                   << "   Communication 2    : " << sumTimes_[ 5] << " / " << minTimes_[ 5] << " / " << maxTimes_[ 5] << " / " << avgTimes_[ 5] << "\n"
                   << "      Encode          : " << sumTimes_[ 6] << " / " << minTimes_[ 6] << " / " << maxTimes_[ 6] << " / " << avgTimes_[ 6] << "\n"
                   << "      MPI             : " << sumTimes_[ 7] << " / " << minTimes_[ 7] << " / " << maxTimes_[ 7] << " / " << avgTimes_[ 7] << "\n"
                   << "      Decode          : " << sumTimes_[ 8] << " / " << minTimes_[ 8] << " / " << maxTimes_[ 8] << " / " << avgTimes_[ 8] << "\n"
                   << "   Contact detection  : " << sumTimes_[ 9] << " / " << minTimes_[ 9] << " / " << maxTimes_[ 9] << " / " << avgTimes_[ 9] << "\n"
                   << "   Communication 3    : " << sumTimes_[10] << " / " << minTimes_[10] << " / " << maxTimes_[10] << " / " << avgTimes_[10] << "\n"
                   << "      Encode          : " << sumTimes_[11] << " / " << minTimes_[11] << " / " << maxTimes_[11] << " / " << avgTimes_[11] << "\n"
                   << "      MPI             : " << sumTimes_[12] << " / " << minTimes_[12] << " / " << maxTimes_[12] << " / " << avgTimes_[12] << "\n"
                   << "      Decode          : " << sumTimes_[13] << " / " << minTimes_[13] << " / " << maxTimes_[13] << " / " << avgTimes_[13] << "\n"
                   << "   Half step 2        : " << sumTimes_[14] << " / " << minTimes_[14] << " / " << maxTimes_[14] << " / " << avgTimes_[14] << "\n"
                   << "   Contact resolution : " << sumTimes_[15] << " / " << minTimes_[15] << " / " << maxTimes_[15] << " / " << avgTimes_[15] << "\n"
                   << "   Communication 4    : " << sumTimes_[16] << " / " << minTimes_[16] << " / " << maxTimes_[16] << " / " << avgTimes_[16] << "\n"
                   << "      Encode          : " << sumTimes_[17] << " / " << minTimes_[17] << " / " << maxTimes_[17] << " / " << avgTimes_[17] << "\n"
                   << "      MPI             : " << sumTimes_[18] << " / " << minTimes_[18] << " / " << maxTimes_[18] << " / " << avgTimes_[18] << "\n"
                   << "      Decode          : " << sumTimes_[19] << " / " << minTimes_[19] << " / " << maxTimes_[19] << " / " << avgTimes_[19] << "\n"
                   << "   Clean up           : " << sumTimes_[20] << " / " << minTimes_[20] << " / " << maxTimes_[20] << " / " << avgTimes_[20] << "\n";
            }
         }
      }

      // Logging the profiling results for serial simulations
      pe_SERIAL_SECTION
      {
         pe_LOG_INFO_SECTION( log ) {
            log << "FFD timing results:\n"
                << "   Half step 1        : " << sumTimes_[0] << "\n"
                << "   Contact detection  : " << sumTimes_[1] << "\n"
                << "   Half step 2        : " << sumTimes_[2] << "\n"
                << "   Contact resolution : " << sumTimes_[3] << "\n";
         }
      }
   }

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
inline size_t CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::getNumContacts() const
{
   return numContacts_;
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
inline typename CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::CoarseDetector&
   CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::getCoarseDetector()
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
inline typename CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::ContactSolver&
   CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::getContactSolver()
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
inline const typename CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::BS&
   CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::getBodyStorage() const
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
inline const typename CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::PS&
   CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::getProcessStorage() const
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
inline const typename CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::AS&
   CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::getAttachableStorage() const
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
inline const Domain& CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::getDomain() const
{
   return domain_;
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::add( BodyID body )
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::remove( BodyID body )
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
inline typename CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::BodyIterator CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::remove( BodyIterator body )
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::removeFromCollisionDetector( BodyID body )
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
/*!\brief Implementation of the fast frictional dynamics solver simulationStep() function.
 *
 * \param timestep Size of the time step.
 * \return void
 *
 * The spezialized simulation step function for the fast frictional dynamics solver performs a
 * time step as described in the paper by Kaufman at al [1]. The first step of the algorithm is
 * the update of all rigid bodies by first a position half step and then a velocity half step.
 * After the collision detection phase, the collision constraints for all colliding rigid bodies
 * are set up. The velocities of all rigid bodies are now updated a second time, either by
 * performing a collision resolution or a second velocity half step. The last step is a second
 * position half step.
 *
 * [1]: Kaufman, Edmunds, Pai: Fast Frictional Dynamics for Rigid Bodies, Department of Computer
 *      Science, Rutgers University
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::simulationStep( real timestep )
{
   // Early exit if no rigid bodies are specified
   pe_SERIAL_SECTION {
      if( getBodyStorage().isEmpty() ) return;
   }

   // Adapting the size of the time step for the two half steps
   timestep *= real(0.5);

   // Communicating all local and remote rigid bodies that cross a process boundary
   // This communication synchronizes the total forces acting on the rigid bodies.
   pe_MPI_SECTION {
      firstCommunication();
   }

   // Starting the first half step timer
   pe_PROFILING_SECTION {
      timerHalfStep1_.start();
   }

   // Two step update of all rigid bodies contained in the simulation world
   //  1.) First position half step
   //  2.) Velocity half step
   for( typename BS::Iterator body=bodystorage_.begin(); body!=bodystorage_.end(); ++body )
   {
      pe_INTERNAL_ASSERT( body->getWeightedCV() == real(0), "Invalid weighted constraint violation" );
      pe_INTERNAL_ASSERT( body->getCV()         == real(0), "Invalid constraint violation" );

      // Updating the global position of the rigid body
      body->firstPositionHalfStep( timestep );

      // Updating the velocity of the rigid body (don't update a fixed or remote body)
      if( !body->isFixed() && !body->isRemote() ) {
         body->firstVelocityHalfStep( timestep );
      }
   }

   // Ending the first half step timer
   pe_PROFILING_SECTION {
      timerHalfStep1_.end();
   }

   // Sending all rigid bodies that cross a new process boundary
   pe_MPI_SECTION {
      secondCommunication();
   }

   // Starting the collision detection timer
   pe_PROFILING_SECTION {
      timerDetection_.start();
   }

   // Finding all contacts
   detector_.findContacts( contacts_ );
   numContacts_ = contacts_.size();

   // Ending the collision detection timer and starting the collision response timer
   pe_PROFILING_SECTION {
      timerDetection_.end();
      timerResponse_.start();
   }

   // Setup of the collision constraints
   if( !contacts_.isEmpty() ) {
      solver_.checkContacts( contacts_ );
   }

   // Ending the collision response timer
   pe_PROFILING_SECTION {
      timerResponse_.end();
   }

   // Clearing the contacts
   clearContacts();

   // Sending all remote collision constraints for the collision resolution
   pe_MPI_SECTION {
      thirdCommunication();
   }

   // Logging the state of the rigid bodies
   pe_LOG_DEBUG_SECTION( log ) {
      log << "      State of the rigid bodies before the collision handling and time integration:\n";
      for( typename BS::ConstIterator b=getBodyStorage().begin(); b!=getBodyStorage().end(); ++b ) {
         log << "         Body " << b->getID() << ": pos = " << b->getPosition()
             << " , v = " << b->getLinearVel() << " , w = " << b->getAngularVel()
             << " , f = " << b->getForce() << " , t = " << b->getTorque() << "\n";
      }
   }

   // Two step update of all rigid bodies contained in the simulation world
   //  1.) Velocity update (either a collision update or a velocity half step)
   //  2.) Second position half step
   for( typename BS::Iterator body=bodystorage_.begin(); body!=bodystorage_.end(); ++body )
   {
      // Updating the velocity of the rigid body (don't update fixed or remote bodies)
      if( !body->isFixed() && !body->isRemote() )
      {
         // In case the body is involved in a collision with at least one other rigid body,
         // estimate a post-collision velocity via a collision resolution
         if( body->hasConstraints() )
         {
            // Starting the collision response timer
            pe_PROFILING_SECTION {
               timerResponse_.start();
            }

            // Solving the currently active collision constraints of the rigid body and
            // calculating a post-collision velocity
            solver_.resolveContacts( *body );

            // Ending the collision response timer
            pe_PROFILING_SECTION {
               timerResponse_.end();
            }
         }

         // In case the rigid body is not involved in any collisions, perform a second
         // velocity half step
         else {
            // Starting the second half step timer
            pe_PROFILING_SECTION {
               timerHalfStep2_.start();
            }

            // Performing the second velocity half step
            body->secondVelocityHalfStep( timestep );

            // Ending the second half step timer
            pe_PROFILING_SECTION {
               timerHalfStep2_.end();
            }
         }
      }

      // Starting the second half step timer
      pe_PROFILING_SECTION {
         timerHalfStep2_.start();
      }

      // Updating the global position of the rigid body
      body->secondPositionHalfStep( timestep );

      // Ending the second half step timer
      pe_PROFILING_SECTION {
         timerHalfStep2_.end();
      }
   }

   // Logging the state of the rigid bodies
   pe_LOG_DEBUG_SECTION( log ) {
      log << "      State of the rigid bodies after the collision handling and time integration:\n";
      for( typename BS::ConstIterator b=getBodyStorage().begin(); b!=getBodyStorage().end(); ++b ) {
         log << "         Body " << b->getID() << ": pos = " << b->getPosition()
             << " , v = " << b->getLinearVel() << " , w = " << b->getAngularVel()
             << " , f = " << b->getForce() << " , t = " << b->getTorque() << "\n";
      }
   }

   pe_MPI_SECTION
   {
      // Updating all rigid bodies that cross a process boundary
      fourthCommunication();

      // Logging the start of the clean up phase
      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Start of the clean up phase";
      }

      // Starting the clean up timer
      pe_PROFILING_SECTION {
         timerCleanUp_.start();
      }

      // Updating the remote flag and destroying non-local rigid bodies
      for( typename BS::Iterator body=bodystorage_.begin(); body!=bodystorage_.end(); )
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

         // Removing the rigid body in case it is no longer required by the local domain
         // In case the forces acting on the body are non-zero, the rigid body has been completely
         // moved from this process to the neighboring process in a single time step. In this case
         // the deletion of the body is postponed to the next time step to guarantee a force
         // synchronization.
         if( body->isRemote() && !body->hasForce() && !domain_.requires( *body ) ) {
            BodyID bodyid( *body );
            body = remove( body );
            delete bodyid;              // Destroying the rigid body
         }
         else ++body;
      }

      // Ending the clean up timer
      pe_PROFILING_SECTION {
         timerCleanUp_.end();
      }
   }

   // Collecting profiling data
   pe_PROFILING_SECTION {
      collectProfilingData();
   }
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
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::synchronize()
{
   // Limitions of the implementation:
   // - bodies that are fixed after the first synchonization will not be fixed on remote processes
   // - attachables that are destroyed on any process will not be destroyed on remote processes

   pe_INTERNAL_ASSERT( !processstorage_.isEmpty(), "Synchronization without connected processes" );

   typedef typename AS::Iterator  AttachableIterator;
   typedef typename PS::Iterator  ProcessIterator;

   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   // Synchronizing the rigid bodies contained in the local domain of the simulation world:
   for( BodyIterator body=bodystorage_.begin(); body!=bodystorage_.end(); ++body )
   {
      // Ignoring global and remote rigid bodies
      if( body->isGlobal() || body->isRemote() ) {
         continue;
      }

      // Sending the rigid body to all processes it is required by
      for( ProcessIterator process=pbegin; process!=pend; ++process ) {
         if( process->requires( *body ) ) {
            if( body->isRegistered( *process ) && !body->isFixed() ) {
               updateBody( *process, *body );
            }
            else {
               sendBody( *process, *body );
               body->registerProcess( *process );
            }
         }
         else if( body->isRegistered( *process ) ) {
            updateBody( *process, *body );
            body->deregisterProcess( *process );
         }
      }
   }

   // Synchronizing the attachables contained in the local domain of the simulation world.
   for( AttachableIterator attachable=attachablestorage_.begin(); attachable!=attachablestorage_.end(); ++attachable )
   {
      // Sending the attachable to all processes it is required by
      for( ProcessIterator process=pbegin; process!=pend; ++process ) {
         if( process->requires( *attachable ) ) {
            sendAttachable( *process, *attachable );
         }
      }
   }

   // Performing the MPI communication
   communicate( mpitagFFDSynchronizeComplete );

   // Receiving the rigid bodies contained in this process
   for( ProcessIterator process1=pbegin; process1!=pend; ++process1 )
   {
      while( process1->hasData() )
      {
         // Receiving a rigid body
         if( peek( *process1 ) == rigidbody )
         {
            // Receiving the rigid body
            BodyID body = recvBody( *process1 );
            pe_INTERNAL_ASSERT( body->isRemote() , "Invalid local rigid body detected" );
            pe_INTERNAL_ASSERT( body->isUpdated(), "Invalid update flag detected"      );

            // Registering the processes the rigid body is required by
            for( ProcessIterator process2=pbegin; process2!=pend; ++process2 ) {
               if( process2->requires( body ) )
                  body->registerProcess( *process2 );
            }
         }

         // Receiving an attachable
         else {
            recvAttachable( *process1 );
         }
      }
   }

   // Postprocessing of the rigid bodies
   //  1.) Updating the remote flag of the rigid bodies.
   //  2.) Removing all rigid bodies that are no longer required by the local domain.
   for( BodyIterator body=bodystorage_.begin(); body!=bodystorage_.end(); )
   {
      // Ignoring global rigid bodies
      if( body->isGlobal() ) {
         ++body;
         continue;
      }

      // Setting the remote flag of the rigid body
      body->setRemote( !domain_.ownsPoint( body->getPosition() ) );

      // Removing the rigid body in case it is no longer required by the local domain
      if( body->isRemote() && !domain_.requires( *body ) ) {
         BodyID bodyid( *body );
         body = remove( body );
         delete bodyid;              // Destroying the rigid body
      }
      else ++body;
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief First MPI communication during a single time step of the FFD solver.
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
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::firstCommunication()
{
   // Logging the start of the first communication step
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Start of the first communication step";
   }

   // Starting the timer for the first communication step and the initial encode phase
   pe_PROFILING_SECTION {
      timerComm1_.start();
      timerComm1Encode_.start();
   }

   // Synchronizing the forces of all local and remote rigid bodies
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

      for( MPIRigidBodyTrait::ProcessIterator process=pbegin; process!=pend; ++process ) {
         sendForce( *process, *body );
      }

      // Deregistering all processes
      body->clearProcesses();
   }

   // Ending the timer for the encode phase and starting the timer of the MPI phase
   pe_PROFILING_SECTION {
      timerComm1Encode_.end();
      timerComm1MPI_.start();
   }

   // Performing the MPI communication
   communicate( mpitagFFDSynchronize1 );

   // Ending the timer for the MPI phase and starting the timer of the decode phase
   pe_PROFILING_SECTION {
      timerComm1MPI_.end();
      timerComm1Decode_.start();
   }

   // Receiving the forces and torques from the connected processes
   typedef typename PS::Iterator  ProcessIterator;
   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   for( ProcessIterator process=pbegin; process!=pend; ++process ) {
      recvForces( *process );
   }

   // Ending the timer for the decode phase and the first communication step
   pe_PROFILING_SECTION {
      timerComm1Decode_.end();
      timerComm1_.end();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Second MPI communication during a single time step of the FFD solver.
 *
 * \return void
 *
 * This communciation sends all rigid bodies that have crossed a new process boundary due to
 * the first position half step.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::secondCommunication()
{
   // Logging the start of the second communication step
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Start of the second communication step";
   }

   // Starting the timer for the second communication step and the initial encode phase
   pe_PROFILING_SECTION {
      timerComm2_.start();
      timerComm2Encode_.start();
   }

   const BodyIterator bbegin( bodystorage_.begin() );
   const BodyIterator bend  ( bodystorage_.end()   );

   typedef typename PS::Iterator  ProcessIterator;
   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   // Resetting the update flag of all bodies
   for( BodyIterator body=bbegin; body!=bend; ++body )
      body->setUpdated( false );

   // Updating remote rigid bodies on the remote processes and sending
   // all local rigid bodies that have moved into a new remote process
   for( BodyIterator body=bbegin; body!=bend; ++body )
   {
      // Setting the update flag of the rigid body
      body->setUpdated( false );

      // Don't send a fixed or remote rigid body
      if( body->isFixed() || body->isRemote() ) continue;

      for( ProcessIterator process=pbegin; process!=pend; ++process )
      {
         // In case the rigid body is already registered with the remote process,
         // an update for the rigid body is sent to the remote process
         if( body->isRegistered( *process ) ) {
            updateBody( *process, *body );
         }

         // In case the rigid body is not registered but required by the remote process,
         // the complete rigid body is sent and registered with the remote process along
         // with all attached attachables
         else if( process->requires( *body ) )
         {
            sendBody( *process, *body );
            body->registerProcess( *process );

            typedef typename BodyType::AttachableIterator  AttachableIterator;
            const AttachableIterator abegin( body->beginAttachables() );
            const AttachableIterator aend  ( body->endAttachables()   );

            // Sending the attached attachables
            // An attachable is only send to the remote process in case all attached rigid
            // bodies are known on the remote process. This requires the rigid bodies to be
            // marked as updated.
            for( AttachableIterator attachable=abegin; attachable!=aend; ++attachable )
            {
               const typename AttachableType::Iterator abbegin( attachable->begin() );
               const typename AttachableType::Iterator abend  ( attachable->end()   );

               bool sent( true );

               for( typename AttachableType::Iterator attachedBody=abbegin; attachedBody!=abend; ++attachedBody )
               {
                  if( *attachedBody == *body || attachedBody->isRemote() )
                     continue;

                  if( !attachedBody->isUpdated() ) {
                     sent = false;
                     break;
                  }
               }

               if( sent ) {
                  sendAttachable( *process, *attachable );
               }
            }
         }
      }

      // Setting the updated flag
      body->setUpdated( true );
   }

   // Ending the timer for the encode phase and starting the timer of the MPI phase
   pe_PROFILING_SECTION {
      timerComm2Encode_.end();
      timerComm2MPI_.start();
   }

   // Performing the MPI communication
   communicate( mpitagFFDSynchronize2 );

   // Ending the timer for the MPI phase and starting the timer of the decode phase
   pe_PROFILING_SECTION {
      timerComm2MPI_.end();
      timerComm2Decode_.start();
   }

   // Receiving the remote rigid bodies newly contained in this process
   for( ProcessIterator process=pbegin; process!=pend; ++process )
   {
      while( process->hasData() )
      {
         // Receiving a rigid body
         if( peek( *process ) == rigidbody )
         {
            // Receiving the rigid body
            BodyID body = recvBody( *process );
            pe_INTERNAL_ASSERT( body->isRemote()     , "Invalid local rigid body detected"     );
            pe_INTERNAL_ASSERT( body->isUpdated()    , "Invalid update flag detected"          );
            pe_INTERNAL_ASSERT( !body->hasProcesses(), "Invalid registered processes detected" );

            // Registering the remote process
            body->registerProcess( *process );
         }

         // Receiving an attachable
         else {
            recvAttachable( *process );
         }
      }
   }

   // Checking the update flags of the remote rigid bodies
   pe_INTERNAL_ASSERT( checkUpdateFlags(), "Invalid update flags detected" );

   // Ending the timer for the decode phase and the second communication step
   pe_PROFILING_SECTION {
      timerComm2Decode_.end();
      timerComm2_.end();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Third MPI communication during a single time step of the FFD solver.
 *
 * \return void
 *
 * This communication exchanges collision constraints for the collision resolution.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::thirdCommunication()
{
   // Logging the start of the third communication step
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Start of the third communication step";
   }

   // Starting the timer for the third communication step and the initial encode phase
   pe_PROFILING_SECTION {
      timerComm3_.start();
      timerComm3Encode_.start();
   }

   // Sending the constraints off all remote rigid bodies
   const BodyIterator bbegin( bodystorage_.begin() );
   const BodyIterator bend  ( bodystorage_.end()   );

   for( BodyIterator body=bbegin; body!=bend; ++body )
   {
      // Don't handle a fixed or local rigid body or a remote body without constraints
      if( body->isFixed() || !body->isRemote() || !body->hasConstraints() ) continue;

      // Sending the constraints to all registered processes
      const MPIRigidBodyTrait::ProcessIterator pbegin( body->beginProcesses() );
      const MPIRigidBodyTrait::ProcessIterator pend  ( body->endProcesses()   );

      for( MPIRigidBodyTrait::ProcessIterator process=pbegin; process!=pend; ++process ) {
         sendConstraints( *process, *body );
      }
   }

   // Ending the timer for the encode phase and starting the timer of the MPI phase
   pe_PROFILING_SECTION {
      timerComm3Encode_.end();
      timerComm3MPI_.start();
   }

   // Initiating the MPI communication
   communicate( mpitagFFDSynchronize3 );

   // Ending the timer for the MPI phase and starting the timer of the decode phase
   pe_PROFILING_SECTION {
      timerComm3MPI_.end();
      timerComm3Decode_.start();
   }

   // Receiving all constraints from the remote processes
   typedef typename PS::Iterator  ProcessIterator;
   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   for( ProcessIterator process=pbegin; process!=pend; ++process ) {
      recvConstraints( *process );
   }

   // Ending the timer for the decode phase and the third communication step
   pe_PROFILING_SECTION {
      timerComm3Decode_.end();
      timerComm3_.end();
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Fourth MPI communication during a single time step of the FFD solver.
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
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::fourthCommunication()
{
   // Logging the start of the fourth communication step
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Start of the fourth communication step";
   }

   // Starting the timer for the fourth communication step and the initial encode phase
   pe_PROFILING_SECTION {
      timerComm4_.start();
      timerComm4Encode_.start();
   }

   const BodyIterator bbegin( bodystorage_.begin() );
   const BodyIterator bend  ( bodystorage_.end()   );

   typedef typename PS::Iterator  ProcessIterator;
   const ProcessIterator pbegin( processstorage_.begin() );
   const ProcessIterator pend  ( processstorage_.end()   );

   // Resetting the update flag of all bodies
   for( BodyIterator body=bbegin; body!=bend; ++body )
      body->setUpdated( false );

   // Updating all remote rigid bodies on the remote processes
   for( BodyIterator body=bbegin; body!=bend; ++body )
   {
      // Don't send a fixed or remote rigid body
      if( body->isFixed() || body->isRemote() ) continue;

      // Checking the update and the global flag
      pe_INTERNAL_ASSERT( !body->isUpdated(), "Invalid update flag detected"       );
      pe_INTERNAL_ASSERT( !body->isGlobal() , "Invalid global rigid body detected" );

      for( ProcessIterator process=pbegin; process!=pend; ++process )
      {
         // In case the rigid body is required by the remote process, it is either updated
         // (in case it is already registered) or completely sent (in case it is not registered)
         if( process->requires( *body ) )
         {
            // Updating the rigid body on the remote process
            if( body->isRegistered( *process ) ) {
               updateBody( *process, *body );
            }

            // Sending the rigid body to the remote process along with all attached attachables
            else {
               sendBody( *process, *body );
               body->registerProcess( *process );

               typedef typename BodyType::AttachableIterator  AttachableIterator;
               const AttachableIterator abegin( body->beginAttachables() );
               const AttachableIterator aend  ( body->endAttachables()   );

               // Sending the attached attachables
               // An attachable is only send to the remote process in case all attached rigid
               // bodies are known on the remote process. This requires the rigid bodies to be
               // marked as updated.
               for( AttachableIterator attachable=abegin; attachable!=aend; ++attachable )
               {
                  const typename AttachableType::Iterator abbegin( attachable->begin() );
                  const typename AttachableType::Iterator abend  ( attachable->end()   );

                  bool sent( true );

                  for( typename AttachableType::Iterator attachedBody=abbegin; attachedBody!=abend; ++attachedBody )
                  {
                     if( *attachedBody == *body || attachedBody->isRemote() )
                        continue;

                     if( !attachedBody->isUpdated() ) {
                        sent = false;
                        break;
                     }
                  }

                  if( sent ) {
                     sendAttachable( *process, *attachable );
                  }
               }
            }
         }

         // In case the rigid body is not contained in the remote process but registered,
         // update it one last time and deregister the remote process
         else if( body->isRegistered( *process ) ) {
            updateBody( *process, *body );
            body->deregisterProcess( *process );
         }
      }

      // Setting the updated flag
      body->setUpdated( true );
   }

   // Ending the timer for the encode phase and starting the timer of the MPI phase
   pe_PROFILING_SECTION {
      timerComm4Encode_.end();
      timerComm4MPI_.start();
   }

   // Performing the MPI communication
   communicate( mpitagFFDSynchronize4 );

   // Ending the timer for the MPI phase and starting the timer of the decode phase
   pe_PROFILING_SECTION {
      timerComm4MPI_.end();
      timerComm4Decode_.start();
   }

   // Receiving the updates for the remote rigid bodies from the connected processes
   for( ProcessIterator process=pbegin; process!=pend; ++process )
   {
      while( process->hasData() )
      {
         // Receiving a rigid body
         if( peek( *process ) == rigidbody )
         {
            // Receiving the rigid body
            BodyID body = recvBody( *process );
            pe_INTERNAL_ASSERT( body->isRemote() , "Invalid local rigid body detected" );
            pe_INTERNAL_ASSERT( body->isUpdated(), "Invalid update flag detected"      );

            // Deregistering all processes
            body->clearProcesses();

            // Registering all connected processes the body is required by
            for( ProcessIterator p=pbegin; p!=pend; ++p ) {
               if( p->requires( body ) )
                  body->registerProcess( *p );
            }
         }

         // Receiving an attachable
         else {
            recvAttachable( *process );
         }
      }
   }

   // Checking the update flags of the remote rigid bodies
   pe_INTERNAL_ASSERT( checkUpdateFlags(), "Invalid update flags detected" );

   // Ending the timer for the decode phase and the fourth communication step
   pe_PROFILING_SECTION {
      timerComm4Decode_.end();
      timerComm4_.end();
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendBody( ProcessID p, ConstBodyID b )
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendSphere( ProcessID p, ConstSphereID s )
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendBox( ProcessID p, ConstBoxID b )
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
 * \param p The process to send to.
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendCapsule( ProcessID p, ConstCapsuleID c )
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
 * \param p The process to send to.
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendCylinder( ProcessID p, ConstCylinderID c )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending cylinder " << c->getSystemID() << " to process " << p->getRank() << "\n"
          << "         Global position  = " << c->getPosition() << "\n"
          << "         Linear velocity  = " << c->getLinearVel() << "\n"
          << "         Angular velocity = " << c->getAngularVel();
   }

   // Encoding cylinder primitive
   Encoder::encodeCylinder( p->getSendBuffer(), c, p->getOffset() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a union compound geometry for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendUnion( ProcessID p, ConstUnionID u )
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
 * \param p The process to send to.
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
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::updateBody( ProcessID p, ConstBodyID b )
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
/*!\brief Encoding an attachable for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
 * \param a The attachable to be send to the remote MPI process.
 * \return void
 * \exception std::runtime_error Unknown attachable type.
 *
 * This function encodes the given attachable for the send operation to the remote MPI
 * process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendAttachable( ProcessID p, ConstAttachableID a )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending " << a->getType() << " " << a->getSystemID() << " to process " << p->getRank();
   }

   // Encoding the attachable
   Encoder::encodeAttachable( p->getSendBuffer(), a );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a gravity force generator for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
 * \param g The gravity force generator to be send to the remote MPI process.
 * \return void
 *
 * This function encodes the given gravity force generator for the send operation to the remote
 * MPI process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendGravity( ProcessID p, ConstGravityID g )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending gravity " << g->getSystemID() << " to process " << p->getRank();
   }

   // Encoding gravity force generator
   Encoder::encodeGravity( p->getSendBuffer(), g );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding a spring force generator for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
 * \param s The spring force generator to be send to the remote MPI process.
 * \return void
 *
 * This function encodes the given spring force generator for the send operation to the remote
 * MPI process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendSpring( ProcessID p, ConstSpringID s )
{
   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending spring " << s->getSystemID() << " to process " << p->getRank();
   }

   // Encoding spring force generator
   Encoder::encodeSpring( p->getSendBuffer(), s );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Encoding the forces acting on the given body for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendForce( ProcessID p, ConstBodyID b )
{
   // Checking the the finiteness, the mobility, and the locality of the rigid body
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


//*************************************************************************************************
/*!\brief Encoding the constraints on the given body for the send operation to the remote MPI process.
 *
 * \param p The process to send to.
 * \param b The rigid body.
 * \return void
 *
 * This function encodes the motion constraints on the given rigid body for the send operation
 * to the remote MPI process. Note that it is invalid to send an infinite, fixed or local rigid
 * body!
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::sendConstraints( ProcessID p, ConstBodyID b )
{
   using namespace pe::response::ffd;

   // Checking the finiteness, the mobility, and the locality of the rigid body
   pe_INTERNAL_ASSERT( b->isFinite(), "Invalid infinite rigid body detected" );
   pe_INTERNAL_ASSERT( !b->isFixed(), "Invalid fixed rigid body detected"    );
   pe_INTERNAL_ASSERT( b->isRemote(), "Invalid local rigid body detected"    );

   // Checking if the rigid body has constraints
   // This check includes checks for the correct number of normal constraints, constraint offsets,
   // friction coefficients, and friction bounds.
   pe_INTERNAL_ASSERT( b->hasConstraints(), "Rigid body without constraints detected" );

   // Logging the send operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Sending constraints for " << b->getType() << " " << b->getSystemID() << " to process " << p->getRank();
   }

   const size_t constraints( b->countConstraints() );

   p->getSendBuffer() << b->getSystemID();    // Sending the system ID of the rigid body
   p->getSendBuffer() << constraints;         // Sending the number of constraints
   p->getSendBuffer() << b->getWeightedCV();  // Sending the sum of the weighted constraint violations
   p->getSendBuffer() << b->getCV();          // Sending the total sum of all constraint violations

   // Sending the normal constraints
   for( size_t i=0; i<constraints; ++i )
      marshal( p->getSendBuffer(), b->getConstraint( i ) );

   // Sending the constraint offsets
   for( size_t i=0; i<constraints; ++i )
      p->getSendBuffer() << b->getOffset( i );

   // Sending friction-related data
   if( friction )
   {
      const size_t bounds( b->countBounds() );

      // Sending the coefficients of friction
      for( size_t i=0; i<constraints; ++i )
         p->getSendBuffer() << b->getCoF( i );

      // Sending the friction bounds
      for( size_t i=0; i<bounds; ++i )
         marshal( p->getSendBuffer(), b->getBound( i ) );
   }
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
 * \param p The process to receive from.
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
inline typename CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::BodyID
   CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::recvBody( ProcessID p )
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
/*!\brief Receiving a single attachable from the remote MPI process.
 *
 * \param p The process to receive from.
 * \return The attachable received from the remote MPI process.
 * \exception std::runtime_error Received invalid message tag.
 *
 * This function decodes a single attachable from the message recevied from the remote MPI
 * process. The received attachable is locally instantiated and automatically attached to
 * the according rigid bodies.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::recvAttachable( ProcessID p )
{
   // Decoding the attachable
   const typename Decoder::AttachableData data = decoder_.decodeAttachable( p->getRecvBuffer() );

   // Logging the receive operation
   pe_LOG_DETAIL_SECTION( log ) {
      log << "      Receiving " << boost::get<0>( data ) << " " << boost::get<1>( data ) << " from process " << p->getRank() << "\n"
          << "         Instantiation of the attachable ";
      if( boost::get<2>( data ) ) log << "succeeded";
      else                        log << "failed";
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Decoding all forces and torques received from the remote MPI process.
 *
 * \param p The process to receive from.
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
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::recvForces( ProcessID p )
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


//*************************************************************************************************
/*!\brief Decoding all constraints received from the remote MPI process.
 *
 * \param p The process to receive from.
 * \return void
 *
 * This function decodes all constraints received from the remote MPI process.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::recvConstraints( ProcessID p )
{
   using namespace pe::response::ffd;

   const BodyIterator begin( bodystorage_.begin() );
   const BodyIterator end  ( bodystorage_.end()   );

   id_t sid(0);
   size_t count(0);
   real wcv, cv, tmp;
   Twist twist;

   // Extracting the received constraints
   while( p->hasData() )
   {
      p->getRecvBuffer() >> sid;    // Extracting the unique system-specific ID
      p->getRecvBuffer() >> count;  // Extracting the number of constraints

      // Searching the rigid body
      BodyIterator body( bodystorage_.find( sid ) );
      pe_INTERNAL_ASSERT( body != end, "Constraints for unknown rigid body received" );

      // Skipping the constraints for a remote rigid body
      if( body->isRemote() ) {
         p->getRecvBuffer().skip( sizeof(real)*( count*( 8+6*frictionSamples ) + 2 ) );
         continue;
      }

      // Checking the finiteness and the mobility of the rigid body
      pe_INTERNAL_ASSERT( body->isFinite(), "Invalid infinite rigid body detected" );
      pe_INTERNAL_ASSERT( !body->isFixed(), "Invalid fixed rigid body detected"    );

      // Logging the receive operation
      pe_LOG_DETAIL_SECTION( log ) {
         log << "      Receiving constraints for " << body->getType() << " " << body->getSystemID() << " from process " << p->getRank();
      }

      // Extracting the constraint violation values
      p->getRecvBuffer() >> wcv;
      p->getRecvBuffer() >> cv;
      body->addWeightedCV( wcv );
      body->addCV( cv );

      // Extracting the normal constraints
      for( size_t i=0; i<count; ++i ) {
         unmarshal( p->getRecvBuffer(), twist );
         body->addConstraint( twist );
      }

      // Extracting the constraint offsets
      for( size_t i=0; i<count; ++i ) {
         p->getRecvBuffer() >> tmp;
         body->addOffset( tmp );
      }

      // Extracting friction-related data
      if( friction )
      {
         // Extracting the coefficients of friction
         for( size_t i=0; i<count; ++i ) {
            p->getRecvBuffer() >> tmp;
            body->addCoF( tmp );
         }

         // Extracting the friction bounds
         for( size_t i=0; i<count*frictionSamples; ++i ) {
            unmarshal( p->getRecvBuffer(), twist );
            body->addBound( twist );
         }
      }
   }
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Reading the type of the next entity in the receive buffer.
 *
 * \param p The process to receive from.
 * \return \a rigidbody if the next entity is a rigid body, \a attachable if it is an attachable.
 *
 * This function reads the type of the next entity in the receive buffer without extracting it
 * from the buffer. In case the next entity is a rigid body, the function returns \a rigidbody,
 * in case it is an attachable it returns \a attachable. The attempt to peek into an empty
 * receive buffer results in a \a std::invalid_argument exception.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline MPIEntity CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::peek( ProcessID p ) const
{
   return decoder_.peek( p->getRecvBuffer() );
}
//*************************************************************************************************




//=================================================================================================
//
//  PROFILING FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Collecting profiling data.
 *
 * \return void
 *
 * This function collects profiling data for both MPI parallel as well as serial simulations.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::collectProfilingData()
{
   // Collecting profiling data for MPI parallel simulations
   pe_MPI_SECTION
   {
      const int          root( MPISettings::root() );          // The MPI root process
      const int          size( MPISettings::size() );          // The total number of MPI processes
      const MPI_Datatype type( MPITrait<double>::getType() );  // Data type for the MPI communication
      const MPI_Comm     comm( MPISettings::comm() );          // The current MPI communicator

      double times[21] = { timerComm1_.total(),        // WC-time of the first communication step
                           timerComm1Encode_.total(),  // WC-time of the first encode phase
                           timerComm1MPI_.total(),     // WC-time of the first MPI communication
                           timerComm1Decode_.total(),  // WC-time of the first decode phase
                           timerHalfStep1_.total(),    // WC-time of the first half step
                           timerComm2_.total(),        // WC-time of the second communication step
                           timerComm2Encode_.total(),  // WC-time of the second encode phase
                           timerComm2MPI_.total(),     // WC-time of the second MPI communication
                           timerComm2Decode_.total(),  // WC-time of the second decode phase
                           timerDetection_.total(),    // WC-time of the collision detection step
                           timerComm3_.total(),        // WC-time of the third communication step
                           timerComm3Encode_.total(),  // WC-time of the third encode phase
                           timerComm3MPI_.total(),     // WC-time of the third MPI communication
                           timerComm3Decode_.total(),  // WC-time of the third decode phase
                           timerResponse_.total(),     // WC-time of the collision response step
                           timerHalfStep2_.total(),    // WC-time of the second half step
                           timerComm4_.total(),        // WC-time of the fourth communication step
                           timerComm4Encode_.total(),  // WC-time of the fourth encode phase
                           timerComm4MPI_.total(),     // WC-time of the fourth MPI communication
                           timerComm4Decode_.total(),  // WC-time of the fourth decode phase
                           timerCleanUp_.total() };    // WC-time of the clean up step

      MPI_Reduce( times, sumTimes_, 21, type, MPI_SUM, root, comm );
      MPI_Reduce( times, minTimes_, 21, type, MPI_MIN, root, comm );
      MPI_Reduce( times, maxTimes_, 21, type, MPI_MAX, root, comm );

      for( int i=0; i<21; ++i ) {
         avgTimes_[i] = sumTimes_[i] / size;
      }
   }

   // Collecting profiling data for serial simulations
   pe_SERIAL_SECTION
   {
      sumTimes_[0] = timerHalfStep1_.total();
      sumTimes_[1] = timerDetection_.total();
      sumTimes_[2] = timerResponse_.total();
      sumTimes_[3] = timerHalfStep2_.total();
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::clear()
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
inline void CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::clearContacts()
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
inline bool CollisionSystem< C<CD,FD,BG,response::FFDSolver> >::checkUpdateFlags()
{
   bool error( false );

   const BodyIterator bbegin( bodystorage_.begin() );
   const BodyIterator bend  ( bodystorage_.end()   );

   for( BodyIterator body=bbegin; body!=bend; ++body )
   {
      if( !body->isFixed() && !body->isUpdated() ) {
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
