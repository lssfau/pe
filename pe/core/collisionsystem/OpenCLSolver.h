//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/FFDSolver.h
 *  \brief Specialization of the CollisionSystem class template for the OpenCL solver
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

#ifndef _PE_CORE_COLLISIONSYSTEM_OPENCLSOLVER_H_
#define _PE_CORE_COLLISIONSYSTEM_OPENCLSOLVER_H_


//*************************************************************************************************
// OpenCL includes
//*************************************************************************************************

#if HAVE_OPENCL
#include <pe/core/OpenCLBodyManager.h>
#endif


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
#include <pe/core/Configuration.h>
#include <pe/core/contact/Contact.h>
#include <pe/core/contact/ContactVector.h>
#include <pe/core/detection/coarse/Detectors.h>
#include <pe/core/detection/fine/Detectors.h>
#include <pe/core/domaindecomp/Domain.h>
#include <pe/core/domaindecomp/Process.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/joint/JointStorage.h>
#include <pe/core/ProfilingSection.h>
#include <pe/core/response/Solvers.h>
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

#if HAVE_OPENCL

//*************************************************************************************************
/*!\brief Specialization of the collision system for the OpenCL solver.
 * \ingroup core
 *
 * This specialization of the CollisionSystem class template is used for the OpenCL collision
 * response algorithms.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >
   : public Singleton< CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >, logging::Logger >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,BG,response::OpenCLSolver>   Config;             //!< Type of the configuration.

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

   typedef batches::BatchVector<Contact>        Batches;            //!< Batch container type.
   typedef Batches::Batch                       Batch;              //!< Type of a single batch.

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

   //! Base type of the collision response algorithm.
   typedef response::OpenCLSolver<Config, NullType, NullType>  ContactSolver;

   //! Type of the handle to the active collision response solver.
   typedef boost::scoped_ptr< ContactSolver >  ContactSolverID;

   //! Type of the body manager for the OpenCL device.
   typedef OpenCLBodyManager<Config>  BodyManager;
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
   inline BatchGenerator& getBatchGenerator();
   inline ContactSolver&  getContactSolver();
   inline BodyManager&    getBodyManager();
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
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   inline void clear();

   inline void findContacts();
   inline void generateBatches();
   inline void resolveContacts();
   inline void clearContacts();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t numContacts_;         //!< The number of time steps in the last time step.
   Contacts contacts_;          //!< The currently active contacts of the simulation world.
   Batches batches_;            //!< Container for the generated batches.
   CoarseDetector detector_;    //!< The active collision detection algorithm.
   BatchGenerator generator_;   //!< The active batch generation algorithm.
   ContactSolverID solver_;     //!< The active collision response algorithm handle.
   BodyManager bodyManager_;    //!< The active OpenCL body manager.
   BS bodystorage_;             //!< The rigid body storage.
   PS processstorage_;          //!< The process storage.
   Domain domain_;              //!< The local process domain.
   AS attachablestorage_;       //!< The attachable storage.
   JS jointstorage_;            //!< The joint storage.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   template< typename C2, typename U1, typename U2 > friend class response::OpenCLSolver;
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
CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::CollisionSystem()
   : Singleton<CollisionSystem,logging::Logger>()  // Initialization of the Singleton base object
   , numContacts_( 0 )
   , contacts_( 5000 )
   , batches_()
   , detector_( bodystorage_ )
   , generator_()
   , solver_()
   , bodyManager_( bodystorage_ )
   , bodystorage_()
   , processstorage_()
   , domain_( processstorage_ )
   , attachablestorage_()
   , jointstorage_()
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
CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::~CollisionSystem()
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
inline size_t CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getNumContacts() const
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
inline typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::CoarseDetector&
   CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getCoarseDetector()
{
   return detector_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the currently active batch generator.
 *
 * \return Reference to the active batch generator.
 *
 * With this function it is possible to access the currently active batch generator. The
 * following code shows an example of how to use this function to acquire a reference to
 * the batch generator:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();
   pe::BatchGenerator& batchGenerator = collisionSystem->getBatchGenerator();
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
inline typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::BatchGenerator&
   CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getBatchGenerator()
{
   return generator_;
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
inline typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::ContactSolver&
   CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getContactSolver()
{
   if( !solver_ )
      solver_.reset( new ContactSolver( bodyManager_, bodystorage_ ) );
   return *solver_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the active OpenCL body manager.
 *
 * \return Reference to the active OpenCL body manager.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::BodyManager&
   CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getBodyManager()
{
   return bodyManager_;
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
inline const typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::BS&
   CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getBodyStorage() const
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
inline const typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::PS&
   CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getProcessStorage() const
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
inline const typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::AS&
   CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getAttachableStorage() const
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
inline const Domain& CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::getDomain() const
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
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::add( BodyID body )
{
   detector_.add( body );
   bodystorage_.add( body );
   bodyManager_.bodyAdded(body);    // Notifying the OpenCL body manager of the new rigid body
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
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::remove( BodyID body )
{
   detector_.remove( body );
   bodystorage_.remove( body );
   bodyManager_.bodyRemoved(body);    // Notifying the OpenCL body manager of the removal
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
inline typename CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::BodyIterator CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::remove( BodyIterator body )
{
   BodyID bodyid( *body );
   detector_.remove( bodyid );
   BodyIterator next( bodystorage_.remove( body ) );
   bodyManager_.bodyRemoved( bodyid );    // Notifying the OpenCL body manager of the removal
   return next;
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
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::removeFromCollisionDetector( BodyID body )
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
/*!\brief Default implementation of the CollisionSystem::simulationStep() function.
 *
 * \param timestep Size of the time step.
 * \return void
 *
 * The default implementation of the CollisionSystem::simulationStep() function fits most of
 * the collision response algorithms. The function consists of the following steps:
 *  -# Setting the TimeStep class members
 *  -# Applying the acting forces of all active force generators
 *  -# Collision detection
 *  -# Batch generation
 *  -# Collsion response
 *  -# Rigid body movements
 *  -# Update of all active visualization systems
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::simulationStep( real timestep )
{
   pe_USER_ASSERT( processstorage_.isEmpty(), "MPI parallelization not available for the selected solver" );

   const typename BS::Iterator bodyBegin( bodystorage_.begin() );
   const typename BS::Iterator bodyEnd  ( bodystorage_.end()   );

   pe_LOG_DEBUG_SECTION( log ) {
      // Checking the state of the rigid bodies
      log << "   State of the rigid bodies before the collision handling:\n";
      for( typename BS::ConstIterator b=bodyBegin; b!=bodyEnd; ++b )
         log << "     Body " << b->getID() << ": pos = " << b->getPosition()
             << " , v = " << b->getLinearVel() << " , w = " << b->getAngularVel()
             << " , f = " << b->getForce() << " , t = " << b->getTorque() << "\n";
   }

   // Finding all contacts
   findContacts();

   // Generating batches
   generateBatches();

   // Integrate body velocities
   for( typename BS::Iterator body=bodyBegin; body!=bodyEnd; ++body )
      body->integrateVelocity( timestep );

   // Synchronizing OpenCL data structures
   if( batches_.size() )
      synchronize();

   // Resolving all contacts
   resolveContacts();

   // Clearing the contacts
   clearContacts();

   pe_LOG_DEBUG_SECTION( log ) {
      // Checking the state of the rigid bodies
      log << "   State of the rigid bodies after the collision handling:\n";
      for( typename BS::ConstIterator b=bodyBegin; b!=bodyEnd; ++b )
         log << "     Body " << b->getID() << ": pos = " << b->getPosition()
             << " , v = " << b->getLinearVel() << " , w = " << b->getAngularVel()
             << " , f = " << b->getForce() << " , t = " << b->getTorque() << "\n";
   }

   // Integrate body positions
   for( typename BS::Iterator body=bodyBegin; body!=bodyEnd; ++body )
      body->integratePosition( timestep );
}
//*************************************************************************************************




//=================================================================================================
//
//  COMMUNICATION FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default implementation of the CollisionSystem::synchronize() function.
 *
 * \return void
 *
 * The default implementation of the CollisionSystem::synchronize() function does NOT provide
 * any MPI synchronization of the rigid bodies contained in the simulation world. It merely
 * exist due to the requirements of the World class.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::synchronize()
{
   bodyManager_.syncOpenCL();
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
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::clear()
{
   detector_.clear();

   for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); ++body )
      delete *body;
   bodystorage_.clear();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Contact generation between the rigid bodies of the simulation world.
 *
 * \return void
 *
 * This function detects all contacts between the rigid bodies of the simulation world. For this
 * task it uses the selected collision detection algorithm (see pe::pe_COLLISION_DETECTOR).
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::findContacts()
{
   detector_.findContacts( contacts_ );
   numContacts_ = contacts_.size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Batch generation from the current contacts of the simulation world.
 *
 * \return void
 *
 * This function generates independent batches from the current active contacts of the simulation
 * world. Each of these batches can be solved independently from the other batches. For this task
 * it uses the selected batch generation algorithm (see pe::pe_BATCH_GENERATION).
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::generateBatches()
{
   generator_.generateBatches( contacts_, batches_ );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Collision response calculations for the current contacts of the simulation world.
 *
 * \return void.
 *
 * This function calculates the collision responses for the currently active contacts of the
 * simulation world. For this task it uses the selected collision response algorithm (see
 * pe::pe_COLLSISION_SOLVER).
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::resolveContacts()
{
   if( !solver_ )
      solver_.reset( new ContactSolver( bodyManager_, bodystorage_ ) );
   for( size_t i=0; i<batches_.size(); ++i )
      solver_->resolveContacts( batches_[i] );
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
inline void CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> >::clearContacts()
{
   typename Contacts::Iterator       begin( contacts_.begin() );
   typename Contacts::Iterator const end  ( contacts_.end()   );

   for( ; begin!=end; ++begin )
      delete *begin;

   contacts_.clear();
   batches_.clear();
}
//*************************************************************************************************

#else

//*************************************************************************************************
/*!\brief Specialization of the collision system for the OpenCL solver.
 * \ingroup core
 *
 * This specialization of the CollisionSystem class template is used for the OpenCL collision
 * response algorithms. Since the software is currently compiled without
 * OpenCL support this collision system generates a compile time assertion.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystem< C<CD,FD,BG,response::OpenCLSolver> > : private NonCopyable
{
   typedef C<CD,FD,BG,response::OpenCLSolver> ThisConfig;
   pe_CONSTRAINT_MUST_NOT_BE_SAME_TYPE(ThisConfig, Config);
};
//*************************************************************************************************

#endif

} // namespace pe

#endif
