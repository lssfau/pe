//=================================================================================================
/*!
 *  \file pe/core/collisionsystem/Default.h
 *  \brief Specialization of the CollisionSystem class template for LCP-based collision response
 *         algorithms.
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

#ifndef _PE_CORE_COLLISIONSYSTEM_DEFAULT_H_
#define _PE_CORE_COLLISIONSYSTEM_DEFAULT_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <pe/core/attachable/Attachable.h>
#include <pe/core/attachable/AttachableStorage.h>
#include <pe/core/batches/BatchVector.h>
#include <pe/core/batches/Generators.h>
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
#include <pe/core/rigidbody/BodyStorage.h>
#include <pe/core/rigidbody/RigidBody.h>
#include <pe/core/RootSection.h>
#include <pe/core/SerialSection.h>
#include <pe/core/Types.h>
#include <pe/math/Solvers.h>
#include <pe/system/LCPConfig.h>
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
/*!\brief Specialization of the collision system for all LCP-based algorithms.
 * \ingroup core
 *
 * This specialization of the CollisionSystem class template is used for all LCP-based collision
 * response algorithms. It represents the default instantiation for the selected Configuration
 * pe::Config.
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
class CollisionSystem< C<CD,FD,BG,CR> >
   : public Singleton< CollisionSystem< C<CD,FD,BG,CR> >, logging::Logger >
{
public:
   //**Type definitions****************************************************************************
   typedef C<CD,FD,BG,CR>                       Config;             //!< Type of the configuration.

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
   typedef response::ContactSolver<Config>  ContactSolver;

   //! Type of the handle to the active collision response solver.
   typedef boost::scoped_ptr< ContactSolver >  ContactSolverID;

   //! Type of the initial collision response algorithm.
   /*! The type of the initial collision response algorithm is selected by the setting of
       the pe::pe_CONSTRAINT_SOLVER macro. */
   typedef CR<Config,response::lcp::ComplementaritySolver,NullType>  InitialContactSolver;
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
   inline ContactSolver&  getContactSolver()     const;
   inline const BS&       getBodyStorage()       const;
   inline const PS&       getProcessStorage()    const;
   inline const AS&       getAttachableStorage() const;
   inline const Domain&   getDomain()            const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   template< template<typename,typename,typename> class CS,
             typename CP >
   inline void setContactSolver() const;

   template< template<typename,typename,typename> class CS,
             typename CP, typename A1 >
   inline void setContactSolver( const A1& a1 ) const;

   template< template<typename,typename,typename> class CS,
             typename CP, typename A1, typename A2 >
   inline void setContactSolver( const A1& a1, const A2& a2 ) const;

   template< template<typename,typename,typename> class CS,
             typename CP, typename A1, typename A2, typename A3 >
   inline void setContactSolver( const A1& a1, const A2& a2, const A3& a3 ) const;

   template< template<typename,typename,typename> class CS,
             typename CP, typename A1, typename A2, typename A3, typename A4 >
   inline void setContactSolver( const A1& a1, const A2& a2, const A3& a3, const A4& a4 ) const;
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
   size_t numContacts_;        //!< The number of time steps in the last time step.
   Contacts contacts_;         //!< The currently active contacts of the simulation world.
   Batches batches_;           //!< Container for the generated batches.
   CoarseDetector detector_;   //!< The active collision detection algorithm.
   BatchGenerator generator_;  //!< The active batch generation algorithm.
   ContactSolverID solver_;    //!< The active collision response algorithm.
   BS bodystorage_;            //!< The rigid body storage.
   PS processstorage_;         //!< The process storage.
   Domain domain_;             //!< The local process domain.
   AS attachablestorage_;      //!< The attachable storage.
   JS jointstorage_;           //!< The joint storage.
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
CollisionSystem< C<CD,FD,BG,CR> >::CollisionSystem()
   : Singleton<CollisionSystem,logging::Logger>()       // Initialization of the Singleton base object
   , numContacts_      (    0 )                          // The number of time steps in the last time step
   , contacts_         ( 5000 )                          // The currently active contacts of the simulation world
   , batches_          ()                                // Container for the generated batches
   , detector_         ( bodystorage_ )                  // The active collision detection algorithm
   , generator_        ()                                // The active batch generation algorithm
   , solver_           ( new InitialContactSolver() )    // The active collision response algorithm
   , bodystorage_      ()
   , processstorage_   ()
   , domain_           ( processstorage_ )
   , attachablestorage_()
   , jointstorage_     ()
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
CollisionSystem< C<CD,FD,BG,CR> >::~CollisionSystem()
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline size_t CollisionSystem< C<CD,FD,BG,CR> >::getNumContacts() const
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,CR> >::CoarseDetector&
   CollisionSystem< C<CD,FD,BG,CR> >::getCoarseDetector()
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,CR> >::BatchGenerator&
   CollisionSystem< C<CD,FD,BG,CR> >::getBatchGenerator()
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,CR> >::ContactSolver&
   CollisionSystem< C<CD,FD,BG,CR> >::getContactSolver() const
{
   return *solver_;
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const typename CollisionSystem< C<CD,FD,BG,CR> >::BS&
   CollisionSystem< C<CD,FD,BG,CR> >::getBodyStorage() const
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const typename CollisionSystem< C<CD,FD,BG,CR> >::PS&
   CollisionSystem< C<CD,FD,BG,CR> >::getProcessStorage() const
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const typename CollisionSystem< C<CD,FD,BG,CR> >::AS&
   CollisionSystem< C<CD,FD,BG,CR> >::getAttachableStorage() const
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline const Domain& CollisionSystem< C<CD,FD,BG,CR> >::getDomain() const
{
   return domain_;
}
//*************************************************************************************************





//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Replaces the currently active contact solver.
 *
 * \return void
 *
 * With this function it is possible to replace the currently active LCP-based contact solver.
 * The function expects two template arguments that specify the type of the new contact solver
 * and the type of the complementarity solver.\n
 * The first template argument \a CS specifies the type of the new contact solver. Note that
 * it is only possible to select among the available LCP-based contact solvers. The attempt to
 * use any other solver results in a compile time error. The following contact solvers are
 * available:
 *
 *   - pe::response::FrictionlessSolver
 *   - pe::response::BoxFrictionSolver
 *   - pe::response::ConeFrictionSolver
 *   - pe::response::PolyhedralFrictionSolver
 *
 * The second template argument \a CP specifies the type of the applied complementarity solver.
 * The selection must be made from the following list of complementarity solvers:
 *
 *   - pe::solvers::Lemke
 *   - pe::solvers::PGS
 *   - pe::solvers::CPG
 *
 * In case the selected complementarity solver is not compatible with the selected contact solver
 * a \a CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED compile time error is created.\n
 * In order to pass arguments to the selected contact solver, up to four arguments can be passed
 * with the setContactSolver() function. The following code shows two examples of how to use this
 * function to set a new contact solver, once without any argument, once with a single argument:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();

   // Switching to a cone friction contact solver using a CPG complementarity solver
   collisionSystem->setContactSolver<pe::response::ConeFrictionSolver,pe::solvers::CPG>();

   // Switching to a box friction contact solver using a PGS complementarity solver. The passed
   // argument is directly relayed to the contact solver. In this case it specifies the number
   // of times the box friction problem is solved.
   collisionSystem->setContactSolver<pe::response::BoxFrictionSolver,pe::solvers::PGS>( 2 );
   \endcode
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
template< template<typename,typename,typename> class CS         // Type of the new LCP-based contact solver
        , typename CP >                                         // Type of the new complementarity solver
inline void CollisionSystem< C<CD,FD,BG,CR> >::setContactSolver() const
{
   typedef CS<Config,CP,NullType>  NewContactSolver;
   pe_CONSTRAINT_MUST_BE_BASE_OF( response::ContactSolver<Config>, NewContactSolver );
   solver_.reset( new NewContactSolver() );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Replaces the currently active contact solver.
 *
 * \param a1 The first argument for the new contact solver.
 * \return void
 *
 * With this function it is possible to replace the currently active LCP-based contact solver.
 * The function expects two template arguments that specify the type of the new contact solver
 * and the type of the complementarity solver.\n
 * The first template argument \a CS specifies the type of the new contact solver. Note that
 * it is only possible to select among the available LCP-based contact solvers. The attempt to
 * use any other solver results in a compile time error. The following contact solvers are
 * available:
 *
 *   - pe::response::FrictionlessSolver
 *   - pe::response::BoxFrictionSolver
 *   - pe::response::ConeFrictionSolver
 *   - pe::response::PolyhedralFrictionSolver
 *
 * The second template argument \a CP specifies the type of the applied complementarity solver.
 * The selection must be made from the following list of complementarity solvers:
 *
 *   - pe::solvers::Lemke
 *   - pe::solvers::PGS
 *   - pe::solvers::CPG
 *
 * In case the selected complementarity solver is not compatible with the selected contact solver
 * a \a CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED compile time error is created.\n
 * In order to pass arguments to the selected contact solver, up to four arguments can be passed
 * with the setContactSolver() function. The following code shows two examples of how to use this
 * function to set a new contact solver, once without any argument, once with a single argument:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();

   // Switching to a cone friction contact solver using a CPG complementarity solver
   collisionSystem->setContactSolver<pe::response::ConeFrictionSolver,pe::solvers::CPG>();

   // Switching to a box friction contact solver using a PGS complementarity solver. The passed
   // argument is directly relayed to the contact solver. In this case it specifies the number
   // of times the box friction problem is solved.
   collisionSystem->setContactSolver<pe::response::BoxFrictionSolver,pe::solvers::PGS>( 2 );
   \endcode
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
template< template<typename,typename,typename> class CS         // Type of the new LCP-based contact solver
        , typename CP                                           // Type of the new complementarity solver
        , typename A1 >                                         // Type of the first contact solver argument
inline void CollisionSystem< C<CD,FD,BG,CR> >::setContactSolver( const A1& a1 ) const
{
   typedef CS<Config,CP,NullType>  NewContactSolver;
   pe_CONSTRAINT_MUST_BE_BASE_OF( response::ContactSolver<Config>, NewContactSolver );
   solver_.reset( new NewContactSolver( a1 ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Replaces the currently active contact solver.
 *
 * \param a1 The first argument for the new contact solver.
 * \param a2 The second argument for the new contact solver.
 * \return void
 *
 * With this function it is possible to replace the currently active LCP-based contact solver.
 * The function expects two template arguments that specify the type of the new contact solver
 * and the type of the complementarity solver.\n
 * The first template argument \a CS specifies the type of the new contact solver. Note that
 * it is only possible to select among the available LCP-based contact solvers. The attempt to
 * use any other solver results in a compile time error. The following contact solvers are
 * available:
 *
 *   - pe::response::FrictionlessSolver
 *   - pe::response::BoxFrictionSolver
 *   - pe::response::ConeFrictionSolver
 *   - pe::response::PolyhedralFrictionSolver
 *
 * The second template argument \a CP specifies the type of the applied complementarity solver.
 * The selection must be made from the following list of complementarity solvers:
 *
 *   - pe::solvers::Lemke
 *   - pe::solvers::PGS
 *   - pe::solvers::CPG
 *
 * In case the selected complementarity solver is not compatible with the selected contact solver
 * a \a CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED compile time error is created.\n
 * In order to pass arguments to the selected contact solver, up to four arguments can be passed
 * with the setContactSolver() function. The following code shows two examples of how to use this
 * function to set a new contact solver, once without any argument, once with a single argument:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();

   // Switching to a cone friction contact solver using a CPG complementarity solver
   collisionSystem->setContactSolver<pe::response::ConeFrictionSolver,pe::solvers::CPG>();

   // Switching to a box friction contact solver using a PGS complementarity solver. The passed
   // argument is directly relayed to the contact solver. In this case it specifies the number
   // of times the box friction problem is solved.
   collisionSystem->setContactSolver<pe::response::BoxFrictionSolver,pe::solvers::PGS>( 2 );
   \endcode
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
template< template<typename,typename,typename> class CS         // Type of the new LCP-based contact solver
        , typename CP                                           // Type of the new complementarity solver
        , typename A1                                           // Type of the first contact solver argument
        , typename A2 >                                         // Type of the second contact solver argument
inline void CollisionSystem< C<CD,FD,BG,CR> >::setContactSolver( const A1& a1, const A2& a2 ) const
{
   typedef CS<Config,CP,NullType>  NewContactSolver;
   pe_CONSTRAINT_MUST_BE_BASE_OF( response::ContactSolver<Config>, NewContactSolver );
   solver_.reset( new NewContactSolver( a1, a2 ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Replaces the currently active contact solver.
 *
 * \param a1 The first argument for the new contact solver.
 * \param a2 The second argument for the new contact solver.
 * \param a3 The third argument for the new contact solver.
 * \return void
 *
 * With this function it is possible to replace the currently active LCP-based contact solver.
 * The function expects two template arguments that specify the type of the new contact solver
 * and the type of the complementarity solver.\n
 * The first template argument \a CS specifies the type of the new contact solver. Note that
 * it is only possible to select among the available LCP-based contact solvers. The attempt to
 * use any other solver results in a compile time error. The following contact solvers are
 * available:
 *
 *   - pe::response::FrictionlessSolver
 *   - pe::response::BoxFrictionSolver
 *   - pe::response::ConeFrictionSolver
 *   - pe::response::PolyhedralFrictionSolver
 *
 * The second template argument \a CP specifies the type of the applied complementarity solver.
 * The selection must be made from the following list of complementarity solvers:
 *
 *   - pe::solvers::Lemke
 *   - pe::solvers::PGS
 *   - pe::solvers::CPG
 *
 * In case the selected complementarity solver is not compatible with the selected contact solver
 * a \a CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED compile time error is created.\n
 * In order to pass arguments to the selected contact solver, up to four arguments can be passed
 * with the setContactSolver() function. The following code shows two examples of how to use this
 * function to set a new contact solver, once without any argument, once with a single argument:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();

   // Switching to a cone friction contact solver using a CPG complementarity solver
   collisionSystem->setContactSolver<pe::response::ConeFrictionSolver,pe::solvers::CPG>();

   // Switching to a box friction contact solver using a PGS complementarity solver. The passed
   // argument is directly relayed to the contact solver. In this case it specifies the number
   // of times the box friction problem is solved.
   collisionSystem->setContactSolver<pe::response::BoxFrictionSolver,pe::solvers::PGS>( 2 );
   \endcode
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
template< template<typename,typename,typename> class CS         // Type of the new LCP-based contact solver
        , typename CP                                           // Type of the new complementarity solver
        , typename A1                                           // Type of the first contact solver argument
        , typename A2                                           // Type of the second contact solver argument
        , typename A3 >                                         // Type of the third contact solver argument
inline void CollisionSystem< C<CD,FD,BG,CR> >::setContactSolver( const A1& a1, const A2& a2, const A3& a3 ) const
{
   typedef CS<Config,CP,NullType>  NewContactSolver;
   pe_CONSTRAINT_MUST_BE_BASE_OF( response::ContactSolver<Config>, NewContactSolver );
   solver_.reset( new NewContactSolver( a1, a2, a3 ) );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Replaces the currently active contact solver.
 *
 * \param a1 The first argument for the new contact solver.
 * \param a2 The second argument for the new contact solver.
 * \param a3 The third argument for the new contact solver.
 * \param a4 The fourth argument for the new contact solver.
 * \return void
 *
 * With this function it is possible to replace the currently active LCP-based contact solver.
 * The function expects two template arguments that specify the type of the new contact solver
 * and the type of the complementarity solver.\n
 * The first template argument \a CS specifies the type of the new contact solver. Note that
 * it is only possible to select among the available LCP-based contact solvers. The attempt to
 * use any other solver results in a compile time error. The following contact solvers are
 * available:
 *
 *   - pe::response::FrictionlessSolver
 *   - pe::response::BoxFrictionSolver
 *   - pe::response::ConeFrictionSolver
 *   - pe::response::PolyhedralFrictionSolver
 *
 * The second template argument \a CP specifies the type of the applied complementarity solver.
 * The selection must be made from the following list of complementarity solvers:
 *
 *   - pe::solvers::Lemke
 *   - pe::solvers::PGS
 *   - pe::solvers::CPG
 *
 * In case the selected complementarity solver is not compatible with the selected contact solver
 * a \a CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION_FAILED compile time error is created.\n
 * In order to pass arguments to the selected contact solver, up to four arguments can be passed
 * with the setContactSolver() function. The following code shows two examples of how to use this
 * function to set a new contact solver, once without any argument, once with a single argument:

   \code
   pe::CollisionSystemID collisionSystem = pe::theCollisionSystem();

   // Switching to a cone friction contact solver using a CPG complementarity solver
   collisionSystem->setContactSolver<pe::response::ConeFrictionSolver,pe::solvers::CPG>();

   // Switching to a box friction contact solver using a PGS complementarity solver. The passed
   // argument is directly relayed to the contact solver. In this case it specifies the number
   // of times the box friction problem is solved.
   collisionSystem->setContactSolver<pe::response::BoxFrictionSolver,pe::solvers::PGS>( 2 );
   \endcode
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
template< template<typename,typename,typename> class CS         // Type of the new LCP-based contact solver
        , typename CP                                           // Type of the new complementarity solver
        , typename A1                                           // Type of the first contact solver argument
        , typename A2                                           // Type of the second contact solver argument
        , typename A3                                           // Type of the third contact solver argument
        , typename A4 >                                         // Type of the fourth contact solver argument
inline void CollisionSystem< C<CD,FD,BG,CR> >::setContactSolver( const A1& a1, const A2& a2, const A3& a3, const A4& a4 ) const
{
   typedef CS<Config,CP,NullType>  NewContactSolver;
   pe_CONSTRAINT_MUST_BE_BASE_OF( response::ContactSolver<Config>, NewContactSolver );
   solver_.reset( new NewContactSolver( a1, a2, a3, a4 ) );
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::add( BodyID body )
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::remove( BodyID body )
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline typename CollisionSystem< C<CD,FD,BG,CR> >::BodyIterator CollisionSystem< C<CD,FD,BG,CR> >::remove( BodyIterator body )
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::removeFromCollisionDetector( BodyID body )
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
 *  -# Collision response
 *  -# Rigid body movements
 *  -# Update of all active visualization systems
 */
template< template<typename> class CD                           // Type of the coarse collision detection algorithm
        , typename FD                                           // Type of the fine collision detection algorithm
        , template<typename> class BG                           // Type of the batch generation algorithm
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,CR> >::simulationStep( real timestep )
{
   pe_USER_ASSERT( processstorage_.isEmpty(), "MPI parallelization not available for the selected solver" );

   const typename BS::Iterator bodyBegin( bodystorage_.begin() );
   const typename BS::Iterator bodyEnd  ( bodystorage_.end()   );

   // Logging the state of the rigid bodies
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   State of the rigid bodies before the collision handling:\n";
      for( typename BS::ConstIterator b=bodyBegin; b!=bodyEnd; ++b )
         log << "      Body " << b->getID() << ": pos = " << b->getPosition()
             << " , v = " << b->getLinearVel() << " , w = " << b->getAngularVel()
             << " , f = " << b->getForce() << " , t = " << b->getTorque() << "\n";
   }

   // Finding all contacts
   findContacts();

   // Generating batches
   generateBatches();

   // Resolving all contacts
   resolveContacts();

   // Clearing the contacts
   clearContacts();

   // Logging the state of the rigid bodies
   pe_LOG_DEBUG_SECTION( log ) {
      log << "   State of the rigid bodies after the collision handling:\n";
      for( typename BS::ConstIterator b=bodyBegin; b!=bodyEnd; ++b )
         log << "      Body " << b->getID() << ": pos = " << b->getPosition()
             << " , v = " << b->getLinearVel() << " , w = " << b->getAngularVel()
             << " , f = " << b->getForce() << " , t = " << b->getTorque() << "\n";
   }

   // Moving all rigid bodies
   for( typename BS::Iterator body=bodyBegin; body!=bodyEnd; ++body ) {
      body->move( timestep );
   }
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
void CollisionSystem< C<CD,FD,BG,CR> >::synchronize()
{}
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::clear()
{
   detector_.clear();

   for( BodyIterator body = bodystorage_.begin(); body != bodystorage_.end(); )
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::findContacts()
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::generateBatches()
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::resolveContacts()
{
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
        , template<typename,typename,typename> class CR         // Type of the collision response algorithm
        , template< template<typename> class                    // Template signature of the coarse collision detection algorithm
                  , typename                                    // Template signature of the fine collision detection algorithm
                  , template<typename> class                    // Template signature of the batch generation algorithm
                  , template<typename,typename,typename> class  // Template signature of the collision response algorithm
                  > class C >                                   // Type of the configuration
inline void CollisionSystem< C<CD,FD,BG,CR> >::clearContacts()
{
   typename Contacts::Iterator       begin( contacts_.begin() );
   typename Contacts::Iterator const end  ( contacts_.end()   );

   for( ; begin!=end; ++begin )
      delete *begin;

   contacts_.clear();
   batches_.clear();
}
//*************************************************************************************************

} // namespace pe

#endif
