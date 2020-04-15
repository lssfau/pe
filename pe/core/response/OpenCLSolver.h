//=================================================================================================
/*!
 *  \file pe/core/response/OpenCLSolver.h
 *  \brief Contact solver for solving cone frictional multibody systems on an OpenCL device.
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

#ifndef _PE_CORE_RESPONSE_OPENCLSOLVER_H_
#define _PE_CORE_RESPONSE_OPENCLSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <algorithm>
#include <iostream>
#include <pe/core/CollisionSystem.h>
#include <pe/core/contact/ContactVector.h>
#include <pe/core/OpenCLBodyManager.h>
#include <pe/core/OpenCLColoredContacts.h>
#include <pe/core/OpenCLContacts.h>
#include <pe/core/OpenCLUpdateCache.h>
#include <pe/core/response/constraints/SolverRestriction.h>
#include <pe/core/response/ConstructorDriver.h>
#include <pe/core/response/ContactGraphColoring.h>
#include <pe/core/response/ContactSolver.h>
#include <pe/core/response/FrictionConstructor.h>
#include <pe/core/TimeStep.h>
#include <pe/math/problems/ContactLCP.h>
#include <pe/math/Vector3.h>
#include <pe/system/Collisions.h>
#include <pe/system/LCPConfig.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NullType.h>
#include <pe/util/TypeList.h>
#include <pe/util/Types.h>
#include <string>

#include <gpusolve.hpp>


//*************************************************************************************************
// Macro definitions
//*************************************************************************************************

#define PE_STRINGIZE(x) #x


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact solver for solving cone frictional multibody systems on an OpenCL device.
 * \ingroup collision_response
 *
 * The contact solver uses a matrix-free projected multi-color Gauss-Seidel for resolving the
 * contacts.
 */
template< typename C              // Type of the configuration
        , typename U1=NullType    // First unused auxiliary template parameter
        , typename U2=NullType >  // Second unused auxiliary template parameter
class OpenCLSolver : public ContactSolver<C>
{
private:
   //**Type definitions****************************************************************************
   typedef C                                Config;            //!< Type of the configuration.
   typedef OpenCLSolver<C,U1,U2>            This;              //!< Type of this OpenCLSolver instance.
   typedef typename Config::ContactType     ContactType;       //!< Type of the contacts.
   typedef typename Config::ContactID       ContactID;         //!< Handle to a contact.
   typedef typename Config::ConstContactID  ConstContactID;    //!< Handle to a contact.
   typedef ContactVector<ContactType>       Contacts;          //!< Type of the contact container.
   typedef BodyStorage<Config>              BS;                //!< Type of the body storage.
   typedef std::pair<real, size_t>          SolverInformation; //!< The convergence criterion and number of iterations.

   //! Type of the body manager for the OpenCL device.
   typedef OpenCLBodyManager<C>             BodyManager;
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit OpenCLSolver( BodyManager& bodyManager, BS& bodystorage );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~OpenCLSolver();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   virtual size_t getMaxIterations()       const;
   virtual size_t getLastIterations()      const;
   virtual real   getLastPrecision()       const;
   virtual real   getThreshold()           const;
           real   getRelaxationParameter() const;
           size_t getColorLimit()          const;
   //@}
   //**********************************************************************************************

   //**Set functions***************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setMaxIterations      ( size_t maxIterations );
   virtual void setThreshold          ( real   threshold );
           void setRelaxationParameter( real   omega );
           void setColorLimit         ( size_t colorLimit );
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   virtual void resolveContacts( const Contacts& contacts );
   //@}
   //**********************************************************************************************

protected:
   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   void initDeviceMemory();
   bool solve( const std::vector<size_t> groupSizes );
   //@}
   //**********************************************************************************************

private:
   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U1, NullType );
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U2, NullType );
   /*! \endcond */
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   size_t maxIterations_;     //!< The maximum number of iterations.
                              /*!< This is the maximum number of iterations the solver will spend
                                   for solving the given problem. */
   size_t lastIterations_;    //!< The number of iterations spent in the last solution process.
   real   lastPrecision_;     //!< The precision of the solution after the solution process.
   real   threshold_;         //!< Precision threshold for the solution.
   real   omega_;             //!< Relaxation parameter.
   size_t colorLimit_;        //!< The maximum number of colors.
   static bool initialized_;  /*!< Flag indicating if the OpenCL sources were already loaded and
                                   compiled. */
   OpenCLContacts        contacts_;         //!< Contact storage for the OpenCL solver.
   OpenCLUpdateCache     cache_;            //!< Cache for velocity updates reduction.
   OpenCLColoredContacts coloredContacts_;  //!< Auxiliary data structure for reordering contacts.
   BodyManager&          bodyManager_;      //!< Reference to the OpenCL body manager.
   OpenCLBodyProperties& bodyProperties_;   //!< Reference to the OpenCL body properties.
   BS&                   bodystorage_;      //!< Reference to the central body storage.
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************


template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
bool OpenCLSolver<C, U1, U2>::initialized_ = false;


//=================================================================================================
//
//  CONSTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Default constructor for the OpenCLSolver class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
OpenCLSolver<C,U1,U2>::OpenCLSolver( BodyManager& bodyManager, BS& bodystorage )
   : ContactSolver<C>()                                   // Initialization of the base class
   , maxIterations_ ( response::lcp::maxIterations )      // The maximum number of iterations
   , lastIterations_( 0 )                                 // The number of iterations spent in the last solution process
   , lastPrecision_ ( std::numeric_limits<real>::max() )  // The precision of the solution after the solution process
   , threshold_     ( response::lcp::threshold )          // Precision threshold for the solution
   , omega_         ( 0.3 )                               // Relaxation parameter
   , colorLimit_    ( 4 )                                 // Color limit
   , bodyManager_   ( bodyManager )                       // Reference to the OpenCL body manager.
   , bodyProperties_( bodyManager.getBodyProperties() )   // Reference to the OpenCL body properties.
   , bodystorage_   ( bodystorage )
{
}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the OpenCLSolver class.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
OpenCLSolver<C,U1,U2>::~OpenCLSolver()
{}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Initializes OpenCL device memory data structures.
 *
 * \return void
 *
 * Among other things body centric data is copied to the contact storage. For that purpose the
 * InitContactsMassInertia() and InitContactsCoordinates() kernels are called.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void OpenCLSolver<C,U1,U2>::initDeviceMemory() {
   const size_t blockSize = 128;
   const size_t workItems = gpusolve::util::math::ceil<blockSize>(contacts_.size());

   gpusolve::opencl::KernelCall kernelCallFirstHalf("InitContactsMassInertia");
   kernelCallFirstHalf.setup(cl::NDRange(workItems), cl::NDRange(blockSize));

   uint32_t contactsSize = contacts_.size();
   uint32_t contactsSizeAllocated = contacts_.sizeAllocated();
   uint32_t bodiesSizeAllocated = bodyProperties_.sizeAllocated();

   kernelCallFirstHalf(contacts_.bodyInvMass_.data()(),
      contacts_.bodyInvInertia_.data()(), bodyProperties_.invMass_.data()(), bodyProperties_.invI_.data()(),
      bodyProperties_.mutex_.data()(),
      contacts_.bodies_.data()(), contacts_.status_.data()(), contactsSize, contactsSizeAllocated,
      bodiesSizeAllocated);

   gpusolve::opencl::KernelCall kernelCallSecondHalf("InitContactsCoordinates");
   kernelCallSecondHalf.setup(cl::NDRange(workItems), cl::NDRange(blockSize));

   kernelCallSecondHalf(contacts_.bodyGPos_.data()(), contacts_.bodyLinVel_.data()(),
      contacts_.bodyAngVel_.data()(), bodyProperties_.gpos_.data()(), bodyProperties_.v_.data()(),
      bodyProperties_.w_.data()(), contacts_.bodies_.data()(), contacts_.status_.data()(),
      contactsSize, contactsSizeAllocated);
}




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the maximum number of iterations the solver may spend solving the problem.
 *
 * \return The maximum number of iterations spent in the solver.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
size_t OpenCLSolver<C,U1,U2>::getMaxIterations() const
{
   return maxIterations_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of iterations spent in the last solution process.
 *
 * \return The number of iterations spent in the last solution process.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
size_t OpenCLSolver<C,U1,U2>::getLastIterations() const
{
   return lastIterations_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the precision of the solution after the solution process.
 *
 * \return The precision of the solution after the solution process.
 *
 * The solver is not enforced to compute the precision after the solution. Instead it can just
 * report infinity as the last precision.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
real OpenCLSolver<C,U1,U2>::getLastPrecision() const
{
   return lastPrecision_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the threshold that classifies a solution as good enough.
 *
 * \return The threshold for the solution quality.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
real OpenCLSolver<C,U1,U2>::getThreshold() const
{
   return threshold_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the relaxation parameter used in the solution process.
 *
 * \return The relaxation parameter.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
real OpenCLSolver<C,U1,U2>::getRelaxationParameter() const
{
   return omega_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the maximum number of colors used in the coloring of the data dependency graph.
 *
 * \return The maximum number of colors.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
size_t OpenCLSolver<C,U1,U2>::getColorLimit() const
{
   return colorLimit_;
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets the maximum number of iterations the solver may spend solving the problem.
 *
 * \param maxIterations The maximum number of iterations spent in the solver.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void OpenCLSolver<C,U1,U2>::setMaxIterations( size_t maxIterations )
{
   maxIterations_ = maxIterations;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the threshold which classifies a solution as good enough.
 *
 * \param threshold The threshold for the solution quality.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void OpenCLSolver<C,U1,U2>::setThreshold( real threshold )
{
   threshold_ = threshold;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the relaxation parameter used in the solution process.
 *
 * \param omega The relaxation parameter.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void OpenCLSolver<C,U1,U2>::setRelaxationParameter( real omega )
{
   pe_USER_ASSERT( omega > 0, "The relaxation parameter must be positive." );
   omega_ = omega;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the maximum number of colors used in the coloring of the data dependency graph.
 *
 * \param colorLimit The maximum number of colors.
 * \return void
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void OpenCLSolver<C,U1,U2>::setColorLimit( size_t colorLimit )
{
   pe_USER_ASSERT( colorLimit >= 1 && colorLimit <= 32, "The number of colors must not exceed 32 and must be at least 1." );
   colorLimit_ = colorLimit;
}
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Resolves the given contacts.
 *
 * \param contacts The set of potential contacts.
 * \return void
 *
 * Performs a coloring of the contact graph and prepares the corresponding OpenCL data structures.
 * Synchronizes contact data to the OpenCL device. The PGS solver is executed and the contact
 * reactions are synchronized back to the host.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
void OpenCLSolver<C,U1,U2>::resolveContacts( const Contacts& contacts )
{
   const real   dt   ( TimeStep::size() );
   const typename BS::Iterator bodyBegin( bodystorage_.begin() );
   const typename BS::Iterator bodyEnd  ( bodystorage_.end()   );

   if( contacts.isEmpty() || !contacts.isActive() ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Skipping inactive/empty batch \n";
      }
      return;
   }

   // load and compile OpenCL kernels needed for OpenCLSolver
   if( !initialized_ ) {
      initialized_ = true;

      std::string kernelPath( PE_STRINGIZE( PE_OPENCL_KERNEL_PATH ) );
      gpusolve::opencl::util::CompilerDriver::loadAndCompile(kernelPath, ".cl", true);
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Resolving the " << contacts.size() << " cone friction contact(s)"
                << " (";
      for( typename Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c )
         log << " " << c->getID();
      log << " )...\n";
   }

   pe_INTERNAL_ASSERT( colorLimit_ <= 32, "The maximum number of support colors is 32." );

   //const std::vector<size_t> groups( color( contacts ) );
   const std::vector<size_t> groups( colorBalanced( contacts, colorLimit_ ) );
   Contacts contactsReordered( contacts );

   const std::vector<size_t> groupSizes( reorderContacts( contactsReordered, groups ) );
   const size_t numGroups( groupSizes.size() );

   pe_LOG_DEBUG_SECTION( log ) {
      for( size_t i = 0; i < numGroups; ++i ) {
         log << "Color " << i << " has " << groupSizes[i] << " contacts.\n";
      }
   }

   // contacts from 0 to groupSizes[0]-1 have color 0
   // contacts from groupSizes[0] to groupSizes[0]+groupSizes[1]-1 have color 1
   // ...

   // set up cache for velocity updates
   coloredContacts_.init(groupSizes.size(), bodyProperties_.size());

   bodyProperties_.contactsHostAccess(true);
   bodyProperties_.contacts_.fill(0);

   size_t index = 0; 
   for(size_t g = 0; g < groupSizes.size(); ++g) {
      for(size_t gi = 0; gi < groupSizes[g]; ++gi, ++index) {
         ContactID c = contactsReordered[index];

         if(!c->getBody1()->isFixed()) {
            size_t bodyA = bodyManager_.getBodyIndex(c->getBody1());
            ++coloredContacts_.involved(bodyA, g);
            bodyProperties_.contacts_[bodyA] |= 0x1 << g;
         }
         if(!c->getBody2()->isFixed()) {
            size_t bodyB = bodyManager_.getBodyIndex(c->getBody2());
            ++coloredContacts_.involved(bodyB, g);
            bodyProperties_.contacts_[bodyB] |= 0x1 << g;
         }
      }
   }

   bodyProperties_.contactsHostAccess(false);

   // update internals of OpenCLColoredContacts
   coloredContacts_.calcOffsets();

   // offsets into cache
   uint32_t offset = 0;

   offset = 0;
   contacts_.offsets_.resize(groupSizes.size() + 1);
   for(size_t i = 0; i < groupSizes.size(); ++i) {
      contacts_.offsets_[i] = offset;
      offset += gpusolve::util::math::ceil<16>(groupSizes[i]);
   }
   contacts_.offsets_[groupSizes.size()] = offset;

   // resize OpenCL data structures for current number of contacts
   contacts_.resize(offset);
   cache_.resize(coloredContacts_.maxSize());

   contacts_.hostAccess(true);

   // sync to OpenCL
   index = 0;
   offset = 0;

   for(size_t g = 0; g < groupSizes.size(); ++g) {
      offset = contacts_.offsets_[g];
      size_t size = contacts_.sizeAllocated();
      size_t gi = 0;
      for(; gi < groupSizes[g]; ++gi, ++index, ++offset) {
         ContactID c = contactsReordered[index];

         contacts_.status_[offset] = 1;

         // normals
         const Vec3& n = c->getNormal();
         contacts_.normal_[offset][0] = n[0];
         contacts_.normal_[offset][1] = n[1];
         contacts_.normal_[offset][2] = n[2];
         contacts_.normal_[offset][3] = 0;

         const Vec3& t = c->getTangentX();
         contacts_.normal_[offset + size][0] = t[0];
         contacts_.normal_[offset + size][1] = t[1];
         contacts_.normal_[offset + size][2] = t[2];
         contacts_.normal_[offset + size][3] = 0;

         const Vec3& o = c->getTangentY();
         contacts_.normal_[offset + 2 * size][0] = o[0];
         contacts_.normal_[offset + 2 * size][1] = o[1];
         contacts_.normal_[offset + 2 * size][2] = o[2];
         contacts_.normal_[offset + 2 * size][3] = 0;

         // involved bodies
         size_t bodyA = bodyManager_.getBodyIndex(c->getBody1());
         size_t bodyB = bodyManager_.getBodyIndex(c->getBody2());

         contacts_.bodies_[offset][0] = bodyA;
         contacts_.bodies_[offset][1] = bodyB;

         // masses
         const real invMassA = c->getBody1()->getInvMass();
         const real invMassB = c->getBody2()->getInvMass();

         // rA, rB
         const Vec3& rA = c->getPosition() - c->getBody1()->getPosition();
         const Vec3& rB = c->getPosition() - c->getBody2()->getPosition();

         contacts_.r_[offset][0] = rA[0];
         contacts_.r_[offset][1] = rA[1];
         contacts_.r_[offset][2] = rA[2];
         contacts_.r_[offset][3] = 0;

         contacts_.r_[offset + size][0] = rB[0];
         contacts_.r_[offset + size][1] = rB[1];
         contacts_.r_[offset + size][2] = rB[2];
         contacts_.r_[offset + size][3] = 0;

         // diagonal block
         Mat3 Q(n, t, o);
         const Mat3 mA = Mat3(invMassA, invMassA, invMassA) - rA % c->getBody1()->getInvInertia() %
            rA;

         const Mat3 mB = Mat3(invMassB, invMassB, invMassB) - rB % c->getBody2()->getInvInertia() %
            rB;

         Mat3 diag = (trans(Q) * (mA + mB) * Q) * Mat3(dt, dt, dt);
         diag = diag.getInverse();

         for(size_t i = 0; i < 3; ++i) {
            contacts_.diagonalBlock_[offset + i * size][0] = diag(i, 0);
            contacts_.diagonalBlock_[offset + i * size][1] = diag(i, 1);
            contacts_.diagonalBlock_[offset + i * size][2] = diag(i, 2);
            contacts_.diagonalBlock_[offset + i * size][3] = 0;
         }

         // p (unknown)
         contacts_.p_[offset][0] = 0;
         contacts_.p_[offset][1] = 0;
         contacts_.p_[offset][2] = 0;
         contacts_.p_[offset][3] = 0;

         // friction coefficient
         contacts_.mu_[offset] = c->getFriction();

         // cache management
         if(!c->getBody1()->isFixed()) {
            uint32_t offA = coloredContacts_.offset(bodyA, g)++;
            contacts_.reductionInfo_[offset][0] = offA;
            contacts_.reductionInfo_[offset][2] = offA -
               coloredContacts_.base(bodyA, g);
         } else {
            contacts_.reductionInfo_[offset][0] = -1;
         }

         if(!c->getBody2()->isFixed()) {
            uint32_t offB = coloredContacts_.offset(bodyB, g)++;
            contacts_.reductionInfo_[offset][1] = offB;
            contacts_.reductionInfo_[offset][3] = offB -
               coloredContacts_.base(bodyB, g);
         } else {
            contacts_.reductionInfo_[offset][1] = -1;
         }
      }

      for(; offset < contacts_.offsets_[g + 1]; ++offset)
         contacts_.status_[offset] = 0;
   }

   contacts_.hostAccess(false);

   // set up the solver
   solve( groupSizes );

   bodyManager_.syncHost();
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Solves the contact problem.
 *
 * \param groupSizes The number of contacts per color.
 * \return Whether the desired convergence criterion was fulfilled.
 *
 * Executes a matrix-free multi-color Gauss-Seidel on the OpenCL device. One iteration consists of
 * separate sweeps over each color. Each sweep starts by recomputing all predicted gaps at the end
 * of the time step taking into account all contact reactions due to previous sweeps but not due
 * to the current sweep of the current color. This is accomplished in the CalcPhi() kernel call.
 * This information is fed into the CalcP() kernel, which computes new contact reactions or
 * rather contact reaction changes for all contacts of the current color. These contact reaction
 * changes are translated into linear and angular velocity changes of the involved bodies and
 * stored in the contact cache by the CalcVelocities() kernel. There they are grouped by bodies
 * and thus can be reduced by a segmented reduction in the ReduceVelocities() kernel. The
 * total velocity changes of each body in the current sweep is then applied to the body centric
 * data structures in the UpdateVelocities() kernel.
 */
template< typename C     // Type of the configuration
        , typename U1    // First unused auxiliary template parameter
        , typename U2 >  // Second unused auxiliary template parameter
bool OpenCLSolver<C,U1,U2>::solve( const std::vector<size_t> groupSizes )
{
   bool converged( false );

   // initialize data structures on device
   initDeviceMemory();

   const size_t blockSize = 128;
   const size_t maxBlocks = 128;
   const size_t maxWorkItems = blockSize * maxBlocks;

   const size_t colorCount = groupSizes.size();
   const uint32_t contactsSizeAllocated = contacts_.sizeAllocated();
   const uint32_t bodiesSize = bodyProperties_.size();
   const uint32_t bodiesSizePadded = gpusolve::util::math::ceil<32>(bodiesSize);

   const float dt = TimeStep::size();

   // kernels
   gpusolve::opencl::KernelCall CalcPhi("CalcPhi");
   gpusolve::opencl::KernelCall CalcP("CalcP");
   gpusolve::opencl::KernelCall CalcVelocities("CalcVelocities");
//   gpusolve::opencl::KernelCall CalcVelocities("CalcVelocitiesAtomic");
   gpusolve::opencl::KernelCall ReduceVelocities("ReduceVelocities");
   gpusolve::opencl::KernelCall UpdateVelocities("UpdateVelocities");
   gpusolve::opencl::KernelCall ReduceResidual("ReduceResidual");
   gpusolve::opencl::KernelCall ReduceMaxVelocity("ReduceMaxVelocity");

   double timeCalcPhi = 0;
   double timeCalcP = 0;
   double timeCalcVelocities = 0;
   double timeReduceVelocities = 0;
   double timeUpdateVelocities = 0;
   double timeReduceResidual = 0;
   double timeReduceMaxVelocity = 0;

   double calcPhiOps = 0;
   double calcPOps = 0;
   double calcVelocitiesOps = 0;

   size_t residualSpacing = 25;
   real residual = 0;

   bool computeMaxVelocity = false;

   size_t it = 1;
   for(; !converged && it < maxIterations_; ++it) {
      uint32_t reductionOffset = 0;

      uint32_t updateResidual = 0;
      if(!(it % residualSpacing) || it == maxIterations_ - 1)
         updateResidual = 1;

      for(uint32_t c = 0; c < colorCount; ++c) {
         uint32_t size = groupSizes[c];
         size_t workItems = min(gpusolve::util::math::ceil<blockSize>(size), maxWorkItems);

         // set up kernels
         CalcPhi.setup(cl::NDRange(workItems), cl::NDRange(blockSize));
         CalcP.setup(cl::NDRange(workItems), cl::NDRange(blockSize));
         CalcVelocities.setup(cl::NDRange(workItems), cl::NDRange(blockSize));

         // determine values of phi for current time step
         timeCalcPhi += CalcPhi(contacts_.phi_.data()(), contacts_.r_.data()(),
            contacts_.bodyGPos_.data()(), bodyProperties_.v_.data()(), bodyProperties_.w_.data()(), 
            contacts_.bodies_.data()(), size, contacts_.offsets_[c], contactsSizeAllocated, dt);

         calcPhiOps += size;

         // find new approximation to resolve collisions (1. step)
         timeCalcP += CalcP(contacts_.normal_.data()(), contacts_.diagonalBlock_.data()(),
            contacts_.phi_.data()(), contacts_.p_.data()(), contacts_.mu_.data()(), size,
            contacts_.offsets_[c], contactsSizeAllocated, updateResidual);

         calcPOps += size;

         // find new approximation to resolve collisions (2. step)
         timeCalcVelocities += CalcVelocities(contacts_.normal_.data()(),
            contacts_.phi_.data()(), contacts_.bodyInvMass_.data()(),
            contacts_.bodyInvInertia_.data()(), contacts_.reductionInfo_.data()(),
            contacts_.r_.data()(), cache_.count_.data()(), cache_.v_.data()(),
            cache_.w_.data()(), (cl_uint)size, (cl_uint)contacts_.offsets_[c], (cl_uint)contactsSizeAllocated, (cl_float)omega_);

         calcVelocitiesOps += size;

/*
         timeCalcVelocities += CalcVelocities(contacts_.normal_.data()(),
            contacts_.phi_.data()(), contacts_.bodyInvMass_.data()(),
            contacts_.bodyInvInertia_.data()(), contacts_.r_.data()(),
            contacts_.bodies_.data()(), bodyProperties_.v_.data()(), bodyProperties_.w_.data()(),
            bodyProperties_.mutex_.data()(), sizes);
 */
         uint32_t cacheSize = coloredContacts_.colorSize(c);
         pe_INTERNAL_ASSERT( cacheSize <= 2 * size, "Each contact can at most require two cache update entries." );
         workItems = min(gpusolve::util::math::ceil<blockSize>(cacheSize), maxWorkItems);

         ReduceVelocities.setup(cl::NDRange(workItems), cl::NDRange(blockSize));
         for(uint32_t s = 1; s <= coloredContacts_.reductionSteps(c); ++s) {
            timeReduceVelocities += ReduceVelocities(cache_.count_.data()(), cache_.v_.data()(),
               cache_.w_.data()(), cacheSize, s);
         }

         workItems = min(gpusolve::util::math::ceil<blockSize>(size), maxWorkItems);

         UpdateVelocities.setup(cl::NDRange(workItems), cl::NDRange(blockSize));

         timeUpdateVelocities += UpdateVelocities(coloredContacts_.bases().data()(),
            cache_.v_.data()(), cache_.w_.data()(), bodyProperties_.v_.data()(), bodyProperties_.w_.data()(),
            bodyProperties_.contacts_.data()(), bodiesSize, c, bodiesSizePadded * c);

         // reduce residual
         if(updateResidual) {
            size = groupSizes[c];
            workItems = min(gpusolve::util::math::ceil<blockSize>(size), maxWorkItems);

            ReduceResidual.setup(cl::NDRange(workItems), cl::NDRange(blockSize));

            timeReduceResidual += ReduceResidual(contacts_.p_.data()(),
            contacts_.residual_.data()(), cl::__local(sizeof(cl_float) * blockSize), size,
            reductionOffset);

            reductionOffset += workItems / blockSize;
         }
      }

      // compute maximum velocity (for debugging purposes)
      if(computeMaxVelocity && updateResidual) {
         size_t workItems = min(gpusolve::util::math::ceil<blockSize>(bodiesSize), maxWorkItems);

         ReduceMaxVelocity.setup(cl::NDRange(workItems), cl::NDRange(blockSize));

         timeReduceMaxVelocity += ReduceMaxVelocity(bodyProperties_.v_.data()(), bodyProperties_.status_.data()(),
            bodyProperties_.maxVelocity_.data()(), cl::__local(sizeof(cl_float) * blockSize), bodiesSize);

         bodyProperties_.maxVelocity_.enableAccess<gpusolve::memoryaccess::host>();
         real maxVel = 0;
         for(size_t i = 0; i < workItems / blockSize; ++i)
            maxVel = max(maxVel, bodyProperties_.maxVelocity_[i]);

         bodyProperties_.maxVelocity_.disableAccess<gpusolve::memoryaccess::host>();

         std::cerr << "maximum lin. velocity after iteration " << it << ": " << maxVel << std::endl;
      }

      // check residual
      if(updateResidual) {
         contacts_.residual_.enableAccess<gpusolve::memoryaccess::host>();

         residual = 0;
         for(size_t i = 0; i < reductionOffset; ++i)
            residual = max(residual, contacts_.residual_[i]);

         contacts_.residual_.disableAccess<gpusolve::memoryaccess::host>();

         if(residual < threshold_)
            converged = true;
      }
   }

#if OPENCLSOLVER_TIMEMEASURE
   std::cerr << "CalcPhi: " << (timeCalcPhi / it) << " s" << std::endl;
   std::cerr << "\t" << ((calcPhiOps * 62) / timeCalcPhi) * 1e-9 << " GFlops" << std::endl;
   std::cerr << "\t" << gpusolve::util::MemorySize(calcPhiOps * sizeof(float) * 38) / timeCalcPhi
      << "/s" << std::endl;

   std::cerr << "CalcP: " << (timeCalcP / it) << " s" << std::endl;
   std::cerr << "\t" << ((calcPOps * 53) / timeCalcP) * 1e-9 << " GFlops" << std::endl;
   std::cerr << "\t" << gpusolve::util::MemorySize(calcPOps * sizeof(float) * 41) / timeCalcP
      << "/s" << std::endl;

   std::cerr << "CalcVelocities: " << (timeCalcVelocities / it) << " s" << std::endl;
   std::cerr << "\t" << ((calcVelocitiesOps * 86) / timeCalcVelocities) * 1e-9 << " GFlops";
   std::cerr << std::endl;
   std::cerr << "\t" << gpusolve::util::MemorySize(calcVelocitiesOps * sizeof(float) * 72) /
      timeCalcVelocities << "/s" << std::endl;

   std::cerr << "ReduceVelocities: " << (timeReduceVelocities / it) << " s" << std::endl;
   std::cerr << "UpdateVelocities: " << (timeUpdateVelocities / it) << " s" << std::endl;
   std::cerr << "ReduceResidual: " << (timeReduceResidual / (it / residualSpacing)) << " s"
      << std::endl;

   double time = timeCalcPhi + timeCalcP + timeCalcVelocities + timeReduceVelocities +
      timeUpdateVelocities + timeReduceResidual + timeReduceMaxVelocity;

   std::cerr << "iteration step: " << (time / it) << " s" << std::endl;
#endif

   lastPrecision_ = residual;
   lastIterations_ = it;

   pe_LOG_DEBUG_SECTION( log ) {
      if( converged )
         log << "     Solved the complementarity problem in " << it << " PGS iterations.\n";
      else
         log << pe_YELLOW << "     WARNING: Did not solve the complementarity problem within accuracy. (" << lastPrecision_ << ")" << pe_OLDCOLOR << "\n";
   }

   return converged;
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#undef PE_STRINGIZE

#endif
