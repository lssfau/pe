//=================================================================================================
/*!
 *  \file pe/core/response/BoxFrictionSolver.h
 *  \brief Contact solver framework for solving box frictional multibody systems
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

#ifndef _PE_CORE_RESPONSE_BOXFRICTIONSOLVER_H_
#define _PE_CORE_RESPONSE_BOXFRICTIONSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/contact/ContactVector.h>
#include <pe/core/response/constraints/SolverRestriction.h>
#include <pe/core/response/ConstructorDriver.h>
#include <pe/core/response/ContactSolver.h>
#include <pe/core/response/FrictionConstructor.h>
#include <pe/core/response/FrictionlessConstructor.h>
#include <pe/core/TimeStep.h>
#include <pe/math/problems/BoxLCP.h>
#include <pe/math/problems/LCP.h>
#include <pe/math/solvers/CPG.h>
#include <pe/math/solvers/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Precision.h>
#include <pe/util/constraints/SameType.h>
#include <pe/util/logging/DebugSection.h>
#include <pe/util/NullType.h>
#include <pe/util/TypeList.h>
#include <pe/util/Types.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Contact solver framework for solving box frictional multibody systems.
 * \ingroup collision_response
 *
 * The box frictional contact solver constructs a \f$J^T M^{-1} J\f$ type of matrix as part
 * of a box LCP problem. The complementarity problem is then solved using the supplied solver.
 * Afterwards the computed contact forces are applied.\n
 * Box friction is a simplification of a general friction cone, where the contact forces are
 * constrained to reside within boxes. The advantage over conventional cone friction is that
 * the constraints on the friction forces do not depend on the normal contact force. The
 * resulting system is a monotone QP with box constraints and this leads to simpler, more
 * efficient and a larger number of available solvers. The drawback is that the normal forces
 * need to be estimated in order that the friction box approximates the cone well.
 */
template< typename C              // Type of the configuration
        , typename CS             // Type of the complementarity solver
        , typename U1=NullType >  // Unused auxiliary template parameter
class BoxFrictionSolver : public ContactSolver<C>
{
private:
   //**Type definitions****************************************************************************
   typedef C                                Config;          //!< Type of the configuration.
   typedef BoxFrictionSolver<C,CS,U1>       This;            //!< Type of this BoxFrictionSolver instance.
   typedef typename Config::ContactType     ContactType;     //!< Type of the contacts.
   typedef typename Config::ContactID       ContactID;       //!< Handle to a contact.
   typedef typename Config::ConstContactID  ConstContactID;  //!< Handle to a contact.
   typedef ContactVector<ContactType>       Contacts;        //!< Type of the contact container.

   //! Type of the complementarity solver for the frictionless system.
   typedef solvers::CPG  FrictionlessSolver;

   //! Type of the complementarity solver for the box friction system.
   typedef CS  FrictionSolver;

   //! Valid complementarity solvers for the polyhedral friction problem.
   typedef pe_TYPELIST_2( solvers::PGS, solvers::CPG )  ValidSolvers;
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit BoxFrictionSolver( size_t solveCount=1 );
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~BoxFrictionSolver();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   virtual size_t getMaxIterations()  const;
   virtual size_t getLastIterations() const;
   virtual real   getLastPrecision()  const;
   virtual real   getThreshold()      const;

   inline  size_t getSolveCount() const;
   //@}
   //**********************************************************************************************

   //**Set functions****************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setMaxIterations( size_t maxIterations );
   virtual void setThreshold    ( real threshold );

   inline  void setSolveCount( size_t solveCount );
   //@}
   //**********************************************************************************************

   //**Solver functions****************************************************************************
   /*!\name Solver functions */
   //@{
   virtual void resolveContacts( const Contacts& contacts );
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   FrictionlessSolver frictionlessSolver_;  //!< TODO
   FrictionSolver     frictionSolver_;      //!< TODO
   size_t             solveCount_;          //!< TODO
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U1, NullType );
   pe_CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION( FrictionSolver, ValidSolvers );
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
/*!\brief Default constructor for the BoxFrictionSolver class.
 *
 * \param solveCount Number of times the box friction problem is solved.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
BoxFrictionSolver<C,CS,U1>::BoxFrictionSolver( size_t solveCount )
   : ContactSolver<C>()         // Initialization of the base class
   , solveCount_( solveCount )  // TODO
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the BoxFrictionSolver class.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
BoxFrictionSolver<C,CS,U1>::~BoxFrictionSolver()
{}
//*************************************************************************************************




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
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
size_t BoxFrictionSolver<C,CS,U1>::getMaxIterations() const
{
   return frictionSolver_.getMaxIterations();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of iterations spent in the last solution process.
 *
 * \return The number of iterations spent in the last solution process.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
size_t BoxFrictionSolver<C,CS,U1>::getLastIterations() const
{
   return frictionSolver_.getLastIterations();
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
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
real BoxFrictionSolver<C,CS,U1>::getLastPrecision() const
{
   return frictionSolver_.getLastPrecision();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the threshold that classifies a solution as good enough.
 *
 * \return The threshold for the solution quality.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
real BoxFrictionSolver<C,CS,U1>::getThreshold() const
{
   return frictionSolver_.getThreshold();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of times the box friction model is solved.
 *
 * \return The number of times the box friction model is solved.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
inline size_t BoxFrictionSolver<C,CS,U1>::getSolveCount() const
{
   return solveCount_;
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
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
void BoxFrictionSolver<C,CS,U1>::setMaxIterations( size_t maxIterations )
{
   frictionSolver_.setMaxIterations( maxIterations );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the threshold which classifies a solution as good enough.
 *
 * \param threshold The threshold for the solution quality.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
void BoxFrictionSolver<C,CS,U1>::setThreshold( real threshold )
{
   frictionSolver_.setThreshold( threshold );
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Specifies the number of times the box friction model is solved.
 *
 * \param solveCount The number of times the box friction model is solved.
 *
 * Each time the box friction system is solved, the estimate of the normal forces presumably
 * improves. By repeatedly adapting the bounds of the friction forces and resolving the system
 * a good overall approximation can be computed. If \a solveCount is set to 0, only the initial
 * solution of the frictionless system is computed. The default value is 1.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
inline void BoxFrictionSolver<C,CS,U1>::setSolveCount( size_t solveCount )
{
   solveCount_ = solveCount;
}
//*************************************************************************************************




//=================================================================================================
//
//  SOLVER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief TODO
 *
 * \param contacts TODO
 * \return void
 *
 * TODO
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
void BoxFrictionSolver<C,CS,U1>::resolveContacts( const Contacts& contacts )
{
   const size_t N    ( contacts.size()  );
   const real   dt   ( TimeStep::size() );
   const real   dtinv( real(1) / dt     );

   if( contacts.isEmpty() || !contacts.isActive() ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Skipping inactive/empty batch";
      }
      return;
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Resolving the " << contacts.size() << " box frication contact(s)"
          << " (";
      for( typename Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c )
         log << " " << c->getID();
      log << " )...";
   }

   // Constructing the box frictional LCP system
   LCP frictionlessSystem;
   FrictionlessConstructor<C> frictionlessCtor( frictionlessSystem );
   ConstructorDriver<C>::construct( contacts, frictionlessCtor );

   // Solving the frictionless LCP system
   frictionlessSolver_.solve( frictionlessSystem );

   // Constructing the box frictional LCP system
   BoxLCP system;
   FrictionConstructor<C> ctor( system );
   ConstructorDriver<C>::construct( contacts, ctor );

   // Preparing the initial solution and projection limits
   system.xmin_.resize( 3*N, false );
   system.xmax_.resize( 3*N, false );
   for( size_t i=0; i<N; ++i ) {
      system.x_[3*i  ]  = frictionlessSystem.x_[i];
      system.x_[3*i+1]  = real(0);
      system.x_[3*i+2]  = real(0);
      system.xmin_[3*i] = real(0);
      system.xmax_[3*i] = inf;
   }

   // Solving the box frictional LCP system
   for( size_t i=0; i<solveCount_; ++i )
   {
      // Adapting the friction bounds
      for( size_t j=0; j<N; ++j ) {
         ConstContactID c( contacts[j] );
         real flimit( c->getFriction() * system.x_[3*j] );
         system.xmin_[3*j+1] = system.xmin_[3*j+2] = -flimit;
         system.xmax_[3*j+1] = system.xmax_[3*j+2] = +flimit;
      }

      frictionSolver_.solve( system );
   }

   // Applying the contact forces
   for( size_t i=0; i<N; ++i )
   {
      ConstContactID c( contacts[i] );

      // Calculating the total force acting at contact i
      const Vec3 contactForce( ( c->getNormal()   * system.x_[3*i  ] +
                                 c->getTangentX() * system.x_[3*i+1] +
                                 c->getTangentY() * system.x_[3*i+2] ) * dtinv );

      pe_LOG_DEBUG_SECTION( log ) {
         log << "      Contact " << c->getID() << ": cor = " << c->getRestitution()
             << " , cof = " << c->getFriction() << "\n"
             << "         f = " << contactForce;
      }

      c->applyForce( contactForce );
   }
}
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
