//=================================================================================================
/*!
 *  \file pe/core/response/PolyhedralFrictionSolver.h
 *  \brief Contact solver framework for solving frictional systems with a polyhedral friction cone
 *
 *  Copyright (C) 2006-2007 Klaus Iglberger
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

#ifndef _PE_CORE_RESPONSE_POLYHEDRALFRICTIONSOLVER_H_
#define _PE_CORE_RESPONSE_POLYHEDRALFRICTIONSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/contact/ContactVector.h>
#include <pe/core/response/constraints/SolverRestriction.h>
#include <pe/core/response/ConstructorDriver.h>
#include <pe/core/response/ContactSolver.h>
#include <pe/core/response/PolyhedralFrictionConstructor.h>
#include <pe/core/TimeStep.h>
#include <pe/math/problems/LCP.h>
#include <pe/math/solvers/Types.h>
#include <pe/math/Vector3.h>
#include <pe/system/Collisions.h>
#include <pe/system/LCPConfig.h>
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
/*!\brief Contact solver framework for solving polyhedral frictional systems.
 * \ingroup collision_response
 *
 * The polyhedral frictional contact solver constructs a matrix with a \f$J^T M^{-1} J\f$ type
 * submatrix and some additional constraints to enforce that the computed contact forces remain
 * inside the friction cone and are dissipative as part of a standard (non-symmetric) LCP. The
 * complementarity problem is then solved using the supplied solver. Afterwards the computed
 * contact forces are applied.
 */
template< typename C              // Type of the configuration
        , typename CS             // Type of the complementarity solver
        , typename U1=NullType >  // Unused auxiliary template parameter
class PolyhedralFrictionSolver : public ContactSolver<C>
{
private:
   //**Type definitions****************************************************************************
   typedef C                                  Config;          //!< Type of the configuration.
   typedef PolyhedralFrictionSolver<C,CS,U1>  This;            //!< Type of this PolyhedralFrictionSolver instance.
   typedef typename Config::ContactType       ContactType;     //!< Type of the contacts.
   typedef typename Config::ContactID         ContactID;       //!< Handle to a contact.
   typedef typename Config::ConstContactID    ConstContactID;  //!< Handle to a contact.
   typedef ContactVector<ContactType>         Contacts;        //!< Type of the contact container.

   //! Type of the complementarity solver for the polyhedral friction system.
   typedef CS  Solver;

   //! Valid complementarity solvers for the polyhedral friction problem.
   typedef pe_TYPELIST_1( solvers::Lemke )  ValidSolvers;
   //**********************************************************************************************

public:
   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   explicit PolyhedralFrictionSolver();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~PolyhedralFrictionSolver();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   virtual size_t getMaxIterations()  const;
   virtual size_t getLastIterations() const;
   virtual real   getLastPrecision()  const;
   virtual real   getThreshold()      const;
   //@}
   //**********************************************************************************************

   //**Set functions***************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setMaxIterations( size_t maxIterations );
   virtual void setThreshold    ( real threshold );
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
   Solver solver_;  //!< TODO
   //@}
   //**********************************************************************************************

   //**Compile time checks*************************************************************************
   /*! \cond PE_INTERNAL */
   pe_CONSTRAINT_MUST_BE_SAME_TYPE( U1, NullType );
   pe_CONSTRAINT_COMPLEMENTARITY_SOLVER_RESTRICTION( Solver, ValidSolvers );
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
/*!\brief Default constructor for the PolyhedralFrictionSolver class.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
PolyhedralFrictionSolver<C,CS,U1>::PolyhedralFrictionSolver()
   : ContactSolver<C>()  // Initialization of the base class
{}
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the PolyhedralFrictionSolver class.
 */
template< typename C     // Type of the configuration
        , typename CS    // Type of the complementarity solver
        , typename U1 >  // Unused auxiliary template parameter
PolyhedralFrictionSolver<C,CS,U1>::~PolyhedralFrictionSolver()
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
size_t PolyhedralFrictionSolver<C,CS,U1>::getMaxIterations() const
{
   return solver_.getMaxIterations();
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
size_t PolyhedralFrictionSolver<C,CS,U1>::getLastIterations() const
{
   return solver_.getLastIterations();
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
real PolyhedralFrictionSolver<C,CS,U1>::getLastPrecision() const
{
   return solver_.getLastPrecision();
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
real PolyhedralFrictionSolver<C,CS,U1>::getThreshold() const
{
   return solver_.getThreshold();
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
void PolyhedralFrictionSolver<C,CS,U1>::setMaxIterations( size_t maxIterations )
{
   solver_.setMaxIterations( maxIterations );
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
void PolyhedralFrictionSolver<C,CS,U1>::setThreshold( real threshold )
{
   solver_.setThreshold( threshold );
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
void PolyhedralFrictionSolver<C,CS,U1>::resolveContacts( const Contacts& contacts )
{
   using namespace pe::response::lcp;

   if( contacts.isEmpty() || !contacts.isActive() ) {
      pe_LOG_DEBUG_SECTION( log ) {
         log << "   Skipping inactive/empty batch";
      }
      return;
   }

   pe_LOG_DEBUG_SECTION( log ) {
      log << "   Resolving the " << contacts.size() << " polyhedral friction contact(s)"
          << " (";
      for( typename Contacts::ConstIterator c=contacts.begin(); c!=contacts.end(); ++c )
         log << " " << c->getID();
      log << " )...";
   }

   // Constructing the polyhedral friction LCP system
   LCP lcp;
   PolyhedralFrictionConstructor<C> ctor( lcp );
   ConstructorDriver<C>::construct( contacts, ctor );

   // Solving the polyhedral friction LCP system
   solver_.solve( lcp );

   // Apply the contact forces
   const size_t N    ( contacts.size() );
   const real   dtinv( real(1)/TimeStep::size() );

   for( size_t i=0; i<N; ++i )
   {
      ConstContactID c( contacts[i] );

      // Calculate the total force acting at contact i
      Vec3 contactForce( contacts[i]->getNormal() * lcp.x_[(facets+1)*i] );
      for( size_t j=0; j<facets; ++j ) {
         contactForce += c->getTangent(j) * lcp.x_[(facets+1)*i+1+j];
      }
      contactForce *= dtinv;

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
