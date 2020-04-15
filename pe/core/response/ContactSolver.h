//=================================================================================================
/*!
 *  \file pe/core/response/ContactSolver.h
 *  \brief Base class for all contact solver classes
 *
 *  Copyright (C) 2006-2008 Klaus Iglberger
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

#ifndef _PE_CORE_RESPONSE_CONTACTSOLVER_H_
#define _PE_CORE_RESPONSE_CONTACTSOLVER_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/contact/ContactVector.h>
#include <pe/util/NonCopyable.h>


namespace pe {

namespace response {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Base class for all contact solver classes.
 * \ingroup collision_response
 *
 * TODO
 */
template< typename C >  // Type of the configuration
class ContactSolver : private NonCopyable
{
public:
   //**Type definitions****************************************************************************
   typedef C                             Config;       //!< Type of the configuration.
   typedef typename Config::ContactType  ContactType;  //!< Type of the contacts.
   typedef ContactVector<ContactType>    Contacts;     //!< Type of the contact container.
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~ContactSolver();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   virtual size_t getMaxIterations()  const = 0;
   virtual size_t getLastIterations() const = 0;
   virtual real   getLastPrecision()  const = 0;
   virtual real   getThreshold()      const = 0;
   //@}
   //**********************************************************************************************

   //**Set functions***************************************************************************
   /*!\name Set functions */
   //@{
   virtual void setMaxIterations( size_t maxIterations ) = 0;
   virtual void setThreshold    ( real threshold ) = 0;
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   virtual void resolveContacts( const Contacts& contacts ) = 0;
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  DESTRUCTOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Destructor for the ContactSolver class.
 */
template< typename C >  // Type of the configuration
ContactSolver<C>::~ContactSolver()
{}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\fn size_t ContactSolver::getMaxIterations() const
 * \brief Returns the maximum number of iterations the solver may spend solving the problem.
 *
 * \return The maximum number of iterations spent in the solver.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn size_t ContactSolver::getLastIterations() const
 * \brief Returns the number of iterations spent in the last solution process.
 *
 * \return The number of iterations spent in the last solution process.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn real ContactSolver::getLastPrecision() const
 * \brief Returns the precision of the solution after the solution process.
 *
 * \return The precision of the solution after the solution process.
 *
 * The solver is not enforced to compute the precision after the solution. Instead it can just
 * report infinity as the last precision.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn real ContactSolver::getThreshold() const
 * \brief Returns the threshold that classifies a solution as good enough.
 *
 * \return The threshold for the solution quality.
 */
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\fn void ContactSolver::setMaxIterations( size_t maxIterations )
 * \brief Sets the maximum number of iterations the solver may spend solving the problem.
 *
 * \param maxIterations The maximum number of iterations spent in the solver.
 */
//*************************************************************************************************


//*************************************************************************************************
/*!\fn void ContactSolver::setThreshold( real threshold )
 * \brief Sets the threshold which classifies a solution as good enough.
 *
 * \param threshold The threshold for the solution quality.
 */
//*************************************************************************************************

} // namespace response

} // namespace pe

#endif
