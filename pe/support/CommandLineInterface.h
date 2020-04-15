//=================================================================================================
/*!
 *  \file pe/support/CommandLineInterface.h
 *  \brief Implementation of a default command-line interface for pe examples.
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

#ifndef _PE_UTIL_COMMANDLINEINTERFACE_H_
#define _PE_UTIL_COMMANDLINEINTERFACE_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/CollisionSystem.h>
#include <pe/util/Random.h>
#include <boost/program_options.hpp>


namespace pe {

using namespace ::boost::program_options;


//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Implementation of a default command-line interface for pe examples.
 * \ingroup util
 *
 * The CommandLineInterface class parses the arguments given on the command-line interface and
 * extracts several standard switches.
 */
class PE_PUBLIC CommandLineInterface
{
private:
   static std::auto_ptr<CommandLineInterface> instance_; //!< Singleton instance.

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   options_description desc_;
   variables_map vm_;
   //@}
   //**********************************************************************************************

   //**Constructors********************************************************************************
   /*!\name Constructors */
   //@{
   CommandLineInterface();
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   // No explicitly declared destructor.
   //**********************************************************************************************
   
   //**Helper Functions****************************************************************************
   /*!\name Helper Functions */
   //@{
   template<typename T>
   void evaluateContactSolverOptions( T& contactSolver );
   template<typename T>
   void addContactSolverOptions();
   //@}
   //**********************************************************************************************


public:

   //**Get functions****************************************************************************
   /*!\name Get functions */
   //@{
   options_description& getDescription();
   const options_description& getDescription() const;
   variables_map& getVariablesMap();
   const variables_map& getVariablesMap() const;
   static CommandLineInterface& getInstance();
   //@}
   //**********************************************************************************************

   //**CLI option functions****************************************************************************
   /*!\name CLI option functions */
   //@{
   void parse(int argc, char* argv[]);
   void evaluateOptions();
   //@}
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  HELPER FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Helper function to evaluate options for the contact solvers in general.
 *
 * \param contactSolver A reference to the current contact solver.
 * \return void
 */
template<typename T>
inline void CommandLineInterface::evaluateContactSolverOptions( T& /* contactSolver */ ) {
}
//*************************************************************************************************

//*************************************************************************************************
/*!\brief Helper function to register options for the contact solvers in general.
 *
 * \return void
 */
template<typename T>
inline void CommandLineInterface::addContactSolverOptions() {
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*! \brief Returns a read-write reference to the CLI option descriptions.
 *
 * \return A read-write reference to the CLI option descriptions.
 */
inline options_description& CommandLineInterface::getDescription() {
   return desc_;
}
//*************************************************************************************************

//*************************************************************************************************
/*! \brief Returns a read-only reference to the CLI option descriptions.
 *
 * \return A read-only reference to the CLI option descriptions.
 */
inline const options_description& CommandLineInterface::getDescription() const {
   return desc_;
}
//*************************************************************************************************

//*************************************************************************************************
/*! \brief Returns a read-write reference to the CLI option values.
 *
 * \return A read-write reference to the CLI option values.
 */
inline variables_map& CommandLineInterface::getVariablesMap() {
   return vm_;
}
//*************************************************************************************************

//*************************************************************************************************
/*! \brief Returns a read-only reference to the CLI option values.
 *
 * \return A read-only reference to the CLI option values.
 */
inline const variables_map& CommandLineInterface::getVariablesMap() const {
   return vm_;
}
//*************************************************************************************************

//*************************************************************************************************
/*! \brief Returns a read-write reference to a singleton object of the CLI class.
 *
 * \return A read-write reference to a singleton object of the CLI class.
 */
inline CommandLineInterface& CommandLineInterface::getInstance() {
   if( !instance_.get() )
      instance_.reset( new CommandLineInterface() );

   return *instance_;
}
//*************************************************************************************************

} // namespace pe

#endif

