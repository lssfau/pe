//=================================================================================================
/*!
 *  \file pe/core/ExclusiveSection.h
 *  \brief Exclusive section for a single process in a parallel environment
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

#ifndef _PE_CORE_EXCLUSIVESECTION_H_
#define _PE_CORE_EXCLUSIVESECTION_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <new>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Exclusive section for a single process in a parallel environment.
 * \ingroup mpi
 *
 * The ExclusiveSection class is an auxiliary helper class for the \a pe_EXCLUSIVE_SECTION
 * macro. It provides the functionality to detect whether an exclusive section is active, i.e.
 * if the currently executed code is inside an exclusive section. This feature can be used to
 * detect possible deadlocks in functions that assume that all MPI processes are involved in
 * the computation.
 */
class PE_PUBLIC ExclusiveSection
{
public:
   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   ExclusiveSection( int rank );
   // No explicitly declared copy constructor.
   //@}
   //**********************************************************************************************

   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   inline ~ExclusiveSection();
   //@}
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline bool isActive();
   //@}
   //**********************************************************************************************

   //**Conversion operator*************************************************************************
   /*!\name Conversion operator */
   //@{
   inline operator bool() const;
   //@}
   //**********************************************************************************************

private:
   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static bool active_;  //!< Activity flag for the exclusive section.
                         /*!< In case an exclusive section is active (i.e. the currently executed
                              code is inside an exclusive section), the flag is set to \a true,
                              otherwise it is \a false. */
   //@}
   //**********************************************************************************************

   //**Forbidden operations************************************************************************
   /*!\name Forbidden operations */
   //@{
   ExclusiveSection& operator=( const ExclusiveSection& );

   void* operator new  ( std::size_t );
   void* operator new[]( std::size_t );
   void* operator new  ( std::size_t, const std::nothrow_t& ) PE_NOTHROW;
   void* operator new[]( std::size_t, const std::nothrow_t& ) PE_NOTHROW;

   void operator delete  ( void* ) PE_NOTHROW;
   void operator delete[]( void* ) PE_NOTHROW;
   void operator delete  ( void*, const std::nothrow_t& ) PE_NOTHROW;
   void operator delete[]( void*, const std::nothrow_t& ) PE_NOTHROW;
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
/*!\brief Destructor of the ExclusiveSection class.
 */
inline ExclusiveSection::~ExclusiveSection()
{
   active_ = false;  // Resetting the activity flag
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether an exclusive section is active or not.
 *
 * \return \a true if an exclusive section is active, \a false if not.
 */
inline bool ExclusiveSection::isActive()
{
   return active_;
}
//*************************************************************************************************




//=================================================================================================
//
//  CONVERSION OPERATOR
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Conversion operator to \a bool.
 *
 * The conversion operator returns \a true in case an exclusive section is active and \a false
 * otherwise.
 */
inline ExclusiveSection::operator bool() const
{
   return active_;
}
//*************************************************************************************************








//=================================================================================================
//
//  EXCLUSIVE SECTION MACROS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Exclusive section for a single process in a parallel environment.
 * \ingroup mpi
 *
 * This macro provides the option to start an exclusive section for a single process in a MPI
 * parallel environment. The following example demonstrates how an exclusive section is used:

   \code
   int main( int argc, char** argv )
   {
      // Initialization of the MPI system
      // The MPI system must be initialized before any pe functionality is used. It is
      // recommended to make MPI_Init() the very first call of the main function.
      MPI_Init( &argc, &argv );

      // Parallel region
      // This code is executed by all processes of the parallel simulation.
      ...

      // Exclusive section
      // This section is only executed by process 0 and skipped by all other processes. This
      // can for example be used to perform special setups of checks of the parallel simulation
      // environment.
      pe_EXCLUSIVE_SECTION( 0 ) {
         ...
      }

      // Second parallel region
      // This code is again executed by all processes of the parallel simulation.
      ...

      // Finalizing the MPI system
      // The MPI system must be finalized after the last pe functionality has been used. It
      // is recommended to make MPI_Finalize() the very last call of the main function.
      MPI_Finalize();
   }
   \endcode

 * Using the pe_EXCLUSIVE_SECTION macro has a similar effect as using an if-condition to test
 * the rank of the MPI process:

   \code
   if( rank == ... ) {
      // Exclusive code for a specific MPI process
   }
   \endcode

 * However, it is strongly recommended to use the pe::pe_EXCLUSIVE_SECTION instead since only
 * in this case the \b pe is able to detect certain errors.\n
 * Note that starting an exclusive section for an invalid MPI rank (i.e. a negative rank or
 * a rank greater-or-equal than the actual total number of MPI processes) will result in a
 * \a std::invalid_argument exception.
 *
 * The following actions must not be performed inside an exclusive section:
 * - starting another exclusive section (i.e. a nested exclusive section); this results in a
 *   \a std::runtime_error exception)
 * - starting a process serialization (see pe::pe_SERIALIZATION)
 * - the setup of infinite rigid bodies (as for instance the Plane geometric primitive)
 * - translating or rotating global rigid bodies (i.e. rigid bodies created inside a
 *   pe::pe_GLOBAL_SECTION) via setPosition(), setOrientation(), translate(), or any rotate()
 *   function
 *
 * The following functions must not be called inside an exclusive section:
 * - the pe::World::simulationStep() function
 * - the pe::World::run() function
 * - the pe::World::synchronize() function
 * - the pe::MPISystem::checkProcesses() function
 * - any visualization functionality (e.g. the pe::povray::activateWriter() function or the
 *   pe::povray::Writer::writeFile() function)
 */
#define pe_EXCLUSIVE_SECTION( RANK ) \
   if( pe::ExclusiveSection exclusiveSection = (RANK) )
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Else-branch of an exclusive section.
 * \ingroup mpi
 *
 * This macro represents the starting point of an else-branch for an exclusive section initiated
 * via the pe_EXCLUSIVE_SECTION macro. This macro can be used to in combination with an exclusive
 * section to exclude a single process from the execution of the code contained in the else branch:

   \code
   pe_EXCLUSIVE_SECTION( 0 )
   {
      // This code is exclusively executed by process 0 and skipped by all other processes
      ...
   }
   pe_EXCLUSIVE_ELSE
   {
      // This code is executed by all processes except process 0
      ...
   }
   \endcode
 */
#define pe_EXCLUSIVE_ELSE \
   else
//*************************************************************************************************

} // namespace pe

#endif
