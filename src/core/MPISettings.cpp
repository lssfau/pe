//=================================================================================================
/*!
 *  \file src/core/MPISettings.cpp
 *  \brief Source file for the MPI settings
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


//*************************************************************************************************
// Platform/compiler-specific includes
//*************************************************************************************************

#include <pe/system/WarningDisable.h>


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <stdexcept>
#include <pe/core/ExclusiveSection.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/util/Types.h>


namespace pe {

//=================================================================================================
//
//  DEFINITION AND INITIALIZATION OF THE STATIC MEMBER VARIABLES
//
//=================================================================================================

bool     MPISettings::active_  ( false );
bool     MPISettings::parallel_( false );
int      MPISettings::size_( 1 );
int      MPISettings::rank_( 0 );
int      MPISettings::root_( 0 );
size_t   MPISettings::bits_( 0 );
MPI_Comm MPISettings::comm_;




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Setting the root process of the rigid body physics engine.
 *
 * \param rootProcess The new root process.
 * \return void
 * \exception std::invalid_argument Invalid root process.
 * \exception std::runtime_error Invalid function call inside exclusive section.
 *
 * This function sets the root processes of the MPI parallel simulation. In case \a rootProcess
 * is larger or equal than the total number of processes, a \a std::invalid_argument exception
 * is thrown.
 *
 * \b Note: This function must not be called from inside an exclusive section. Calling this
 * function inside an exclusive section results in a \a std::runtime_error exception!
 */
void MPISettings::root( int rootProcess )
{
   // Checking the root process
   if( rootProcess >= size_ )
      throw std::invalid_argument( "Invalid root process" );

   // Checking if the function is called inside an exclusive section
   if( size_ > 1 && ExclusiveSection::isActive() )
      throw std::runtime_error( "Invalid function call inside exclusive section" );

   // Activating the MPI system
   if( !active_ ) activate();

   // Setting the root process
   root_ = rootProcess;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Setting the MPI communicator of the rigid body physics engine.
 *
 * \param communicator The new MPI communicator.
 * \return void
 * \exception std::runtime_error Invalid function call inside exclusive section.
 *
 * This function sets the MPI communicator of the MPI parallel simulation. Additionally,
 * it reevaluates the total number of processes and the rank of this processes for the
 * new communicator.
 *
 * \b Note: This function must not be called from inside an exclusive section. Calling this
 * function inside an exclusive section results in a \a std::runtime_error exception!
 */
void MPISettings::comm( MPI_Comm communicator )
{
   // Checking if the function is called inside an exclusive section
   if( size_ > 1 && ExclusiveSection::isActive() )
      throw std::runtime_error( "Invalid function call inside exclusive section" );

   // Setting the communicator
   comm_ = communicator;

#if HAVE_MPI
   // Checking if the MPI system has been initialized
   int initialized( 0 );
   MPI_Initialized( &initialized );

   // Reinitializing the total number of processes and the rank of this process
   if( initialized ) {
      parallel_ = true;
      MPI_Comm_size( comm_, &size_ );  // Estimating the total number of MPI processes
      MPI_Comm_rank( comm_, &rank_ );  // Estimating the rank of this process
   }

   // Estimating the number of bits necessary to represent the largest MPI rank
   // Though the largest rank is size_-1 we also need an escape value for global bodies
   bits_ = 0;
   int tmp( size_ );
   while( tmp ) {
      tmp >>= 1;
      ++bits_;
   }
#endif

   active_ = true;
}
//*************************************************************************************************




//=================================================================================================
//
//  UTILITY FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Initialization of the MPI settings.
 *
 * \return void
 *
 * This function is automatically called prior to the first access to any of the MPI settings.
 * It handles the initialization of the MPI system of the \b pe physics engine. In case the MPI
 * communication system has been initialized via the MPI_Init() function, the number of processes
 * and the current rank of this process are initialized accordingly. The initial communicator
 * for the \b pe engine is \a MPI_COMM_WORLD. In case the MPI communication system has not been
 * initialized it is assumed that no MPI parallelization is required, which results in a default
 * initialization of the number of processes (default: 1) and the current rank (default: 0).
 */
void MPISettings::activate()
{
#if HAVE_MPI
   // Default initialization of the communicator
   comm_ = MPI_COMM_WORLD;

   // Checking if the MPI system has been initialized
   int initialized( 0 );
   MPI_Initialized( &initialized );

   // Initializing the total number of processes and the rank of this process
   if( initialized ) {
      parallel_ = true;
      MPI_Comm_size( comm_, &size_ );  // Estimating the total number of MPI processes
      MPI_Comm_rank( comm_, &rank_ );  // Estimating the rank of this process
   }

   // Estimating the number of bits necessary to represent the largest MPI rank
   // Though the largest rank is size_-1 we also need an escape value for global bodies
   int tmp( size_ );
   while( tmp ) {
      tmp >>= 1;
      ++bits_;
   }
#endif

   active_ = true;
}
//*************************************************************************************************

} // namespace pe
