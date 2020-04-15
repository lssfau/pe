//=================================================================================================
/*!
 *  \file pe/core/MPISystem.h
 *  \brief Header file for the MPI communication system
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

#ifndef _PE_CORE_MPISYSTEM_H_
#define _PE_CORE_MPISYSTEM_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <iosfwd>
#include <pe/core/CollisionSystem.h>
#include <pe/core/Configuration.h>
#include <pe/core/MPI.h>
#include <pe/core/MPISettings.h>
#include <pe/core/MPISystemID.h>
#include <pe/core/ProcessManager.h>
#include <pe/core/domaindecomp/ProcessStorage.h>
#include <pe/core/Types.h>
#include <pe/util/logging/Logger.h>
#include <pe/util/singleton/Singleton.h>


namespace pe {

//=================================================================================================
//
//  CLASS DEFINITION
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Settings of the MPI communication system.
 * \ingroup mpi
 *
 * The MPISystem class represents the MPI communication system of the \b pe physics engine.
 * It offers the functionality to acquire the current settings for the MPI communication as
 * for instance the total number of MPI processes, the rank of the local process, the rank
 * of the root process and the MPI communicator.
 */
class PE_PUBLIC MPISystem : public ProcessManager
                , private Singleton< MPISystem,logging::Logger,CollisionSystem<Config> >
{
private:
   //**Type definitions****************************************************************************
   typedef ProcessStorage<Config>  PS;  //!< Process storage of the MPI system.
   //**********************************************************************************************

   //**Constructor*********************************************************************************
   /*!\name Constructor */
   //@{
   explicit MPISystem();
   //@}
   //**********************************************************************************************

public:
   //**Destructor**********************************************************************************
   /*!\name Destructor */
   //@{
   virtual ~MPISystem();
   //@}
   //**********************************************************************************************

   //**Type definitions****************************************************************************
   typedef PS::Processes           Processes;      //!< Process container.
   typedef PS::SizeType            SizeType;       //!< Process count of the MPI system.
   typedef PS::Iterator            Iterator;       //!< Iterator over the processes.
   typedef PS::ConstIterator       ConstIterator;  //!< Constant iterator over the processes.
   //**********************************************************************************************

   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   inline int           getSize() const;
   inline int           getRank() const;
   inline int           getRoot() const;
   inline MPI_Comm      getComm() const;

   inline Iterator      begin();
   inline ConstIterator begin() const;
   inline Iterator      end();
   inline ConstIterator end()   const;
   //@}
   //**********************************************************************************************

   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   inline void setRoot( int      rootProcess  );
   inline void setComm( MPI_Comm communicator );
   //@}
   //**********************************************************************************************

   //**Process manager functions*******************************************************************
   /*!\name Process manager functions */
   //@{
   virtual void add   ( ProcessID process );
   virtual void remove( ProcessID process );
   //@}
   //**********************************************************************************************

   //**Debugging functions*************************************************************************
   /*!\name Debugging functions */
   //@{
   void checkProcesses() const;
   //@}
   //**********************************************************************************************

   //**Output functions****************************************************************************
   /*!\name Output functions */
   //@{
   void print( std::ostream& os ) const;
   //@}
   //**********************************************************************************************

private:
   //**Debugging functions*************************************************************************
   /*!\name Debugging functions */
   //@{
   bool isSymmetric( const std::vector<int>& adjlists, const std::vector<int>& adjoffsets, std::vector<bool>& visited, int node ) const;
   void extractHalfSpacesFromProcessGeometry( std::list< std::pair<Vec3, real> >& ret, ProcessGeometry* p ) const;
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend MPISystemID theMPISystem();
   pe_BEFRIEND_SINGLETON;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  WORLD SETUP FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\name MPI communication system setup functions */
//@{
inline MPISystemID theMPISystem();
//@}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns a handle to the MPI communication system.
 * \ingroup mpi
 *
 * \return Handle to the MPI communication system.
 * \exception std::runtime_error MPI system is not initialized.
 *
 * This function returns a handle to the MPI communication system of the \b pe physics
 * engine. This handle can be used to configure the communication system or to acquire
 * the current settings. The first call to this function will activate the communication
 * system. However, the function expects that MPI has already been properly initialized
 * (e.g. via the MPI_Init() or any similar function). In case MPI was not initialized,
 * a \a std::runtime_error exception is thrown.
 */
inline MPISystemID theMPISystem()
{
   return MPISystem::instance();
}
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns the total number of MPI processes.
 *
 * \return The total number of MPI processes.
 *
 * This function returns the total number of MPI processes within the active MPI communicator.
 * Note that this function requires the MPI system to be initialized via MPI_Init() in order
 * to be able to return the correct number of processes. Otherwise the function assumes that
 * this is a nonparallel setup and returns 1.
 */
inline int MPISystem::getSize() const
{
   return MPISettings::size();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the current rank of the local MPI process.
 *
 * \return The current rank of the local MPI process.
 *
 * This function returns the current rank of the local MPI process . Note that this function
 * requires the MPI system to be initialized via MPI_Init() in order to be able to return the
 * correct number of processes. Otherwise the function assumes that this is a nonparallel
 * setup and returns 0.
 */
inline int MPISystem::getRank() const
{
   return MPISettings::rank();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the root process of the rigid body physics engine.
 *
 * \return The current root process.
 */
inline int MPISystem::getRoot() const
{
   return MPISettings::root();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the MPI communicator of the rigid body physics engine.
 *
 * \return The current MPI communicator.
 */
inline MPI_Comm MPISystem::getComm() const
{
   return MPISettings::comm();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first connected remote MPI process.
 *
 * \return Iterator to the first remote process.
 */
inline MPISystem::Iterator MPISystem::begin()
{
   // WARNING: Using friend relationship to get non-constant iterator from process storage.
   return theCollisionSystem()->processstorage_.begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator to the first connected remote MPI process.
 *
 * \return Iterator to the first remote process.
 */
inline MPISystem::ConstIterator MPISystem::begin() const
{
   return theCollisionSystem()->getProcessStorage().begin();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last connected remote MPI process.
 *
 * \return Iterator just past the last remote process.
 */
inline MPISystem::Iterator MPISystem::end()
{
   // WARNING: Using friend relationship to get non-constant iterator from process storage.
   return theCollisionSystem()->processstorage_.end();
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns an iterator just past the last connected remote MPI process.
 *
 * \return Iterator just past the last remote process.
 */
inline MPISystem::ConstIterator MPISystem::end() const
{
   return theCollisionSystem()->getProcessStorage().end();
}
//*************************************************************************************************




//=================================================================================================
//
//  SET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Sets the root process of the rigid body physics engine.
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
inline void MPISystem::setRoot( int rootProcess )
{
   MPISettings::root( rootProcess );
}
//************************************************************************************************


//*************************************************************************************************
/*!\brief Sets the MPI communicator of the rigid body physics engine.
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
inline void MPISystem::setComm( MPI_Comm communicator )
{
   MPISettings::comm( communicator );
}
//*************************************************************************************************




//=================================================================================================
//
//  GLOBAL OPERATORS
//
//=================================================================================================

//*************************************************************************************************
/*!\name MPISystem operators */
//@{
std::ostream& operator<<( std::ostream& os, const MPISystem& ms );
std::ostream& operator<<( std::ostream& os, const MPISystemID& ms );
std::ostream& operator<<( std::ostream& os, const ConstMPISystemID& ms );
//@}
//*************************************************************************************************

} // namespace pe

#endif
