//=================================================================================================
/*!
 *  \file pe/core/MPISettings.h
 *  \brief Header file for the MPI settings
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

#ifndef _PE_CORE_MPISETTINGS_H_
#define _PE_CORE_MPISETTINGS_H_


//*************************************************************************************************
// Includes
//*************************************************************************************************

#include <pe/core/MPI.h>
#include <pe/util/NonCreatable.h>
#include <pe/util/Types.h>


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
 * The MPISettings class represents the current settings of the MPI communication system.
 * It offers the functionality to acquire the current settings (as for instance the total
 * number of MPI processes and the current rank of this process) for all modules of the
 * physics engine. Although it is closely coupled to the MPISystem class, it is implemented
 * as a separate class to minimize coupling for other classes.
 */
class PE_PROTECTED MPISettings : private NonCreatable
{
public:
   //**Get functions*******************************************************************************
   /*!\name Get functions */
   //@{
   static inline bool     isParallel();
   static inline int      size();
   static inline int      rank();
   static inline int      root();
   static inline size_t   bits();
   static inline MPI_Comm comm();
   //@}
   //**********************************************************************************************

private:
   //**Set functions*******************************************************************************
   /*!\name Set functions */
   //@{
   static void root( int      rootProcess  );
   static void comm( MPI_Comm communicator );
   //@}
   //**********************************************************************************************

   //**Utility functions***************************************************************************
   /*!\name Utility functions */
   //@{
   static void activate();
   //@}
   //**********************************************************************************************

   //**Member variables****************************************************************************
   /*!\name Member variables */
   //@{
   static bool     active_;    //!< Activation flag.
   static bool     parallel_;  //!< MPI parallel flag.
                               /*!< The parallel flag indicates whether or not the simulation
                                    is MPI parallel (i.e., whether the MPI_Init() function has
                                    been called). In case the simulation runs in parallel (even
                                    if it is only a single MPI process), the flag is set to
                                    \a true, otherwise it is set to \a false. */
   static int      size_;      //!< The total number of active MPI processes.
   static int      rank_;      //!< The current rank of the local MPI process.
   static int      root_;      //!< The root MPI process.
   static size_t   bits_;      //!< The number of bits necessary to represent the largest rank.
   static MPI_Comm comm_;      //!< The MPI communicator for the physics engine.
   //@}
   //**********************************************************************************************

   //**Friend declarations*************************************************************************
   /*! \cond PE_INTERNAL */
   friend class MPISystem;
   /*! \endcond */
   //**********************************************************************************************
};
//*************************************************************************************************




//=================================================================================================
//
//  GET FUNCTIONS
//
//=================================================================================================

//*************************************************************************************************
/*!\brief Returns whether or not the simulation is MPI parallel.
 *
 * \return \a true in case the simulation is MPI parallel, \a false if not.
 *
 * This function returns whether or not the simulation is MPI parallel. In case the MPI_Init()
 * function has been called, the function returns \a true, otherwise it returns \a false.
 */
inline bool MPISettings::isParallel()
{
   if( !active_ ) activate();
   return parallel_;
}
//*************************************************************************************************


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
inline int MPISettings::size()
{
   if( !active_ ) activate();
   return size_;
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
inline int MPISettings::rank()
{
   if( !active_ ) activate();
   return rank_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the root process of the rigid body physics engine.
 *
 * \return The current root process.
 */
inline int MPISettings::root()
{
   if( !active_ ) activate();
   return root_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the number of bits necessary to represent the largest MPI rank.
 *
 * \return The number of bits necessary to represent the largest MPI rank.
 *
 * This function returns the minimum number of bits that are necessary to represent the largest
 * possible MPI rank of the active simulation.
 */
inline size_t MPISettings::bits()
{
   if( !active_ ) activate();
   return bits_;
}
//*************************************************************************************************


//*************************************************************************************************
/*!\brief Returns the MPI communicator of the rigid body physics engine.
 *
 * \return The current MPI communicator.
 */
inline MPI_Comm MPISettings::comm()
{
   if( !active_ ) activate();
   return comm_;
}
//*************************************************************************************************

} // namespace pe

#endif
